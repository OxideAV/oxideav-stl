//! Shared deterministic fixture builders for the `profile_*` example
//! binaries.
//!
//! Each profile-target binary pulls this module in via `#[path =
//! "profile_common.rs"] mod profile_common;` and uses the helpers to
//! synthesise the same triangle soup the Criterion bench suite drives
//! its hot-path measurements against. The point of the example
//! binaries is to be **single-threaded, long-running, non-randomised**
//! drivers suitable for a profiler (`cargo flamegraph`, `samply
//! record`, `perf record`, Instruments' Time Profiler) to attribute
//! cycles to source lines without Criterion's outer-loop noise or
//! per-iteration adaptive batching getting in the way.
//!
//! There is no committed binary corpus and no `docs/` traffic — each
//! generator is seeded from a fixed constant so a flamegraph from one
//! run matches a flamegraph from another bit-for-bit.

#![allow(dead_code)]

use oxideav_mesh3d::{Indices, Mesh, Node, Primitive, Scene3D, Topology};

/// Cheap deterministic xorshift32. Same generator the bench suite's
/// `benches/common/mod.rs` uses, so a profile target's input is
/// byte-identical to the matching bench's input at the same triangle
/// count.
pub fn xorshift32(state: &mut u32) -> u32 {
    *state ^= *state << 13;
    *state ^= *state >> 17;
    *state ^= *state << 5;
    *state
}

pub fn norm_f32(state: &mut u32) -> f32 {
    let r = xorshift32(state) >> 8; // 24 bits of entropy
    (r as f32) / ((1u32 << 24) as f32)
}

pub fn synth_scene_unindexed(n_tris: usize) -> Scene3D {
    let mut state: u32 = 0x6f37_19c4;
    let mut positions = Vec::with_capacity(n_tris * 3);
    let mut normals = Vec::with_capacity(n_tris * 3);
    for _ in 0..n_tris {
        let v0 = [
            norm_f32(&mut state),
            norm_f32(&mut state),
            norm_f32(&mut state),
        ];
        let v1 = [
            norm_f32(&mut state),
            norm_f32(&mut state),
            norm_f32(&mut state),
        ];
        let v2 = [
            norm_f32(&mut state),
            norm_f32(&mut state),
            norm_f32(&mut state),
        ];
        let n = [
            norm_f32(&mut state),
            norm_f32(&mut state),
            norm_f32(&mut state),
        ];
        positions.push(v0);
        positions.push(v1);
        positions.push(v2);
        normals.push(n);
        normals.push(n);
        normals.push(n);
    }
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = positions;
    prim.normals = Some(normals);
    let mesh = Mesh::new(None).with_primitive(prim);
    let mut scene = Scene3D::new();
    let mesh_id = scene.add_mesh(mesh);
    let mut node = Node::new();
    node.mesh = Some(mesh_id);
    let node_id = scene.add_node(node);
    scene.add_root(node_id);
    scene
}

/// Synthesise a binary STL byte stream of `n_tris` triangles with the
/// same xorshift32-derived bytes the bench suite uses. The 80-byte
/// header is filled with NUL so the ASCII sniffer is guaranteed to
/// route this through the binary parser.
pub fn synth_binary_bytes(n_tris: usize) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(84 + n_tris * 50);
    bytes.extend_from_slice(&[0u8; 80]);
    bytes.extend_from_slice(&(n_tris as u32).to_le_bytes());
    let mut state: u32 = 0xa3c1_77e9;
    for _ in 0..n_tris {
        for _ in 0..12 {
            let v = (xorshift32(&mut state) as f32) / (u32::MAX as f32);
            bytes.extend_from_slice(&v.to_le_bytes());
        }
        // attribute byte count
        bytes.extend_from_slice(&0u16.to_le_bytes());
    }
    bytes
}

/// Synthesise an ASCII STL byte stream of `n_tris` triangles with
/// xorshift32-derived coordinates and Rust's default float formatter.
pub fn synth_ascii_bytes(n_tris: usize) -> Vec<u8> {
    let mut s = String::new();
    s.push_str("solid profile\n");
    let mut state: u32 = 0x91ab_d7c5;
    for _ in 0..n_tris {
        let n = [
            norm_f32(&mut state),
            norm_f32(&mut state),
            norm_f32(&mut state),
        ];
        let v0 = [
            norm_f32(&mut state),
            norm_f32(&mut state),
            norm_f32(&mut state),
        ];
        let v1 = [
            norm_f32(&mut state),
            norm_f32(&mut state),
            norm_f32(&mut state),
        ];
        let v2 = [
            norm_f32(&mut state),
            norm_f32(&mut state),
            norm_f32(&mut state),
        ];
        s.push_str(&format!(
            "  facet normal {} {} {}\n    outer loop\n      vertex {} {} {}\n      vertex {} {} {}\n      vertex {} {} {}\n    endloop\n  endfacet\n",
            n[0], n[1], n[2],
            v0[0], v0[1], v0[2],
            v1[0], v1[1], v1[2],
            v2[0], v2[1], v2[2],
        ));
    }
    s.push_str("endsolid profile\n");
    s.into_bytes()
}

/// Build a deterministic shared-index `Scene3D` for dedup-path
/// profiling. Vertex positions are drawn from a small pool so the
/// tolerance-based deduplicator has work to do.
pub fn synth_scene_shared(n_tris: usize) -> Scene3D {
    let mut state: u32 = 0xb71f_2db5;
    let pool_size = (n_tris / 6).max(8);
    let mut pool: Vec<[f32; 3]> = Vec::with_capacity(pool_size);
    for _ in 0..pool_size {
        pool.push([
            norm_f32(&mut state),
            norm_f32(&mut state),
            norm_f32(&mut state),
        ]);
    }
    let mut positions = Vec::with_capacity(n_tris * 3);
    let mut indices: Vec<u32> = Vec::with_capacity(n_tris * 3);
    for _ in 0..n_tris {
        let i0 = (xorshift32(&mut state) as usize) % pool_size;
        let i1 = (xorshift32(&mut state) as usize) % pool_size;
        let i2 = (xorshift32(&mut state) as usize) % pool_size;
        // Reuse the pool entries directly to simulate a real
        // shared-index mesh round-tripped through STL's per-vertex
        // expansion. Floating-point noise in [0, eps) is added so the
        // tolerance path has merge candidates that aren't bit-exact.
        let noise = |state: &mut u32| -> f32 {
            ((xorshift32(state) >> 16) as f32) / ((1u32 << 16) as f32) * 1e-6
        };
        positions.push([
            pool[i0][0] + noise(&mut state),
            pool[i0][1] + noise(&mut state),
            pool[i0][2] + noise(&mut state),
        ]);
        positions.push([
            pool[i1][0] + noise(&mut state),
            pool[i1][1] + noise(&mut state),
            pool[i1][2] + noise(&mut state),
        ]);
        positions.push([
            pool[i2][0] + noise(&mut state),
            pool[i2][1] + noise(&mut state),
            pool[i2][2] + noise(&mut state),
        ]);
        let base = indices.len() as u32;
        indices.push(base);
        indices.push(base + 1);
        indices.push(base + 2);
    }
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = positions;
    prim.indices = Some(Indices::U32(indices));
    let mesh = Mesh::new(None).with_primitive(prim);
    let mut scene = Scene3D::new();
    let mesh_id = scene.add_mesh(mesh);
    let mut node = Node::new();
    node.mesh = Some(mesh_id);
    let node_id = scene.add_node(node);
    scene.add_root(node_id);
    scene
}

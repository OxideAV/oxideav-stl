//! Shared fixture builders for the STL Criterion bench suite.
//!
//! All inputs are synthesised on the fly from deterministic xorshift32
//! pseudo-random numbers — no committed binary fixtures, no `docs/`
//! traffic. Each builder produces either:
//!
//! * raw STL bytes (for decode benches), or
//! * a constructed `Scene3D` (for encode and analysis benches),
//!
//! at a caller-chosen triangle count so the same generator drives
//! throughput sweeps across small / medium / large meshes.
//!
//! Each bench file pulls this module in via
//! `#[path = "common/mod.rs"] mod common;` — the standard
//! Criterion-bench helper-sharing pattern.

#![allow(dead_code)]

use oxideav_mesh3d::{Indices, Mesh, Node, Primitive, Scene3D, Topology};

/// Cheap deterministic xorshift32. Same generator the utvideo bench
/// suite uses, so seed-derived fixture geometry stays comparable
/// across crates.
fn xorshift32(state: &mut u32) -> u32 {
    *state ^= *state << 13;
    *state ^= *state >> 17;
    *state ^= *state << 5;
    *state
}

/// Map a `u32` to a "spatially-natural" `f32` in `[0, 1)` — uniformly
/// distributed but with the full 24-bit mantissa populated, so the
/// vertex stream looks like a real CAD/scanner output rather than
/// the all-integer cube fixtures the integration tests use.
fn norm_f32(state: &mut u32) -> f32 {
    let r = xorshift32(state) >> 8; // 24 bits of entropy
    (r as f32) / ((1u32 << 24) as f32)
}

/// Build an *unindexed* triangle soup of `n_tris` triangles with
/// deterministic xorshift32-derived positions and matching per-vertex
/// normals (one stored copy per corner). Returns a single-mesh
/// `Scene3D` matching the topology STL itself uses on encode.
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
        // Right-hand-rule normal of the synthesised triangle, computed
        // here once per face so the bench input matches what the
        // encoder would emit if the caller had run
        // `repair_recompute_zero_normals` first.
        let n = rhr_normal(v0, v1, v2);
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
    let mesh = Mesh::new(Some("bench".to_string())).with_primitive(prim);
    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(mesh);
    let mut node = Node::new();
    node.mesh = Some(mid);
    let nid = scene.add_node(node);
    scene.add_root(nid);
    scene
}

/// Build an *indexed* scene with `n_unique` shared corner positions
/// stitched into `n_tris` triangles. Useful for the encoder's
/// unique-vertex / share-factor benches: the same scene reports
/// `emitted_vertices == 3 * n_tris` and `unique_vertices == n_unique`,
/// so the share factor approaches `n_tris / n_unique * 3`.
pub fn synth_scene_indexed(n_unique: usize, n_tris: usize) -> Scene3D {
    assert!(n_unique >= 3, "need at least 3 unique corners");
    let mut state: u32 = 0xc0ff_ee01;
    let positions: Vec<[f32; 3]> = (0..n_unique)
        .map(|_| {
            [
                norm_f32(&mut state),
                norm_f32(&mut state),
                norm_f32(&mut state),
            ]
        })
        .collect();
    let mut indices: Vec<u32> = Vec::with_capacity(n_tris * 3);
    let mut idx_state: u32 = 0x1234_5678;
    let n_u = n_unique as u32;
    for _ in 0..n_tris {
        // Three distinct indices per triangle so we never trip the
        // degenerate-on-equal-indices rule.
        let a = xorshift32(&mut idx_state) % n_u;
        let mut b = xorshift32(&mut idx_state) % n_u;
        if b == a {
            b = (b + 1) % n_u;
        }
        let mut c = xorshift32(&mut idx_state) % n_u;
        while c == a || c == b {
            c = (c + 1) % n_u;
        }
        indices.push(a);
        indices.push(b);
        indices.push(c);
    }
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = positions;
    prim.indices = Some(Indices::U32(indices));
    let mesh = Mesh::new(Some("bench-shared".to_string())).with_primitive(prim);
    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(mesh);
    let mut node = Node::new();
    node.mesh = Some(mid);
    let nid = scene.add_node(node);
    scene.add_root(nid);
    scene
}

/// Synthesise a binary STL byte stream with `n_tris` deterministic
/// triangles — header bytes are blank (`b' '` padding) with a writer
/// signature embedded so the ASCII sniffer doesn't mistake the stream
/// for ASCII.
pub fn synth_binary_bytes(n_tris: usize) -> Vec<u8> {
    let mut state: u32 = 0x9e37_79b9;
    let mut buf = Vec::with_capacity(84 + n_tris * 50);
    let mut header = [b' '; 80];
    let sig = b"oxideav-stl-bench";
    header[..sig.len()].copy_from_slice(sig);
    buf.extend_from_slice(&header);
    buf.extend_from_slice(&(n_tris as u32).to_le_bytes());
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
        let n = rhr_normal(v0, v1, v2);
        for c in n {
            buf.extend_from_slice(&c.to_le_bytes());
        }
        for c in v0 {
            buf.extend_from_slice(&c.to_le_bytes());
        }
        for c in v1 {
            buf.extend_from_slice(&c.to_le_bytes());
        }
        for c in v2 {
            buf.extend_from_slice(&c.to_le_bytes());
        }
        buf.extend_from_slice(&0u16.to_le_bytes());
    }
    buf
}

/// Synthesise an ASCII STL byte stream with `n_tris` deterministic
/// triangles, default-formatted floats (Rust's round-trip-safe `{}`
/// formatter — the encoder's default).
pub fn synth_ascii_bytes(n_tris: usize) -> Vec<u8> {
    let mut state: u32 = 0xa55a_5aa5;
    let mut out = String::with_capacity(n_tris * 220);
    out.push_str("solid bench\n");
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
        let n = rhr_normal(v0, v1, v2);
        out.push_str(&format!("  facet normal {} {} {}\n", n[0], n[1], n[2]));
        out.push_str("    outer loop\n");
        out.push_str(&format!("      vertex {} {} {}\n", v0[0], v0[1], v0[2]));
        out.push_str(&format!("      vertex {} {} {}\n", v1[0], v1[1], v1[2]));
        out.push_str(&format!("      vertex {} {} {}\n", v2[0], v2[1], v2[2]));
        out.push_str("    endloop\n");
        out.push_str("  endfacet\n");
    }
    out.push_str("endsolid bench\n");
    out.into_bytes()
}

/// Right-hand-rule unit normal of triangle `(a, b, c)`. Returns
/// `[0, 0, 0]` when the cross product magnitude is zero (degenerate
/// fixture); the bench infra never trips this in practice because
/// the random positions almost never coincide.
fn rhr_normal(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> [f32; 3] {
    let ab = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let ac = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let cx = [
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    ];
    let mag = (cx[0] * cx[0] + cx[1] * cx[1] + cx[2] * cx[2]).sqrt();
    if mag == 0.0 {
        [0.0, 0.0, 0.0]
    } else {
        [cx[0] / mag, cx[1] / mag, cx[2] / mag]
    }
}

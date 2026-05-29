//! Long-running deterministic driver for `StlDecoder::decode` on the
//! **binary** input path. See `profile_encode_binary.rs` for the
//! profiler invocation pattern.
//!
//! Hot path: the `crate::binary::decode` per-triangle 50-byte parse
//! loop (`read_vec3 × 4` + per-face attribute pickup) plus the
//! `Scene3D` construction at the tail.

use oxideav_mesh3d::Mesh3DDecoder;
use oxideav_stl::StlDecoder;

#[path = "profile_common/mod.rs"]
mod profile_common;

const N_TRIS: usize = 10_000;
const ITERATIONS: usize = 2_000;

fn main() {
    let bytes = profile_common::synth_binary_bytes(N_TRIS);
    let mut total_triangles: usize = 0;
    for _ in 0..ITERATIONS {
        let mut dec = StlDecoder::new();
        let scene = dec.decode(&bytes).expect("binary decode");
        // Sanity-add the per-iteration triangle count so the
        // optimiser can't elide the decode.
        for m in &scene.meshes {
            for p in &m.primitives {
                total_triangles += p.positions.len() / 3;
            }
        }
    }
    println!(
        "profile_decode_binary: iterations={ITERATIONS} triangles_per_iter={N_TRIS} total_triangles={total_triangles}"
    );
}

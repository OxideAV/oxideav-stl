//! Long-running deterministic driver for `StlDecoder::decode` on the
//! **ASCII** input path. See `profile_encode_binary.rs` for the
//! profiler invocation pattern.
//!
//! Hot path: the ASCII token walk (`solid` / `facet` / `outer loop` /
//! `vertex` / `endloop` / `endfacet` / `endsolid` keyword
//! recognition) plus per-coordinate float lexing.

use oxideav_mesh3d::Mesh3DDecoder;
use oxideav_stl::StlDecoder;

#[path = "profile_common/mod.rs"]
mod profile_common;

const N_TRIS: usize = 5_000;
const ITERATIONS: usize = 200;

fn main() {
    let bytes = profile_common::synth_ascii_bytes(N_TRIS);
    let mut total_triangles: usize = 0;
    for _ in 0..ITERATIONS {
        let mut dec = StlDecoder::new();
        let scene = dec.decode(&bytes).expect("ascii decode");
        for m in &scene.meshes {
            for p in &m.primitives {
                total_triangles += p.positions.len() / 3;
            }
        }
    }
    println!(
        "profile_decode_ascii: iterations={ITERATIONS} triangles_per_iter={N_TRIS} total_triangles={total_triangles}"
    );
}

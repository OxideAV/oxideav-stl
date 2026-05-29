//! Long-running deterministic driver for `StlEncoder::encode` on the
//! **ASCII** output path. See `profile_encode_binary.rs` for the
//! profiler invocation pattern.
//!
//! Hot path: per-coordinate float-to-string formatting (`{}` default
//! formatter unless `with_float_precision` / `with_spec_scientific`
//! is set) and the keyword/whitespace emit pattern for the `solid /
//! facet / outer loop / vertex × 3 / endloop / endfacet / endsolid`
//! grammar. ASCII encode is ~50× slower than binary at matched
//! triangle count, so a smaller `N_TRIS` is used to keep the
//! profiler driver under a minute on a modern host.

use oxideav_mesh3d::Mesh3DEncoder;
use oxideav_stl::StlEncoder;

#[path = "profile_common/mod.rs"]
mod profile_common;

const N_TRIS: usize = 5_000;
const ITERATIONS: usize = 200;

fn main() {
    let scene = profile_common::synth_scene_unindexed(N_TRIS);
    let mut total_bytes: usize = 0;
    for _ in 0..ITERATIONS {
        let mut enc = StlEncoder::new_ascii();
        let out = enc.encode(&scene).expect("ascii encode");
        total_bytes = total_bytes.wrapping_add(out.len());
    }
    println!(
        "profile_encode_ascii: iterations={ITERATIONS} triangles_per_iter={N_TRIS} total_bytes={total_bytes}"
    );
}

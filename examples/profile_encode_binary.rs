//! Long-running deterministic driver for `StlEncoder::encode` on the
//! **binary** output path. Pulled in by `cargo run --release --example
//! profile_encode_binary` and intended to be wrapped in a profiler:
//!
//! ```text
//! cargo flamegraph --release --example profile_encode_binary
//! samply record cargo run --release --example profile_encode_binary
//! perf record -g cargo run --release --example profile_encode_binary
//! ```
//!
//! No randomness, no system clock dependence — every invocation
//! processes the same input bytes, so the resulting flame graph can
//! be diffed across optimisation attempts.
//!
//! Hot path: the 50-byte-per-triangle pack-and-extend loop in
//! `crate::binary::encode`, plus the per-mesh / per-primitive
//! triangle-collection walk above it. The driver loops the encoder
//! `ITERATIONS` times against a 10 000-triangle scene so the hot path
//! dominates wall time even on fast hosts.

use oxideav_mesh3d::Mesh3DEncoder;
use oxideav_stl::StlEncoder;

#[path = "profile_common/mod.rs"]
mod profile_common;

const N_TRIS: usize = 10_000;
const ITERATIONS: usize = 2_000;

fn main() {
    let scene = profile_common::synth_scene_unindexed(N_TRIS);
    let mut total_bytes: usize = 0;
    for _ in 0..ITERATIONS {
        let mut enc = StlEncoder::new_binary();
        let out = enc.encode(&scene).expect("binary encode");
        total_bytes = total_bytes.wrapping_add(out.len());
    }
    // Print a single line so the binary's work isn't optimised away,
    // and so a profiler harness can sanity-check the volume of work
    // done. Anything written here is outside the profiled hot path.
    println!(
        "profile_encode_binary: iterations={ITERATIONS} triangles_per_iter={N_TRIS} total_bytes={total_bytes}"
    );
}

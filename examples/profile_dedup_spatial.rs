//! Long-running deterministic driver for the spatial-grid
//! tolerance-based vertex deduplicator
//! (`EncodeStats::with_tolerance_spatial`). The bench suite
//! demonstrates the brute-force-vs-spatial crossover at matched
//! element counts; this profile target zeroes in on the spatial
//! path's per-cell hash insertion + 27-cell scan inner loop so it
//! can be attributed line-by-line in a profiler.

use oxideav_stl::EncodeStats;

#[path = "profile_common/mod.rs"]
mod profile_common;

const N_TRIS: usize = 50_000;
const ITERATIONS: usize = 50;
const EPS: f32 = 1.0e-5;

fn main() {
    let scene = profile_common::synth_scene_shared(N_TRIS);
    let mut acc: usize = 0;
    for _ in 0..ITERATIONS {
        let stats = EncodeStats::with_tolerance_spatial(&scene, EPS);
        acc = acc.wrapping_add(stats.unique_vertices);
    }
    println!(
        "profile_dedup_spatial: iterations={ITERATIONS} triangles_per_iter={N_TRIS} eps={EPS} sum_unique={acc}"
    );
}

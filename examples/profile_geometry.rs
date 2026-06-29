//! Long-running deterministic driver for the four non-mutating
//! scalar-geometry diagnostics — `mesh_volume`, `mesh_surface_area`,
//! `mesh_edge_length_stats`, and `mesh_centroid`. Each is a single
//! `O(N)` forward pass with `f64` accumulation; running all four in
//! sequence on a fixed fixture lets a profiler attribute cycles across
//! the cross-product / triple-product / sqrt hot loops they share.

use oxideav_stl::{mesh_centroid, mesh_edge_length_stats, mesh_surface_area, mesh_volume};

#[path = "profile_common/mod.rs"]
mod profile_common;

const N_TRIS: usize = 10_000;
const ITERATIONS: usize = 500;

fn main() {
    let scene = profile_common::synth_scene_unindexed(N_TRIS);
    let mut acc: usize = 0;
    for _ in 0..ITERATIONS {
        acc = acc.wrapping_add(mesh_volume(&scene).triangles_summed);
        acc = acc.wrapping_add(mesh_surface_area(&scene).triangles_summed);
        acc = acc.wrapping_add(mesh_edge_length_stats(&scene).edges_summed);
        acc = acc.wrapping_add(mesh_centroid(&scene).triangles_summed);
    }
    println!(
        "profile_geometry: iterations={ITERATIONS} triangles_per_iter={N_TRIS} accumulator={acc}"
    );
}

//! Criterion benchmarks for the non-mutating scalar-geometry
//! diagnostics — `mesh_volume`, `mesh_surface_area`,
//! `mesh_edge_length_stats`, and `mesh_centroid`.
//!
//! Each is a single `O(N)` forward pass over the triangle soup with
//! `f64` accumulation; this bench substantiates that cost at matched
//! triangle counts so the relative weight of the four passes (the
//! centroid does both the area and volume moment, so it is the
//! heaviest) is visible in one comparison alongside the `validate`
//! suite.
//!
//! Run with:
//!     cargo bench -p oxideav-stl --bench geometry

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};

use oxideav_stl::{mesh_centroid, mesh_edge_length_stats, mesh_surface_area, mesh_volume};

#[path = "common/mod.rs"]
mod common;

fn bench_one<F, R>(c: &mut Criterion, name: &str, f: F)
where
    F: Fn(&oxideav_mesh3d::Scene3D) -> R,
    R: 'static,
{
    let mut group = c.benchmark_group(name);
    for &n_tris in &[1_000usize, 10_000, 100_000] {
        let scene = common::synth_scene_unindexed(n_tris);
        group.throughput(Throughput::Elements(n_tris as u64));
        group.bench_with_input(BenchmarkId::from_parameter(n_tris), &scene, |b, scene| {
            b.iter(|| {
                let r = f(black_box(scene));
                black_box(r);
            });
        });
    }
    group.finish();
}

fn bench_volume(c: &mut Criterion) {
    bench_one(c, "mesh_volume", mesh_volume);
}

fn bench_surface_area(c: &mut Criterion) {
    bench_one(c, "mesh_surface_area", mesh_surface_area);
}

fn bench_edge_length_stats(c: &mut Criterion) {
    bench_one(c, "mesh_edge_length_stats", mesh_edge_length_stats);
}

fn bench_centroid(c: &mut Criterion) {
    bench_one(c, "mesh_centroid", mesh_centroid);
}

criterion_group!(
    benches,
    bench_volume,
    bench_surface_area,
    bench_edge_length_stats,
    bench_centroid
);
criterion_main!(benches);

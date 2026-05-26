//! Criterion benchmarks for `oxideav_stl::validate` — the
//! default-on rule set (facet orientation, unit normals,
//! watertight / manifold, consistent winding) and the opt-in
//! T-junction sub-check.
//!
//! The default rule set is `O(N)` plus an `O(E)` edge-map walk
//! (and an additional `O(E)` directed-edge walk for the
//! consistent-winding rule). The T-junction check is brute-force
//! `O(E · V_unique)` and is documented as diagnostic-only; this
//! bench keeps its N small enough to fit in a single sample but
//! confirms the order-of-magnitude gap.
//!
//! Run with:
//!     cargo bench -p oxideav-stl --bench validate

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};

use oxideav_stl::{validate, ValidationOptions};

#[path = "common/mod.rs"]
mod common;

fn bench_validate_default(c: &mut Criterion) {
    let mut group = c.benchmark_group("validate_default_opts");
    let opts = ValidationOptions::default();
    for &n_tris in &[1_000usize, 10_000, 100_000] {
        let scene = common::synth_scene_unindexed(n_tris);
        group.throughput(Throughput::Elements(n_tris as u64));
        group.bench_with_input(BenchmarkId::from_parameter(n_tris), &scene, |b, scene| {
            b.iter(|| {
                let report = validate(black_box(scene), black_box(&opts));
                black_box(report);
            });
        });
    }
    group.finish();
}

fn bench_validate_t_junctions(c: &mut Criterion) {
    let mut group = c.benchmark_group("validate_t_junctions_on");
    let opts = ValidationOptions {
        check_t_junctions: true,
        ..ValidationOptions::default()
    };
    // Brute-force O(E · V_unique). Cap at 1 K triangles = ~3 K edges
    // × ~3 K unique vertices = ~9 M predicate evaluations per call,
    // which already pushes a single sample into the multi-millisecond
    // range. The relative slowdown vs `validate_default_opts` at the
    // same N is the headline number.
    for &n_tris in &[100usize, 300, 1_000] {
        let scene = common::synth_scene_unindexed(n_tris);
        group.throughput(Throughput::Elements(n_tris as u64));
        group.bench_with_input(BenchmarkId::from_parameter(n_tris), &scene, |b, scene| {
            b.iter(|| {
                let report = validate(black_box(scene), black_box(&opts));
                black_box(report);
            });
        });
    }
    group.finish();
}

criterion_group!(benches, bench_validate_default, bench_validate_t_junctions);
criterion_main!(benches);

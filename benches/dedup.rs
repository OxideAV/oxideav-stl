//! Criterion benchmarks comparing the brute-force `O(N · K)` and
//! spatial-grid `O(N)` tolerance-dedup paths.
//!
//! The two paths are documented as having the same termination
//! contract but materially different asymptotic costs. This bench
//! materialises the crossover point: at small N the brute-force scan
//! wins on constant factors; somewhere between 1 K and 10 K vertices
//! the spatial path pulls ahead, and by 100 K vertices the gap is the
//! whole reason `_spatial` exists.
//!
//! Both `with_tolerance` and `with_tolerance_spatial` short-circuit to
//! the bit-exact path at `eps == 0.0`, so this bench uses a small
//! non-zero epsilon (`1e-5`) to keep both paths on their general
//! branches.
//!
//! Also measures the bit-exact `stats()` helper as a baseline — that
//! one is `HashMap`-keyed on `f32::to_bits` and is the cheapest of the
//! three.
//!
//! Run with:
//!     cargo bench -p oxideav-stl --bench dedup

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};

use oxideav_stl::{EncodeStats, StlEncoder};

#[path = "common/mod.rs"]
mod common;

const EPS: f32 = 1.0e-5;

fn bench_stats_bit_exact(c: &mut Criterion) {
    let mut group = c.benchmark_group("stats_bit_exact");
    for &n_tris in &[1_000usize, 10_000, 100_000] {
        let scene = common::synth_scene_unindexed(n_tris);
        group.throughput(Throughput::Elements((n_tris * 3) as u64));
        group.bench_with_input(BenchmarkId::from_parameter(n_tris), &scene, |b, scene| {
            b.iter(|| {
                let s = StlEncoder::stats(black_box(scene));
                black_box(s);
            });
        });
    }
    group.finish();
}

fn bench_dedup_brute(c: &mut Criterion) {
    let mut group = c.benchmark_group("dedup_brute_eps_1e-5");
    // O(N · K) — keep the upper bound at 10 K vertices = ~33 M
    // canonical-vs-emitted comparisons in the worst case, which is
    // about as long as a single bench sample should run.
    for &n_tris in &[300usize, 1_000, 3_000] {
        let scene = common::synth_scene_unindexed(n_tris);
        group.throughput(Throughput::Elements((n_tris * 3) as u64));
        group.bench_with_input(BenchmarkId::from_parameter(n_tris), &scene, |b, scene| {
            b.iter(|| {
                let s = EncodeStats::with_tolerance(black_box(scene), EPS);
                black_box(s);
            });
        });
    }
    group.finish();
}

fn bench_dedup_spatial(c: &mut Criterion) {
    let mut group = c.benchmark_group("dedup_spatial_eps_1e-5");
    // O(N) amortised — comfortable at 100 K vertices.
    for &n_tris in &[1_000usize, 10_000, 100_000] {
        let scene = common::synth_scene_unindexed(n_tris);
        group.throughput(Throughput::Elements((n_tris * 3) as u64));
        group.bench_with_input(BenchmarkId::from_parameter(n_tris), &scene, |b, scene| {
            b.iter(|| {
                let s = EncodeStats::with_tolerance_spatial(black_box(scene), EPS);
                black_box(s);
            });
        });
    }
    group.finish();
}

criterion_group!(
    benches,
    bench_stats_bit_exact,
    bench_dedup_brute,
    bench_dedup_spatial,
);
criterion_main!(benches);

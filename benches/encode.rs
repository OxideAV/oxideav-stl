//! Criterion benchmarks for `StlEncoder::encode` on both binary and
//! ASCII output.
//!
//! Scenes are synthesised once per parameter point then passed by
//! reference into the timed closure, so the bench measures the
//! encoder hot path (LE-float byte packing for binary;
//! float-to-string formatting + keyword emit for ASCII) without
//! folding scene construction into the sample.
//!
//! Run with:
//!     cargo bench -p oxideav-stl --bench encode

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};

use oxideav_mesh3d::Mesh3DEncoder;
use oxideav_stl::StlEncoder;

#[path = "common/mod.rs"]
mod common;

fn bench_encode_binary(c: &mut Criterion) {
    let mut group = c.benchmark_group("encode_binary");
    for &n_tris in &[1_000usize, 10_000, 100_000] {
        let scene = common::synth_scene_unindexed(n_tris);
        // Each triangle produces a 50-byte STL record + 84-byte header
        // overhead, so bytes-out scales linearly with triangle count.
        group.throughput(Throughput::Bytes((84 + n_tris * 50) as u64));
        group.bench_with_input(BenchmarkId::from_parameter(n_tris), &scene, |b, scene| {
            b.iter(|| {
                let mut enc = StlEncoder::new_binary();
                let bytes = enc.encode(black_box(scene)).expect("binary encode");
                black_box(bytes);
            });
        });
    }
    group.finish();
}

fn bench_encode_ascii(c: &mut Criterion) {
    let mut group = c.benchmark_group("encode_ascii");
    for &n_tris in &[1_000usize, 5_000, 10_000] {
        let scene = common::synth_scene_unindexed(n_tris);
        group.bench_with_input(BenchmarkId::from_parameter(n_tris), &scene, |b, scene| {
            b.iter(|| {
                let mut enc = StlEncoder::new_ascii();
                let bytes = enc.encode(black_box(scene)).expect("ascii encode");
                black_box(bytes);
            });
        });
    }
    group.finish();
}

criterion_group!(benches, bench_encode_binary, bench_encode_ascii);
criterion_main!(benches);

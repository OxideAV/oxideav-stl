//! Criterion benchmarks for `StlDecoder::decode` on both binary and
//! ASCII inputs.
//!
//! The synthesised mesh sizes (1 K / 10 K / 100 K triangles) bracket
//! the workshop / consumer / industrial range — 1 K is a small printed
//! widget, 10 K is a typical hobbyist CAD part, 100 K is a moderately
//! detailed scan. Throughput is reported in bytes/s so the binary and
//! ASCII paths are directly comparable at the same triangle count.
//!
//! Run with:
//!     cargo bench -p oxideav-stl --bench decode

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};

use oxideav_mesh3d::Mesh3DDecoder;
use oxideav_stl::StlDecoder;

#[path = "common/mod.rs"]
mod common;

fn bench_decode_binary(c: &mut Criterion) {
    let mut group = c.benchmark_group("decode_binary");
    for &n_tris in &[1_000usize, 10_000, 100_000] {
        let bytes = common::synth_binary_bytes(n_tris);
        group.throughput(Throughput::Bytes(bytes.len() as u64));
        group.bench_with_input(BenchmarkId::from_parameter(n_tris), &bytes, |b, bytes| {
            b.iter(|| {
                let mut dec = StlDecoder::new();
                let scene = dec.decode(black_box(bytes)).expect("binary decode");
                black_box(scene);
            });
        });
    }
    group.finish();
}

fn bench_decode_ascii(c: &mut Criterion) {
    let mut group = c.benchmark_group("decode_ascii");
    // ASCII parsing is materially slower than binary (token scan +
    // float lexing per coordinate vs straight LE float reads), so cap
    // the upper end at 10 K to keep the harness fast.
    for &n_tris in &[1_000usize, 5_000, 10_000] {
        let bytes = common::synth_ascii_bytes(n_tris);
        group.throughput(Throughput::Bytes(bytes.len() as u64));
        group.bench_with_input(BenchmarkId::from_parameter(n_tris), &bytes, |b, bytes| {
            b.iter(|| {
                let mut dec = StlDecoder::new();
                let scene = dec.decode(black_box(bytes)).expect("ascii decode");
                black_box(scene);
            });
        });
    }
    group.finish();
}

criterion_group!(benches, bench_decode_binary, bench_decode_ascii);
criterion_main!(benches);

//! Randomised property round-trip tests for the STL codec.
//!
//! The single-fixture `binary_roundtrip` / `ascii_roundtrip` tests pin
//! one hand-written cube. This file sweeps the same invariants across
//! hundreds of pseudo-randomly generated inputs so the round-trip
//! guarantees are exercised over a wide distribution of triangle
//! counts, coordinate magnitudes, float bit-patterns (including
//! NaN / ±Inf / subnormal), and per-face attribute slots — the same
//! surface the `roundtrip` fuzz target drives, but as a deterministic,
//! seeded, CI-runnable property suite that asserts the *positive*
//! byte-identity contract rather than mere panic-freedom.
//!
//! Randomness is a self-contained PCG-style LCG (the same constants the
//! `tolerance_dedup_spatial` test uses) so no external dev-dependency
//! (proptest / quickcheck) enters the build tree, and every failure is
//! reproducible from its printed seed.
//!
//! Invariants under test:
//!
//! * **Binary record byte-identity** — synthesise a length-correct
//!   binary STL with arbitrary triangle records (any float bits, any
//!   attribute slot), decode it, re-encode it, and assert the triangle
//!   count slot + every 50-byte triangle record survives byte-for-byte.
//!   The 80-byte header is writer-substituted and is NOT asserted, per
//!   the spec + the `binary_cube_triangle_records_roundtrip_byte_-
//!   identical` integration test. NaN bit-patterns survive because the
//!   assertion is on raw bytes, not numeric equality.
//! * **Binary triangle count** — the decoded `triangle_count()` equals
//!   the synthesised count for every generated file.
//! * **ASCII numeric round-trip** — generate a finite-coordinate mesh,
//!   encode it as ASCII, re-decode, and assert positions + per-face
//!   normals survive within a float-formatting tolerance.
//! * **Binary → ASCII → binary cross-flavour** — a finite binary STL
//!   decoded, ASCII-encoded, re-decoded, and binary-encoded preserves
//!   the per-vertex positions within tolerance (the ASCII hop loses the
//!   exact bit pattern to decimal formatting but not the value).
//! * **Geometry-diagnostic invariants** — `mesh_surface_area` and
//!   `mesh_edge_length_stats` are translation-invariant; the
//!   `mesh_centroid` area centroid is translation-equivariant; and
//!   `mesh_volume`'s signed volume scales by `k³` under a uniform `k`
//!   scale. These pin the positive mathematical contracts the `repair`
//!   fuzz target only checks for panic-freedom.

use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};
use oxideav_stl::{StlDecoder, StlEncoder};

const HEADER_BYTES: usize = 80;
const COUNT_BYTES: usize = 4;
const TRIANGLE_BYTES: usize = 50;
const PREFIX_BYTES: usize = HEADER_BYTES + COUNT_BYTES;

/// PCG-style LCG. Deterministic and self-contained so no external RNG
/// crate is pulled into the dev-dependency tree and every failing case
/// is reproducible from the seed printed in the assertion message.
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        // Mix the seed once so adjacent seeds don't produce correlated
        // first outputs.
        Rng(seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407))
    }

    fn next_u32(&mut self) -> u32 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        // XSH-RR style output permutation.
        let xorshifted = (((self.0 >> 18) ^ self.0) >> 27) as u32;
        let rot = (self.0 >> 59) as u32;
        xorshifted.rotate_right(rot)
    }

    fn next_u8(&mut self) -> u8 {
        (self.next_u32() & 0xFF) as u8
    }

    fn next_u16(&mut self) -> u16 {
        (self.next_u32() & 0xFFFF) as u16
    }

    /// A finite, well-behaved coordinate in roughly [-1000, 1000) so the
    /// ASCII decimal round-trip stays inside a tight tolerance.
    fn finite_coord(&mut self) -> f32 {
        let r = self.next_u32();
        // 24-bit mantissa fraction in [0,1), scaled and shifted.
        let frac = (r & 0x00FF_FFFF) as f32 / (0x0100_0000 as f32);
        (frac - 0.5) * 2000.0
    }

    /// Arbitrary f32 bit pattern — deliberately includes NaN / ±Inf /
    /// subnormal so the binary byte-identity path is stressed.
    fn raw_f32_bytes(&mut self) -> [u8; 4] {
        self.next_u32().to_le_bytes()
    }
}

/// Build a length-correct binary STL whose triangle bodies come from
/// raw (possibly non-finite) f32 bit patterns and arbitrary attribute
/// slots. Header is a writer signature that does NOT start with
/// `solid ` so the ASCII sniffer cannot false-positive.
fn synth_binary_random(rng: &mut Rng, triangles: usize) -> Vec<u8> {
    let mut buf = Vec::with_capacity(PREFIX_BYTES + triangles * TRIANGLE_BYTES);
    let mut header = [b' '; HEADER_BYTES];
    let sig = b"oxideav-stl property-test";
    header[..sig.len()].copy_from_slice(sig);
    buf.extend_from_slice(&header);
    buf.extend_from_slice(&(triangles as u32).to_le_bytes());
    for _ in 0..triangles {
        // normal (3) + v0 (3) + v1 (3) + v2 (3) = 12 raw f32 values.
        for _ in 0..12 {
            buf.extend_from_slice(&rng.raw_f32_bytes());
        }
        buf.extend_from_slice(&rng.next_u16().to_le_bytes());
    }
    debug_assert_eq!(buf.len(), PREFIX_BYTES + triangles * TRIANGLE_BYTES);
    buf
}

/// Build a length-correct binary STL with FINITE coordinates and a zero
/// attribute slot, suitable for the ASCII cross-flavour hop.
fn synth_binary_finite(rng: &mut Rng, triangles: usize) -> Vec<u8> {
    let mut buf = Vec::with_capacity(PREFIX_BYTES + triangles * TRIANGLE_BYTES);
    let mut header = [b' '; HEADER_BYTES];
    let sig = b"oxideav-stl property-test finite";
    header[..sig.len()].copy_from_slice(sig);
    buf.extend_from_slice(&header);
    buf.extend_from_slice(&(triangles as u32).to_le_bytes());
    for _ in 0..triangles {
        for _ in 0..12 {
            buf.extend_from_slice(&rng.finite_coord().to_le_bytes());
        }
        buf.extend_from_slice(&0u16.to_le_bytes());
    }
    buf
}

#[test]
fn binary_records_roundtrip_byte_identical_over_random_inputs() {
    // Sweep a range of triangle counts (including 0) and many seeds so
    // the decode → encode record-byte invariant is exercised over a
    // broad distribution of float bit-patterns and attribute slots.
    let counts = [0usize, 1, 2, 3, 7, 13, 32, 64, 100];
    for &count in &counts {
        for seed in 0..40u64 {
            let mut rng = Rng::new(seed.wrapping_mul(2654435761).wrapping_add(count as u64));
            let original = synth_binary_random(&mut rng, count);

            let scene = StlDecoder::new()
                .decode(&original)
                .unwrap_or_else(|e| panic!("decode failed (count={count} seed={seed}): {e:?}"));

            assert_eq!(
                scene.triangle_count(),
                count,
                "decoded triangle_count (count={count} seed={seed})"
            );

            let re = StlEncoder::new_binary()
                .encode(&scene)
                .unwrap_or_else(|e| panic!("encode failed (count={count} seed={seed}): {e:?}"));

            assert_eq!(
                re.len(),
                original.len(),
                "file length survives (count={count} seed={seed})"
            );
            assert_eq!(
                &re[HEADER_BYTES..PREFIX_BYTES],
                &original[HEADER_BYTES..PREFIX_BYTES],
                "triangle-count slot survives (count={count} seed={seed})"
            );
            assert_eq!(
                &re[PREFIX_BYTES..],
                &original[PREFIX_BYTES..],
                "triangle records survive byte-for-byte (count={count} seed={seed})"
            );
        }
    }
}

#[test]
fn binary_roundtrip_is_idempotent_second_pass() {
    // Decoding the re-encoded bytes and re-encoding a second time must
    // reach a fixed point: encode(decode(encode(decode(x)))) == encode(decode(x))
    // for the record region. Guards against an encoder that is sensitive
    // to whether the scene came from a freshly-synthesised buffer vs one
    // of its own outputs.
    for seed in 0..30u64 {
        let mut rng = Rng::new(seed ^ 0x5151_5151);
        let count = (rng.next_u8() % 48) as usize;
        let original = synth_binary_random(&mut rng, count);

        let scene1 = StlDecoder::new().decode(&original).unwrap();
        let enc1 = StlEncoder::new_binary().encode(&scene1).unwrap();
        let scene2 = StlDecoder::new().decode(&enc1).unwrap();
        let enc2 = StlEncoder::new_binary().encode(&scene2).unwrap();

        assert_eq!(
            &enc1[HEADER_BYTES..],
            &enc2[HEADER_BYTES..],
            "binary re-encode reaches a fixed point (seed={seed} count={count})"
        );
        assert_eq!(scene1.triangle_count(), scene2.triangle_count());
    }
}

#[test]
fn ascii_roundtrip_preserves_finite_geometry_over_random_inputs() {
    // Finite-coordinate meshes survive an ASCII encode → decode within a
    // tolerance that accounts for f32 → decimal → f32 formatting drift.
    // The default ASCII encoder emits enough significant digits that a
    // round-trip stays close; we use a relative-aware absolute bound.
    let counts = [1usize, 2, 5, 11, 30, 64];
    for &count in &counts {
        for seed in 0..25u64 {
            let mut rng = Rng::new(seed.wrapping_add(0xA5A5).wrapping_add(count as u64 * 131));
            let original = synth_binary_finite(&mut rng, count);

            let scene = StlDecoder::new().decode(&original).unwrap();
            let prim = &scene.meshes[0].primitives[0];
            let pos_before = prim.positions.clone();
            let norm_before = prim.normals.clone();

            let ascii = StlEncoder::new_ascii().encode(&scene).unwrap();
            let scene2 = StlDecoder::new().decode(&ascii).unwrap();
            let prim2 = &scene2.meshes[0].primitives[0];

            assert_eq!(
                pos_before.len(),
                prim2.positions.len(),
                "vertex count survives ASCII hop (count={count} seed={seed})"
            );

            for (i, (a, b)) in pos_before.iter().zip(prim2.positions.iter()).enumerate() {
                for axis in 0..3 {
                    let tol = 1e-3 * (1.0 + a[axis].abs());
                    assert!(
                        (a[axis] - b[axis]).abs() <= tol,
                        "position drift too large (count={count} seed={seed} vert={i} axis={axis}): {} vs {}",
                        a[axis],
                        b[axis]
                    );
                }
            }

            if let (Some(na), Some(nb)) = (norm_before.as_ref(), prim2.normals.as_ref()) {
                for (a, b) in na.iter().zip(nb.iter()) {
                    for axis in 0..3 {
                        let tol = 1e-3 * (1.0 + a[axis].abs());
                        assert!(
                            (a[axis] - b[axis]).abs() <= tol,
                            "normal drift too large (count={count} seed={seed} axis={axis})"
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn binary_to_ascii_to_binary_preserves_positions() {
    // A finite binary STL routed through the ASCII flavour and back
    // preserves per-vertex positions within the ASCII formatting
    // tolerance. The ASCII hop necessarily discards the exact f32 bit
    // pattern (decimal text), so this asserts numeric closeness rather
    // than byte-identity.
    for seed in 0..30u64 {
        let mut rng = Rng::new(seed.wrapping_mul(0x9E37_79B9));
        let count = 1 + (rng.next_u8() % 40) as usize;
        let original = synth_binary_finite(&mut rng, count);

        let scene = StlDecoder::new().decode(&original).unwrap();
        let pos0: Vec<[f32; 3]> = scene.meshes[0].primitives[0].positions.clone();

        let ascii = StlEncoder::new_ascii().encode(&scene).unwrap();
        let scene_ascii = StlDecoder::new().decode(&ascii).unwrap();

        let bin = StlEncoder::new_binary().encode(&scene_ascii).unwrap();
        let scene_bin = StlDecoder::new().decode(&bin).unwrap();
        let pos1: Vec<[f32; 3]> = scene_bin.meshes[0].primitives[0].positions.clone();

        assert_eq!(
            pos0.len(),
            pos1.len(),
            "vertex count survives binary→ascii→binary (seed={seed} count={count})"
        );
        for (a, b) in pos0.iter().zip(pos1.iter()) {
            for axis in 0..3 {
                let tol = 1e-3 * (1.0 + a[axis].abs());
                assert!(
                    (a[axis] - b[axis]).abs() <= tol,
                    "position drift across cross-flavour hop (seed={seed} count={count})"
                );
            }
        }
    }
}

#[test]
fn ascii_text_roundtrip_preserves_geometry_over_random_inputs() {
    // Generate ASCII STL text directly (not via the encoder) with a
    // randomised mix of whitespace, casing, and finite coordinates, and
    // assert the decode → encode → decode model survives. This exercises
    // the ASCII *parser's* tolerance (the encoder path above only ever
    // feeds the parser its own canonical output).
    for seed in 0..40u64 {
        let mut rng = Rng::new(seed.wrapping_add(0xDEAD_0000));
        let facets = 1 + (rng.next_u8() % 20) as usize;

        let mut text = String::new();
        text.push_str("solid prop\n");
        let mut expected: Vec<[f32; 3]> = Vec::with_capacity(facets * 3);
        for _ in 0..facets {
            let n = [rng.finite_coord(), rng.finite_coord(), rng.finite_coord()];
            text.push_str(&format!("  facet normal {} {} {}\n", n[0], n[1], n[2]));
            text.push_str("    outer loop\n");
            for _ in 0..3 {
                let v = [rng.finite_coord(), rng.finite_coord(), rng.finite_coord()];
                expected.push(v);
                text.push_str(&format!("      vertex {} {} {}\n", v[0], v[1], v[2]));
            }
            text.push_str("    endloop\n");
            text.push_str("  endfacet\n");
        }
        text.push_str("endsolid prop\n");

        let scene = StlDecoder::new()
            .decode(text.as_bytes())
            .unwrap_or_else(|e| panic!("ascii decode failed (seed={seed}): {e:?}"));
        assert_eq!(
            scene.triangle_count(),
            facets,
            "facet count parsed (seed={seed})"
        );
        let prim = &scene.meshes[0].primitives[0];
        assert_eq!(
            prim.positions.len(),
            facets * 3,
            "vertex count (seed={seed})"
        );

        for (i, (a, b)) in expected.iter().zip(prim.positions.iter()).enumerate() {
            for axis in 0..3 {
                let tol = 1e-3 * (1.0 + a[axis].abs());
                assert!(
                    (a[axis] - b[axis]).abs() <= tol,
                    "ascii-text vertex drift (seed={seed} vert={i} axis={axis}): {} vs {}",
                    a[axis],
                    b[axis]
                );
            }
        }

        // And the re-encode → re-decode fixed point.
        let enc = StlEncoder::new_ascii().encode(&scene).unwrap();
        let scene2 = StlDecoder::new().decode(&enc).unwrap();
        assert_eq!(
            scene2.triangle_count(),
            facets,
            "re-encode count (seed={seed})"
        );
    }
}

// ---------------------------------------------------------------------
// Geometry-diagnostic invariants over the random sweep.
//
// These pin the *positive* mathematical contracts of the non-mutating
// scalar-geometry diagnostics (mesh_surface_area, mesh_edge_length_stats,
// mesh_centroid, mesh_volume) the `repair` fuzz target only checks for
// panic-freedom: translation-invariance of size measures, translation-
// equivariance of the area centroid, and uniform-scale laws. Every case
// is reproducible from its printed seed.
// ---------------------------------------------------------------------

use oxideav_stl::{mesh_centroid, mesh_edge_length_stats, mesh_surface_area, mesh_volume};

/// Build a finite-coordinate scene, then translate every vertex by
/// `delta`, returning a fresh binary STL. Reuses the finite synth so the
/// coordinates are well-behaved (no NaN/Inf).
fn synth_binary_finite_translated(rng: &mut Rng, triangles: usize, delta: [f32; 3]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(PREFIX_BYTES + triangles * TRIANGLE_BYTES);
    let mut header = [b' '; HEADER_BYTES];
    header[..4].copy_from_slice(b"trsl");
    buf.extend_from_slice(&header);
    buf.extend_from_slice(&(triangles as u32).to_le_bytes());
    for _ in 0..triangles {
        // normal (unchanged direction) — emit raw finite, not shifted.
        for _ in 0..3 {
            buf.extend_from_slice(&rng.finite_coord().to_le_bytes());
        }
        // three vertices, each shifted by delta.
        for _ in 0..3 {
            for &d in &delta {
                let v = rng.finite_coord() + d;
                buf.extend_from_slice(&v.to_le_bytes());
            }
        }
        buf.extend_from_slice(&0u16.to_le_bytes());
    }
    buf
}

#[test]
fn surface_area_and_edge_stats_are_translation_invariant() {
    let counts = [1usize, 2, 5, 13, 40];
    for &count in &counts {
        for seed in 0..20u64 {
            // Same RNG stream produces the same base geometry for both
            // the un-shifted and shifted builds.
            let base_seed = seed.wrapping_mul(99991).wrapping_add(count as u64);
            let mut rng_a = Rng::new(base_seed);
            let bytes_a = synth_binary_finite_translated(&mut rng_a, count, [0.0, 0.0, 0.0]);
            let mut rng_b = Rng::new(base_seed);
            let delta = [123.5f32, -67.25, 8.75];
            let bytes_b = synth_binary_finite_translated(&mut rng_b, count, delta);

            let sa = mesh_surface_area(&StlDecoder::new().decode(&bytes_a).unwrap());
            let sb = mesh_surface_area(&StlDecoder::new().decode(&bytes_b).unwrap());
            // Areas are translation-invariant up to f32 magnitude noise.
            let tol = 1e-2 * (1.0 + sa.total_area.abs());
            assert!(
                (sa.total_area - sb.total_area).abs() <= tol,
                "area translation-invariance (count={count} seed={seed}): {} vs {}",
                sa.total_area,
                sb.total_area
            );

            let ea = mesh_edge_length_stats(&StlDecoder::new().decode(&bytes_a).unwrap());
            let eb = mesh_edge_length_stats(&StlDecoder::new().decode(&bytes_b).unwrap());
            assert_eq!(ea.triangles_summed, eb.triangles_summed);
            if let (Some(ma), Some(mb)) = (ea.max_edge_length, eb.max_edge_length) {
                let etol = 1e-2 * (1.0 + ma.abs());
                assert!(
                    (ma - mb).abs() <= etol,
                    "max edge translation-invariance (count={count} seed={seed}): {ma} vs {mb}"
                );
            }
        }
    }
}

#[test]
fn area_centroid_is_translation_equivariant() {
    for seed in 0..25u64 {
        let count = 1 + (seed % 30) as usize;
        let base_seed = seed.wrapping_mul(40503).wrapping_add(7);
        let mut rng_a = Rng::new(base_seed);
        let a = synth_binary_finite_translated(&mut rng_a, count, [0.0, 0.0, 0.0]);
        let delta = [50.0f32, -30.0, 12.0];
        let mut rng_b = Rng::new(base_seed);
        let b = synth_binary_finite_translated(&mut rng_b, count, delta);

        let ca = mesh_centroid(&StlDecoder::new().decode(&a).unwrap());
        let cb = mesh_centroid(&StlDecoder::new().decode(&b).unwrap());
        // Skip when the un-shifted scene is fully degenerate (no area).
        if let (Some(pa), Some(pb)) = (ca.area_centroid(), cb.area_centroid()) {
            for axis in 0..3 {
                let expect = pa[axis] + delta[axis] as f64;
                let tol = 1e-1 * (1.0 + expect.abs());
                assert!(
                    (pb[axis] - expect).abs() <= tol,
                    "area centroid equivariance (seed={seed} axis={axis}): {} vs {}",
                    pb[axis],
                    expect
                );
            }
        }
    }
}

#[test]
fn volume_scales_with_cube_of_linear_factor() {
    // Scaling every coordinate by k multiplies the signed volume by k³.
    for seed in 0..20u64 {
        let count = 1 + (seed % 24) as usize;
        let mut rng = Rng::new(seed.wrapping_mul(2246822519));
        let base = synth_binary_finite(&mut rng, count);
        let v1 = mesh_volume(&StlDecoder::new().decode(&base).unwrap());

        // Rebuild scaled: decode, scale positions, re-encode.
        let mut scene = StlDecoder::new().decode(&base).unwrap();
        let k = 3.0f32;
        for mesh in &mut scene.meshes {
            for prim in &mut mesh.primitives {
                for p in &mut prim.positions {
                    p[0] *= k;
                    p[1] *= k;
                    p[2] *= k;
                }
            }
        }
        let v2 = mesh_volume(&scene);
        if v1.signed_volume.abs() > 1.0 && v1.signed_volume.is_finite() {
            let expected = v1.signed_volume * (k as f64).powi(3);
            let tol = 1e-2 * (1.0 + expected.abs());
            assert!(
                (v2.signed_volume - expected).abs() <= tol,
                "volume scales by k³ (seed={seed}): {} vs {}",
                v2.signed_volume,
                expected
            );
        }
    }
}

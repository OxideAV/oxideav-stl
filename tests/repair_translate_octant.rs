//! Integration tests for `repair_translate_to_positive_octant`
//! exercised through the public decoder + repair + validate APIs.
//!
//! The 1989 spec says: *"The object represented must be located in
//! the all-positive octant. In other words, all vertex coordinates
//! must be positive-definite (nonnegative and nonzero) numbers."*
//! `validate` surfaces facets that break this under
//! `positive_octant_defects` (opt-in via
//! `ValidationOptions::check_positive_octant`); this repair is the
//! matching mutating fix-up. Verified end-to-end: a binary STL whose
//! corner sits at `(-2, -3, -4)` decodes, repairs, and re-encodes; a
//! second decode + validate-with-positive-octant-on reports zero
//! defects.

use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};
use oxideav_stl::{
    repair_translate_to_positive_octant, validate, StlDecoder, StlEncoder, ValidationOptions,
    DEFAULT_POSITIVE_OCTANT_MARGIN,
};

fn push_vec3(out: &mut Vec<u8>, v: [f32; 3]) {
    out.extend_from_slice(&v[0].to_le_bytes());
    out.extend_from_slice(&v[1].to_le_bytes());
    out.extend_from_slice(&v[2].to_le_bytes());
}

/// Single-triangle binary STL whose lowest corner is at
/// `(-2.0, -3.0, -4.0)` — i.e. firmly outside the spec's positive
/// octant on every axis.
fn build_binary_negative_corner_triangle() -> Vec<u8> {
    let mut bytes = Vec::with_capacity(84 + 50);
    bytes.extend_from_slice(&[0u8; 80]); // header
    bytes.extend_from_slice(&1u32.to_le_bytes()); // triangle count
    push_vec3(&mut bytes, [0.0, 0.0, 1.0]); // stored normal (+Z)
    push_vec3(&mut bytes, [-2.0, -3.0, -4.0]); // v0
    push_vec3(&mut bytes, [-1.0, -3.0, -4.0]); // v1
    push_vec3(&mut bytes, [-2.0, -2.0, -4.0]); // v2
    bytes.extend_from_slice(&[0u8; 2]); // attr
    bytes
}

#[test]
fn translate_octant_after_binary_decode_shifts_into_positive() {
    let bytes = build_binary_negative_corner_triangle();
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");

    // Pre-condition: validate-with-positive-octant-on flags the facet.
    let opts = ValidationOptions {
        check_positive_octant: true,
        ..Default::default()
    };
    let pre = validate(&scene, &opts);
    assert!(
        pre.positive_octant_defects >= 1,
        "expected positive-octant defect pre-repair, got {pre:?}"
    );

    // Apply the repair with the default safety margin.
    let r = repair_translate_to_positive_octant(&mut scene, DEFAULT_POSITIVE_OCTANT_MARGIN);
    assert_eq!(r.triangles_inspected, 1);
    assert_eq!(r.vertices_translated, 3);
    assert_eq!(r.skipped_non_finite_vertices, 0);
    // Per-axis delta = margin + |min|.
    let m = DEFAULT_POSITIVE_OCTANT_MARGIN;
    let expect = |abs_min: f32| m + abs_min;
    assert!((r.delta[0] - expect(2.0)).abs() < 1e-5);
    assert!((r.delta[1] - expect(3.0)).abs() < 1e-5);
    assert!((r.delta[2] - expect(4.0)).abs() < 1e-5);

    // Post-condition: validate flags zero positive-octant defects.
    let post = validate(&scene, &opts);
    assert_eq!(
        post.positive_octant_defects, 0,
        "expected zero defects post-repair, got {post:?}"
    );
}

#[test]
fn translate_octant_round_trips_through_binary_encoder() {
    // Translate, re-encode, re-decode, and confirm the bbox.min is
    // strictly positive on every axis.
    let bytes = build_binary_negative_corner_triangle();
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let _ = repair_translate_to_positive_octant(&mut scene, DEFAULT_POSITIVE_OCTANT_MARGIN);
    let out = StlEncoder::new_binary().encode(&scene).expect("encode ok");

    // Re-decode and check the first vertex's coordinates are all
    // strictly positive (the wire spec stores 12 floats per triangle:
    // 3 for normal, then 3 vertices × 3 floats).
    let scene2 = StlDecoder::new().decode(&out).expect("re-decode ok");
    let p0 = scene2.meshes[0].primitives[0].positions[0];
    assert!(p0[0] > 0.0, "x = {} should be > 0", p0[0]);
    assert!(p0[1] > 0.0, "y = {} should be > 0", p0[1]);
    assert!(p0[2] > 0.0, "z = {} should be > 0", p0[2]);

    // And the validate-rule should still report zero defects.
    let opts = ValidationOptions {
        check_positive_octant: true,
        ..Default::default()
    };
    let rep = validate(&scene2, &opts);
    assert_eq!(rep.positive_octant_defects, 0);
}

#[test]
fn translate_octant_preserves_triangle_shape() {
    // A pure translation preserves every pairwise vertex distance and
    // therefore the triangle's geometry. Compare edge lengths
    // before/after.
    let bytes = build_binary_negative_corner_triangle();
    let scene_before = StlDecoder::new().decode(&bytes).expect("decode ok");
    let mut scene_after = StlDecoder::new().decode(&bytes).expect("decode ok");
    let _ = repair_translate_to_positive_octant(&mut scene_after, DEFAULT_POSITIVE_OCTANT_MARGIN);

    let edge = |p: [f32; 3], q: [f32; 3]| -> f32 {
        ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2)).sqrt()
    };
    let pb = &scene_before.meshes[0].primitives[0].positions;
    let pa = &scene_after.meshes[0].primitives[0].positions;
    let e_before = (edge(pb[0], pb[1]), edge(pb[1], pb[2]), edge(pb[2], pb[0]));
    let e_after = (edge(pa[0], pa[1]), edge(pa[1], pa[2]), edge(pa[2], pa[0]));
    assert!((e_before.0 - e_after.0).abs() < 1e-5);
    assert!((e_before.1 - e_after.1).abs() < 1e-5);
    assert!((e_before.2 - e_after.2).abs() < 1e-5);
}

#[test]
fn translate_octant_after_decode_idempotent_on_second_pass() {
    let bytes = build_binary_negative_corner_triangle();
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let _first = repair_translate_to_positive_octant(&mut scene, DEFAULT_POSITIVE_OCTANT_MARGIN);
    let second = repair_translate_to_positive_octant(&mut scene, DEFAULT_POSITIVE_OCTANT_MARGIN);
    assert_eq!(second.delta, [0.0, 0.0, 0.0]);
    assert_eq!(second.vertices_translated, 0);
}

#[test]
fn translate_octant_on_positive_scene_after_decode_is_noop() {
    // A binary STL whose corners already sit in the +octant should be
    // left alone.
    let mut bytes = Vec::with_capacity(84 + 50);
    bytes.extend_from_slice(&[0u8; 80]);
    bytes.extend_from_slice(&1u32.to_le_bytes());
    push_vec3(&mut bytes, [0.0, 0.0, 1.0]);
    push_vec3(&mut bytes, [10.0, 20.0, 30.0]);
    push_vec3(&mut bytes, [11.0, 20.0, 30.0]);
    push_vec3(&mut bytes, [10.0, 21.0, 30.0]);
    bytes.extend_from_slice(&[0u8; 2]);

    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = repair_translate_to_positive_octant(&mut scene, DEFAULT_POSITIVE_OCTANT_MARGIN);
    assert_eq!(r.delta, [0.0, 0.0, 0.0]);
    assert_eq!(r.vertices_translated, 0);

    // And the corner positions are unchanged.
    let p0 = scene.meshes[0].primitives[0].positions[0];
    assert_eq!(p0, [10.0, 20.0, 30.0]);
}

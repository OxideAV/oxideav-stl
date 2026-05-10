//! End-to-end validation against real STL bytes.
//!
//! Round-trips through the decoder so the validation contract is
//! exercised on the same `Scene3D` shape that production callers will
//! see — bit-exact positions, per-vertex normals expanded from per-face
//! normals, and the encoder's effective vertex iteration order.

use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};
use oxideav_stl::{
    bbox, validate, Bbox, StlDecoder, StlEncoder, ValidationOptions, ValidationReport,
    MAX_REPORTED_DEFECTS,
};

/// Build a binary STL byte stream for a closed unit cube — 12
/// triangles, 36 emitted vertices, every edge shared by exactly two
/// triangles when bit-exact compared.
fn unit_cube_binary_bytes() -> Vec<u8> {
    let mut buf = Vec::new();
    let mut header = [0u8; 80];
    header[..7].copy_from_slice(b"oxideav");
    buf.extend_from_slice(&header);
    buf.extend_from_slice(&12u32.to_le_bytes());

    let push_tri = |buf: &mut Vec<u8>, n: [f32; 3], v0: [f32; 3], v1: [f32; 3], v2: [f32; 3]| {
        for c in n {
            buf.extend_from_slice(&c.to_le_bytes());
        }
        for c in v0 {
            buf.extend_from_slice(&c.to_le_bytes());
        }
        for c in v1 {
            buf.extend_from_slice(&c.to_le_bytes());
        }
        for c in v2 {
            buf.extend_from_slice(&c.to_le_bytes());
        }
        buf.extend_from_slice(&0u16.to_le_bytes());
    };

    // -Z bottom (normal = (0, 0, -1))
    push_tri(
        &mut buf,
        [0.0, 0.0, -1.0],
        [0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
    );
    push_tri(
        &mut buf,
        [0.0, 0.0, -1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 1.0, 0.0],
    );
    // +Z top (normal = (0, 0, 1))
    push_tri(
        &mut buf,
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [0.0, 1.0, 1.0],
    );
    push_tri(
        &mut buf,
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [1.0, 1.0, 1.0],
        [0.0, 1.0, 1.0],
    );
    // -Y front (normal = (0, -1, 0))
    push_tri(
        &mut buf,
        [0.0, -1.0, 0.0],
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    );
    push_tri(
        &mut buf,
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
    );
    // +Y back (normal = (0, 1, 0))
    push_tri(
        &mut buf,
        [0.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 1.0, 1.0],
        [1.0, 1.0, 0.0],
    );
    push_tri(
        &mut buf,
        [0.0, 1.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 1.0],
        [1.0, 1.0, 1.0],
    );
    // +X right (normal = (1, 0, 0))
    push_tri(
        &mut buf,
        [1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [1.0, 0.0, 1.0],
    );
    push_tri(
        &mut buf,
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [1.0, 1.0, 1.0],
        [1.0, 0.0, 1.0],
    );
    // -X left (normal = (-1, 0, 0))
    push_tri(
        &mut buf,
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
    );
    push_tri(
        &mut buf,
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 1.0],
    );
    buf
}

#[test]
fn round_trip_unit_cube_is_clean_and_watertight() {
    let bytes = unit_cube_binary_bytes();
    let scene = StlDecoder::new().decode(&bytes).unwrap();
    let r = validate(&scene, &ValidationOptions::default());
    assert_eq!(r.triangles_total, 12);
    assert_eq!(r.facet_orientation_defects, 0, "report: {r:?}");
    assert_eq!(r.non_unit_normal_defects, 0, "report: {r:?}");
    assert_eq!(r.boundary_edges, 0, "report: {r:?}");
    assert_eq!(r.non_manifold_edges, 0, "report: {r:?}");
    assert!(r.watertight);
    assert!(r.is_clean());
}

#[test]
fn round_trip_unit_cube_bbox_is_unit_box() {
    let bytes = unit_cube_binary_bytes();
    let scene = StlDecoder::new().decode(&bytes).unwrap();
    let bb = bbox(&scene).unwrap();
    assert_eq!(
        bb,
        Bbox {
            min: [0.0, 0.0, 0.0],
            max: [1.0, 1.0, 1.0],
        }
    );
}

#[test]
fn ascii_round_trip_pair_is_not_watertight() {
    // Two triangles sharing one edge — open mesh (4 boundary edges).
    let s = "solid t\n\
        facet normal 0 0 1\n\
        outer loop\n\
        vertex 0 0 0\n\
        vertex 1 0 0\n\
        vertex 0 1 0\n\
        endloop\n\
        endfacet\n\
        facet normal 0 0 1\n\
        outer loop\n\
        vertex 1 0 0\n\
        vertex 1 1 0\n\
        vertex 0 1 0\n\
        endloop\n\
        endfacet\n\
        endsolid t\n";
    let scene = StlDecoder::new().decode(s.as_bytes()).unwrap();
    let r = validate(&scene, &ValidationOptions::default());
    assert_eq!(r.triangles_total, 2);
    assert_eq!(r.boundary_edges, 4);
    assert_eq!(r.non_manifold_edges, 0);
    assert!(!r.watertight);
}

#[test]
fn validate_off_path_skips_each_rule_independently() {
    let bytes = unit_cube_binary_bytes();
    let scene = StlDecoder::new().decode(&bytes).unwrap();

    // All checks off → completely empty report (counts zero, examples empty).
    let opts = ValidationOptions {
        check_facet_orientation: false,
        check_unit_normal: false,
        check_positive_octant: false,
        check_watertight: false,
        ..ValidationOptions::default()
    };
    let r = validate(&scene, &opts);
    assert_eq!(r.facet_orientation_defects, 0);
    assert_eq!(r.non_unit_normal_defects, 0);
    assert_eq!(r.positive_octant_defects, 0);
    assert_eq!(r.boundary_edges, 0);
    assert_eq!(r.non_manifold_edges, 0);
    assert!(!r.watertight);
    // triangles_total still walks the geometry — it's the loop counter,
    // not a per-rule signal.
    assert_eq!(r.triangles_total, 12);
}

#[test]
fn cube_passes_positive_octant_when_strictly_inside() {
    // Shift the cube to (1..2, 1..2, 1..2) — every vertex now strictly
    // > 0 on every axis, so the positive-octant rule passes.
    let bytes = unit_cube_binary_bytes();
    let mut scene = StlDecoder::new().decode(&bytes).unwrap();
    for prim in scene.meshes[0].primitives.iter_mut() {
        for p in prim.positions.iter_mut() {
            for c in p.iter_mut() {
                *c += 1.0;
            }
        }
    }
    let opts = ValidationOptions {
        check_positive_octant: true,
        ..ValidationOptions::default()
    };
    let r = validate(&scene, &opts);
    assert_eq!(r.positive_octant_defects, 0, "report: {r:?}");
    let bb = bbox(&scene).unwrap();
    assert_eq!(bb.min, [1.0, 1.0, 1.0]);
    assert_eq!(bb.max, [2.0, 2.0, 2.0]);
}

#[test]
fn cube_at_origin_fails_positive_octant_under_spec() {
    // The base cube has vertices at (0, _, _) etc. — every vertex
    // fails the "nonnegative AND nonzero" rule.
    let bytes = unit_cube_binary_bytes();
    let scene = StlDecoder::new().decode(&bytes).unwrap();
    let opts = ValidationOptions {
        check_positive_octant: true,
        ..ValidationOptions::default()
    };
    let r = validate(&scene, &opts);
    // Every triangle has at least one vertex on an axis plane.
    assert_eq!(r.positive_octant_defects, 12);
    // examples capped at MAX_REPORTED_DEFECTS.
    assert_eq!(
        r.positive_octant_examples.len(),
        12.min(MAX_REPORTED_DEFECTS)
    );
}

#[test]
fn validation_report_after_encode_decode_roundtrip_matches_directly() {
    // The validation contract is "what the decoded scene looks like";
    // re-encoding and re-decoding should produce a scene with an
    // identical report (modulo extras).
    let bytes = unit_cube_binary_bytes();
    let scene1 = StlDecoder::new().decode(&bytes).unwrap();
    let bytes2 = StlEncoder::new_binary().encode(&scene1).unwrap();
    let scene2 = StlDecoder::new().decode(&bytes2).unwrap();
    let opts = ValidationOptions::default();
    let r1 = validate(&scene1, &opts);
    let r2 = validate(&scene2, &opts);
    // Triangle count + watertightness + edge counts must match.
    assert_eq!(r1.triangles_total, r2.triangles_total);
    assert_eq!(r1.boundary_edges, r2.boundary_edges);
    assert_eq!(r1.non_manifold_edges, r2.non_manifold_edges);
    assert_eq!(r1.watertight, r2.watertight);
}

#[test]
fn validation_report_default_is_zero_and_unclean_for_watertight() {
    // The Default impl on `ValidationReport` produces a vacuously-clean
    // empty report. Because triangles_total is 0, watertight is false
    // but is_clean() is true — see the validate_empty_scene_is_vacuous
    // unit test for the corresponding integration path.
    let r = ValidationReport::default();
    assert!(r.is_clean());
    assert!(!r.watertight);
}

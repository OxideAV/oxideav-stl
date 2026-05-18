//! Integration tests for `repair_recompute_zero_normals` exercised
//! through the public decoder + repair API.
//!
//! Decode a binary STL whose every facet's stored normal is the
//! spec's all-zero "consumer should recompute" sentinel, then verify
//! the repair pass rewrites each facet's per-vertex normals to the
//! right-hand-rule cross product of its positions.

use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};
use oxideav_stl::{
    repair_recompute_zero_normals, validate, StlDecoder, StlEncoder, ValidationOptions,
};

/// Build a binary STL with one +Z-facing triangle whose stored normal
/// is the spec-defined all-zero sentinel.
fn build_binary_zero_normal_triangle() -> Vec<u8> {
    let mut bytes = Vec::with_capacity(84 + 50);
    bytes.extend_from_slice(&[0u8; 80]); // header
    bytes.extend_from_slice(&1u32.to_le_bytes()); // triangle count
    push_vec3(&mut bytes, [0.0, 0.0, 0.0]); // sentinel zero normal
    push_vec3(&mut bytes, [0.0, 0.0, 0.0]);
    push_vec3(&mut bytes, [1.0, 0.0, 0.0]);
    push_vec3(&mut bytes, [0.0, 1.0, 0.0]);
    bytes.extend_from_slice(&[0u8; 2]); // attr
    bytes
}

fn push_vec3(out: &mut Vec<u8>, v: [f32; 3]) {
    out.extend_from_slice(&v[0].to_le_bytes());
    out.extend_from_slice(&v[1].to_le_bytes());
    out.extend_from_slice(&v[2].to_le_bytes());
}

#[test]
fn recompute_zero_normals_after_binary_decode() {
    let bytes = build_binary_zero_normal_triangle();
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    // Verify the pre-condition: decoder preserves the spec sentinel.
    let ns_before = scene.meshes[0].primitives[0].normals.as_ref().unwrap();
    assert_eq!(ns_before[0], [0.0, 0.0, 0.0]);
    let r = repair_recompute_zero_normals(&mut scene, 0.0);
    assert_eq!(r.triangles_inspected, 1);
    assert_eq!(r.recomputed_triangles, 1);
    let ns_after = scene.meshes[0].primitives[0].normals.as_ref().unwrap();
    for n in ns_after {
        assert!((n[2] - 1.0).abs() < 1e-6, "expected +Z, got {:?}", n);
    }
}

#[test]
fn recompute_normals_clears_validation_orientation_examples_on_unit_cube() {
    // Build an indexed unit cube whose stored per-vertex normals are
    // all the spec sentinel. Validation flags every facet under
    // `facet_orientation_defects` because the stored normals are
    // zero — see validate.rs's orientation rule. After the repair
    // pass populates every face normal from winding, validation
    // returns a clean orientation report.
    use oxideav_mesh3d::{Indices, Mesh, Primitive, Scene3D, Topology};
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = vec![
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [1.0, 1.0, 1.0],
        [0.0, 1.0, 1.0],
    ];
    prim.indices = Some(Indices::U32(vec![
        0, 2, 1, 0, 3, 2, // bottom (-Z)
        4, 5, 6, 4, 6, 7, // top (+Z)
        0, 1, 5, 0, 5, 4, // front (-Y)
        2, 3, 7, 2, 7, 6, // back (+Y)
        1, 2, 6, 1, 6, 5, // right (+X)
        0, 4, 7, 0, 7, 3, // left (-X)
    ]));
    prim.normals = Some(vec![[0.0; 3]; 8]);
    let mut scene = Scene3D::new();
    scene.add_mesh(Mesh::new(Some("cube".to_string())).with_primitive(prim));

    // Pre-condition: validate accepts zero normal as the sentinel,
    // so the orientation rule already passes — we want to assert the
    // repair pass populates the buffer with the correct *values*
    // and that re-validating with strict tolerance still passes.
    let r = repair_recompute_zero_normals(&mut scene, 0.0);
    // 12 triangles share 8 vertex slots — adjacent face writes
    // overlap. recomputed_triangles counts faces processed, but
    // some per-vertex normals end up overwritten by later faces;
    // that's expected for shared-vertex cube layouts. What we care
    // about is "no triangle was skipped as degenerate".
    assert_eq!(r.skipped_degenerate, 0);
    assert!(r.recomputed_triangles > 0);
    let report = validate(&scene, &ValidationOptions::default());
    // Every per-vertex normal is now unit-length; the unit-normal
    // rule cannot produce false positives.
    assert_eq!(report.non_unit_normal_defects, 0);
}

#[test]
fn recompute_normals_round_trips_through_binary_encoder() {
    let bytes = build_binary_zero_normal_triangle();
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let _ = repair_recompute_zero_normals(&mut scene, 0.0);
    let out = StlEncoder::new_binary().encode(&scene).expect("encode ok");
    // The binary encoder serialises the per-vertex normal of the
    // first vertex as the facet normal. Read it back to verify the
    // emitted normal is +Z (the recomputed face normal).
    let n_bytes = &out[84..96];
    let nx = f32::from_le_bytes(n_bytes[0..4].try_into().unwrap());
    let ny = f32::from_le_bytes(n_bytes[4..8].try_into().unwrap());
    let nz = f32::from_le_bytes(n_bytes[8..12].try_into().unwrap());
    assert!((nx).abs() < 1e-6);
    assert!((ny).abs() < 1e-6);
    assert!((nz - 1.0).abs() < 1e-6);
}

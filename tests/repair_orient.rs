//! Integration tests for `repair_orient_normals_from_winding`
//! exercised through the public decoder + repair API.
//!
//! The 1989 spec says facet orientation is "specified redundantly in
//! two ways which must be consistent": (1) the stored normal points
//! outward; (2) the winding is CCW viewed from outside (right-hand
//! rule). When a producer emits an inverted stored normal — `(0,0,-1)`
//! on a `(0,0,+1)`-winding triangle — this repair pass rewrites the
//! stored normal to match the winding (winding is authoritative).

use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};
use oxideav_stl::{
    repair_orient_normals_from_winding, validate, StlDecoder, StlEncoder, ValidationOptions,
};

/// Build a binary STL with one +Z-winding triangle whose stored normal
/// is the *inverted* `(0,0,-1)`. The validate pass should flag it
/// under `facet_orientation_defects`; the repair pass should fix it.
fn build_binary_inverted_normal_triangle() -> Vec<u8> {
    let mut bytes = Vec::with_capacity(84 + 50);
    bytes.extend_from_slice(&[0u8; 80]); // header
    bytes.extend_from_slice(&1u32.to_le_bytes()); // triangle count
    push_vec3(&mut bytes, [0.0, 0.0, -1.0]); // stored normal: inverted
    push_vec3(&mut bytes, [0.0, 0.0, 0.0]); // v0
    push_vec3(&mut bytes, [1.0, 0.0, 0.0]); // v1
    push_vec3(&mut bytes, [0.0, 1.0, 0.0]); // v2 → RHR cross = (0,0,1)
    bytes.extend_from_slice(&[0u8; 2]); // attr
    bytes
}

fn push_vec3(out: &mut Vec<u8>, v: [f32; 3]) {
    out.extend_from_slice(&v[0].to_le_bytes());
    out.extend_from_slice(&v[1].to_le_bytes());
    out.extend_from_slice(&v[2].to_le_bytes());
}

#[test]
fn orient_after_binary_decode_flips_inverted_normal() {
    let bytes = build_binary_inverted_normal_triangle();
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    // Pre-condition: decoder preserves the inverted stored normal.
    let n_before = scene.meshes[0].primitives[0].normals.as_ref().unwrap()[0];
    assert!((n_before[2] + 1.0).abs() < 1e-6);
    // Pre-condition: validate flags it under orientation defects.
    let pre = validate(&scene, &ValidationOptions::default());
    assert!(
        pre.facet_orientation_defects >= 1,
        "expected orientation defect, got {pre:?}"
    );
    // Apply the repair.
    let r = repair_orient_normals_from_winding(&mut scene, 0.0);
    assert_eq!(r.triangles_inspected, 1);
    assert_eq!(r.flipped_normals, 1);
    // Post-condition: stored normal now points +Z.
    let n_after = scene.meshes[0].primitives[0].normals.as_ref().unwrap()[0];
    assert!((n_after[2] - 1.0).abs() < 1e-6);
    // Post-condition: validate no longer flags an orientation defect.
    let post = validate(&scene, &ValidationOptions::default());
    assert_eq!(post.facet_orientation_defects, 0);
}

#[test]
fn orient_round_trips_through_binary_encoder() {
    let bytes = build_binary_inverted_normal_triangle();
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let _ = repair_orient_normals_from_winding(&mut scene, 0.0);
    let out = StlEncoder::new_binary().encode(&scene).expect("encode ok");
    // Read the emitted facet normal at offset 84..96.
    let nx = f32::from_le_bytes(out[84..88].try_into().unwrap());
    let ny = f32::from_le_bytes(out[88..92].try_into().unwrap());
    let nz = f32::from_le_bytes(out[92..96].try_into().unwrap());
    assert!(nx.abs() < 1e-6);
    assert!(ny.abs() < 1e-6);
    assert!((nz - 1.0).abs() < 1e-6);
}

#[test]
fn orient_no_op_after_recompute_on_zero_normal_input() {
    // A zero-normal triangle decoded from binary: recompute populates
    // the normal from winding; orient is then a no-op because the
    // normal is already aligned (dot > 0).
    use oxideav_stl::repair_recompute_zero_normals;
    let mut bytes = Vec::with_capacity(84 + 50);
    bytes.extend_from_slice(&[0u8; 80]);
    bytes.extend_from_slice(&1u32.to_le_bytes());
    push_vec3(&mut bytes, [0.0, 0.0, 0.0]); // zero sentinel
    push_vec3(&mut bytes, [0.0, 0.0, 0.0]);
    push_vec3(&mut bytes, [1.0, 0.0, 0.0]);
    push_vec3(&mut bytes, [0.0, 1.0, 0.0]);
    bytes.extend_from_slice(&[0u8; 2]);
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let rec = repair_recompute_zero_normals(&mut scene, 0.0);
    assert_eq!(rec.recomputed_triangles, 1);
    let orient = repair_orient_normals_from_winding(&mut scene, 0.0);
    assert_eq!(orient.flipped_normals, 0);
}

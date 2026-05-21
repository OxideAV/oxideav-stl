//! Integration tests for `repair_normalize_unit_normals` exercised
//! through the public decoder + repair API.
//!
//! The 1989 spec says each facet's stored normal is a *unit* vector.
//! When a producer emits a non-unit-length stored normal (length-3,
//! length-0.25, etc.), `validate` flags it under
//! `non_unit_normal_defects`; this repair rescales every such normal
//! to unit length, preserving direction.

use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};
use oxideav_stl::{
    repair_normalize_unit_normals, validate, StlDecoder, StlEncoder, ValidationOptions,
};

/// Build a binary STL whose stored normal is `(0,0,3)` — same
/// direction as the RHR cross product `(0,0,1)`, but with length 3.
fn build_binary_overlong_normal_triangle() -> Vec<u8> {
    let mut bytes = Vec::with_capacity(84 + 50);
    bytes.extend_from_slice(&[0u8; 80]); // header
    bytes.extend_from_slice(&1u32.to_le_bytes()); // triangle count
    push_vec3(&mut bytes, [0.0, 0.0, 3.0]); // overlong stored normal
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
fn normalize_after_binary_decode_rescales_overlong_normal() {
    let bytes = build_binary_overlong_normal_triangle();
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    // Pre-condition: decoder preserves the overlong stored normal.
    let n_before = scene.meshes[0].primitives[0].normals.as_ref().unwrap()[0];
    let len_before = (n_before[0].powi(2) + n_before[1].powi(2) + n_before[2].powi(2)).sqrt();
    assert!((len_before - 3.0).abs() < 1e-6);
    // Pre-condition: validate flags it under unit-length defects.
    let pre = validate(&scene, &ValidationOptions::default());
    assert!(
        pre.non_unit_normal_defects >= 1,
        "expected non-unit defect, got {pre:?}"
    );
    // Apply the repair.
    let r = repair_normalize_unit_normals(&mut scene, 1e-3);
    assert_eq!(r.triangles_inspected, 1);
    assert_eq!(r.rescaled_normals, 1);
    let n_after = scene.meshes[0].primitives[0].normals.as_ref().unwrap()[0];
    let len_after = (n_after[0].powi(2) + n_after[1].powi(2) + n_after[2].powi(2)).sqrt();
    assert!((len_after - 1.0).abs() < 1e-6);
    // Post-condition: validate no longer flags a unit-length defect.
    let post = validate(&scene, &ValidationOptions::default());
    assert_eq!(post.non_unit_normal_defects, 0);
}

#[test]
fn normalize_round_trips_through_binary_encoder() {
    let bytes = build_binary_overlong_normal_triangle();
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let _ = repair_normalize_unit_normals(&mut scene, 1e-3);
    let out = StlEncoder::new_binary().encode(&scene).expect("encode ok");
    let nx = f32::from_le_bytes(out[84..88].try_into().unwrap());
    let ny = f32::from_le_bytes(out[88..92].try_into().unwrap());
    let nz = f32::from_le_bytes(out[92..96].try_into().unwrap());
    let len = (nx.powi(2) + ny.powi(2) + nz.powi(2)).sqrt();
    assert!((len - 1.0).abs() < 1e-6);
}

#[test]
fn normalize_preserves_direction_when_rescaling() {
    // Stored normal of length 0.5 along the +X+Y+Z diagonal.
    use oxideav_mesh3d::{Mesh, Primitive, Scene3D, Topology};
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    let inv_sqrt_3 = 1.0_f32 / 3.0_f32.sqrt();
    let half_len = [inv_sqrt_3 * 0.5, inv_sqrt_3 * 0.5, inv_sqrt_3 * 0.5];
    prim.normals = Some(vec![half_len; 3]);
    let mut scene = Scene3D::new();
    scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
    let r = repair_normalize_unit_normals(&mut scene, 1e-3);
    assert_eq!(r.rescaled_normals, 1);
    let n = scene.meshes[0].primitives[0].normals.as_ref().unwrap()[0];
    // Direction preserved (each component should equal `inv_sqrt_3`).
    for component in n {
        assert!(
            (component - inv_sqrt_3).abs() < 1e-6,
            "expected {inv_sqrt_3}, got {component}"
        );
    }
    let len = (n[0].powi(2) + n[1].powi(2) + n[2].powi(2)).sqrt();
    assert!((len - 1.0).abs() < 1e-6);
}

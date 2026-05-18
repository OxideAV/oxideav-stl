//! Integration tests for `repair_drop_degenerate_triangles` exercised
//! through the public decoder + repair API.
//!
//! Decode a binary STL hand-crafted with one healthy triangle and one
//! all-coincident-corners degenerate, then verify the repair pass
//! drops exactly the degenerate face and leaves the healthy one
//! intact.

use oxideav_mesh3d::{Indices, Mesh3DDecoder, Mesh3DEncoder, Topology};
use oxideav_stl::{repair_drop_degenerate_triangles, StlDecoder};

/// Build a binary STL with two triangles laid out byte-for-byte; the
/// second has three coincident corners and counts as degenerate.
fn build_binary_two_tri_with_degenerate() -> Vec<u8> {
    // 80-byte header + u32 triangle count + 2 * 50-byte records.
    let mut bytes = Vec::with_capacity(84 + 2 * 50);
    bytes.extend_from_slice(&[0u8; 80]);
    bytes.extend_from_slice(&2u32.to_le_bytes());
    // Triangle 1 — healthy +Z face: normal, three corners, attr.
    push_vec3(&mut bytes, [0.0, 0.0, 1.0]);
    push_vec3(&mut bytes, [0.0, 0.0, 0.0]);
    push_vec3(&mut bytes, [1.0, 0.0, 0.0]);
    push_vec3(&mut bytes, [0.0, 1.0, 0.0]);
    bytes.extend_from_slice(&[0u8; 2]);
    // Triangle 2 — three coincident corners (degenerate).
    push_vec3(&mut bytes, [0.0, 0.0, 0.0]);
    push_vec3(&mut bytes, [5.0, 5.0, 5.0]);
    push_vec3(&mut bytes, [5.0, 5.0, 5.0]);
    push_vec3(&mut bytes, [5.0, 5.0, 5.0]);
    bytes.extend_from_slice(&[0u8; 2]);
    bytes
}

fn push_vec3(out: &mut Vec<u8>, v: [f32; 3]) {
    out.extend_from_slice(&v[0].to_le_bytes());
    out.extend_from_slice(&v[1].to_le_bytes());
    out.extend_from_slice(&v[2].to_le_bytes());
}

#[test]
fn drop_degenerate_after_binary_decode_culls_one_face() {
    let bytes = build_binary_two_tri_with_degenerate();
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    // Decoder produces a flat triangle soup: 2 triangles → 6 vertex
    // slots, no index buffer.
    let p = &scene.meshes[0].primitives[0];
    assert_eq!(p.topology, Topology::Triangles);
    assert!(p.indices.is_none(), "decoder emits unindexed soup");
    assert_eq!(p.positions.len(), 6);
    let r = repair_drop_degenerate_triangles(&mut scene);
    assert_eq!(r.triangles_inspected, 2);
    assert_eq!(r.dropped_triangles, 1);
    let p = &scene.meshes[0].primitives[0];
    // 3 vertex slots remain (one healthy triangle).
    assert_eq!(p.positions.len(), 3);
    assert_eq!(p.positions[0], [0.0, 0.0, 0.0]);
    assert_eq!(p.positions[1], [1.0, 0.0, 0.0]);
    assert_eq!(p.positions[2], [0.0, 1.0, 0.0]);
}

#[test]
fn drop_degenerate_then_re_encode_round_trips_to_one_triangle() {
    let bytes = build_binary_two_tri_with_degenerate();
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let _ = repair_drop_degenerate_triangles(&mut scene);
    let out = oxideav_stl::StlEncoder::new_binary()
        .encode(&scene)
        .expect("encode ok");
    // 84-byte header/count + 1 × 50-byte triangle record.
    assert_eq!(out.len(), 84 + 50);
    let tri_count = u32::from_le_bytes(out[80..84].try_into().unwrap());
    assert_eq!(tri_count, 1);
}

#[test]
fn drop_degenerate_indexed_path_preserves_index_discriminant() {
    use oxideav_mesh3d::{Mesh, Primitive, Scene3D};
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = vec![[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    prim.indices = Some(Indices::U16(vec![0, 1, 2, 0, 0, 1]));
    let mut scene = Scene3D::new();
    scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
    let r = repair_drop_degenerate_triangles(&mut scene);
    assert_eq!(r.dropped_triangles, 1);
    match &scene.meshes[0].primitives[0].indices {
        Some(Indices::U16(idx)) => assert_eq!(idx, &vec![0u16, 1, 2]),
        _ => panic!("U16 discriminant should be preserved"),
    }
}

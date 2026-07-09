//! Integration tests for `repair_drop_duplicate_facets` — the pass that
//! removes surplus copies of a repeated corner-triple (a doubled surface
//! patch that neither the degenerate rule nor the non-manifold-edge rule
//! catches).

use oxideav_mesh3d::{Indices, Mesh, Mesh3DDecoder, Mesh3DEncoder, Primitive, Scene3D, Topology};
use oxideav_stl::{repair_drop_duplicate_facets, StlDecoder, StlEncoder};

fn push_vec3(out: &mut Vec<u8>, v: [f32; 3]) {
    out.extend_from_slice(&v[0].to_le_bytes());
    out.extend_from_slice(&v[1].to_le_bytes());
    out.extend_from_slice(&v[2].to_le_bytes());
}

/// Binary STL: three triangles where the second is a byte-for-byte
/// duplicate of the first and the third is a distinct healthy face.
fn build_binary_with_exact_duplicate() -> Vec<u8> {
    let mut bytes = Vec::with_capacity(84 + 3 * 50);
    bytes.extend_from_slice(&[0u8; 80]);
    bytes.extend_from_slice(&3u32.to_le_bytes());
    // Triangle 1 — healthy +Z face.
    push_vec3(&mut bytes, [0.0, 0.0, 1.0]);
    push_vec3(&mut bytes, [0.0, 0.0, 0.0]);
    push_vec3(&mut bytes, [1.0, 0.0, 0.0]);
    push_vec3(&mut bytes, [0.0, 1.0, 0.0]);
    bytes.extend_from_slice(&[0u8; 2]);
    // Triangle 2 — exact duplicate of triangle 1.
    push_vec3(&mut bytes, [0.0, 0.0, 1.0]);
    push_vec3(&mut bytes, [0.0, 0.0, 0.0]);
    push_vec3(&mut bytes, [1.0, 0.0, 0.0]);
    push_vec3(&mut bytes, [0.0, 1.0, 0.0]);
    bytes.extend_from_slice(&[0u8; 2]);
    // Triangle 3 — distinct healthy face.
    push_vec3(&mut bytes, [0.0, 0.0, 1.0]);
    push_vec3(&mut bytes, [1.0, 0.0, 0.0]);
    push_vec3(&mut bytes, [1.0, 1.0, 0.0]);
    push_vec3(&mut bytes, [0.0, 1.0, 0.0]);
    bytes.extend_from_slice(&[0u8; 2]);
    bytes
}

#[test]
fn drops_exact_duplicate_keeps_first_and_distinct() {
    let bytes = build_binary_with_exact_duplicate();
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let p = &scene.meshes[0].primitives[0];
    assert_eq!(p.positions.len(), 9, "3 triangles → 9 vertex slots");

    let r = repair_drop_duplicate_facets(&mut scene);
    assert_eq!(r.triangles_inspected, 3);
    assert_eq!(r.dropped_triangles, 1, "one surplus copy removed");

    let p = &scene.meshes[0].primitives[0];
    assert_eq!(p.positions.len(), 6, "two distinct triangles survive");
    // First occurrence survives unchanged.
    assert_eq!(p.positions[0], [0.0, 0.0, 0.0]);
    assert_eq!(p.positions[1], [1.0, 0.0, 0.0]);
    assert_eq!(p.positions[2], [0.0, 1.0, 0.0]);
    // Distinct third triangle survives.
    assert_eq!(p.positions[3], [1.0, 0.0, 0.0]);
    assert_eq!(p.positions[4], [1.0, 1.0, 0.0]);
    assert_eq!(p.positions[5], [0.0, 1.0, 0.0]);
}

#[test]
fn idempotent_on_clean_scene() {
    let bytes = build_binary_with_exact_duplicate();
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let _ = repair_drop_duplicate_facets(&mut scene);
    // Second pass finds nothing — the idempotency signal.
    let r = repair_drop_duplicate_facets(&mut scene);
    assert_eq!(r.dropped_triangles, 0);
}

#[test]
fn reversed_winding_twin_counts_as_duplicate() {
    // (A, B, C) and (A, C, B) cover the same surface patch — the
    // unordered corner-triple key must collapse them.
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = vec![
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        // reversed winding of the same three corners:
        [0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
    ];
    let mut scene = Scene3D::new();
    scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
    let r = repair_drop_duplicate_facets(&mut scene);
    assert_eq!(r.dropped_triangles, 1);
    assert_eq!(scene.meshes[0].primitives[0].positions.len(), 3);
}

#[test]
fn indexed_path_preserves_u16_discriminant() {
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = vec![[0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0]];
    // Face 0 = (0,1,2), face 1 = duplicate of face 0, face 2 = (1,3,2).
    prim.indices = Some(Indices::U16(vec![0, 1, 2, 0, 1, 2, 1, 3, 2]));
    let mut scene = Scene3D::new();
    scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
    let r = repair_drop_duplicate_facets(&mut scene);
    assert_eq!(r.dropped_triangles, 1);
    match &scene.meshes[0].primitives[0].indices {
        Some(Indices::U16(idx)) => assert_eq!(idx, &vec![0u16, 1, 2, 1, 3, 2]),
        _ => panic!("U16 discriminant should be preserved"),
    }
}

#[test]
fn distinct_faces_sharing_two_corners_are_not_duplicates() {
    // Two triangles that share an edge but differ in the third corner
    // are a normal manifold pair — must NOT be dropped.
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = vec![
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
    ];
    let mut scene = Scene3D::new();
    scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
    let r = repair_drop_duplicate_facets(&mut scene);
    assert_eq!(r.dropped_triangles, 0);
    assert_eq!(scene.meshes[0].primitives[0].positions.len(), 6);
}

#[test]
fn re_encode_after_dedup_drops_the_triangle_count() {
    let bytes = build_binary_with_exact_duplicate();
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let _ = repair_drop_duplicate_facets(&mut scene);
    let out = StlEncoder::new_binary().encode(&scene).expect("encode ok");
    assert_eq!(out.len(), 84 + 2 * 50);
    let tri_count = u32::from_le_bytes(out[80..84].try_into().unwrap());
    assert_eq!(tri_count, 2);
}

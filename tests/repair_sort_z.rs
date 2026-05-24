//! Integration tests for `repair_sort_triangles_by_z` exercised
//! through the public decoder + repair + encoder API.
//!
//! The 1989 spec notes: "Sorting the triangles in ascending z-value
//! order is recommended, but not required, in order to optimize
//! performance of the slice program." This repair materialises that
//! recommendation as a pure, count-preserving reordering of every
//! `Triangles` primitive's facets.

use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};
use oxideav_stl::{repair_sort_triangles_by_z, StlDecoder, StlEncoder};

fn push_vec3(out: &mut Vec<u8>, v: [f32; 3]) {
    out.extend_from_slice(&v[0].to_le_bytes());
    out.extend_from_slice(&v[1].to_le_bytes());
    out.extend_from_slice(&v[2].to_le_bytes());
}

/// Build a binary STL whose `flat-at-z` facets are emitted in a
/// deliberately z-shuffled order. Returns the bytes plus the emit-order
/// z list so the test can assert the post-sort permutation.
fn build_binary_shuffled_z(zs: &[f32]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(84 + zs.len() * 50);
    bytes.extend_from_slice(&[0u8; 80]);
    bytes.extend_from_slice(&(zs.len() as u32).to_le_bytes());
    for &z in zs {
        push_vec3(&mut bytes, [0.0, 0.0, 1.0]); // stored normal
        push_vec3(&mut bytes, [0.0, 0.0, z]); // v0
        push_vec3(&mut bytes, [1.0, 0.0, z]); // v1
        push_vec3(&mut bytes, [0.0, 1.0, z]); // v2
        bytes.extend_from_slice(&[0u8; 2]); // attr
    }
    bytes
}

/// The per-face minimum vertex z, in emit order, of the first
/// primitive of the first mesh.
fn face_min_zs(scene: &oxideav_mesh3d::Scene3D) -> Vec<f32> {
    let prim = &scene.meshes[0].primitives[0];
    let face_count = prim.positions.len() / 3;
    (0..face_count)
        .map(|f| {
            let b = f * 3;
            prim.positions[b][2]
                .min(prim.positions[b + 1][2])
                .min(prim.positions[b + 2][2])
        })
        .collect()
}

#[test]
fn sort_after_binary_decode_orders_ascending() {
    let bytes = build_binary_shuffled_z(&[5.0, 1.0, 9.0, 3.0, 7.0]);
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    // Pre: decoder preserves the shuffled emit order.
    assert_eq!(face_min_zs(&scene), vec![5.0, 1.0, 9.0, 3.0, 7.0]);
    let r = repair_sort_triangles_by_z(&mut scene);
    assert_eq!(r.triangles_inspected, 5);
    assert!(r.triangles_reordered > 0);
    // Post: ascending z order.
    assert_eq!(face_min_zs(&scene), vec![1.0, 3.0, 5.0, 7.0, 9.0]);
}

#[test]
fn sort_is_idempotent_through_decoder() {
    let bytes = build_binary_shuffled_z(&[4.0, 2.0, 8.0, 6.0]);
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let first = repair_sort_triangles_by_z(&mut scene);
    assert!(first.triangles_reordered > 0);
    let second = repair_sort_triangles_by_z(&mut scene);
    assert_eq!(second.triangles_reordered, 0);
    assert_eq!(face_min_zs(&scene), vec![2.0, 4.0, 6.0, 8.0]);
}

#[test]
fn sort_round_trips_through_binary_encoder_in_order() {
    let bytes = build_binary_shuffled_z(&[6.0, 2.0, 4.0]);
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    repair_sort_triangles_by_z(&mut scene);
    let out = StlEncoder::new_binary().encode(&scene).expect("encode ok");
    // Re-read each facet's v0.z from the encoded stream. Each facet
    // record is 50 bytes starting at 84; v0.z is at record offset
    // 12 (normal=12) + 8 (v0.x,v0.y) = 20.
    let face_z = |i: usize| {
        let base = 84 + i * 50 + 20;
        f32::from_le_bytes(out[base..base + 4].try_into().unwrap())
    };
    assert_eq!(face_z(0), 2.0);
    assert_eq!(face_z(1), 4.0);
    assert_eq!(face_z(2), 6.0);
    // Triangle count is preserved.
    let count = u32::from_le_bytes(out[80..84].try_into().unwrap());
    assert_eq!(count, 3);
}

#[test]
fn sort_preserves_triangle_count_and_geometry_set() {
    let zs = [9.0_f32, 1.0, 5.0, 2.0, 8.0, 3.0, 7.0, 4.0, 6.0, 0.0];
    let bytes = build_binary_shuffled_z(&zs);
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = repair_sort_triangles_by_z(&mut scene);
    assert_eq!(r.triangles_inspected, zs.len());
    let post = face_min_zs(&scene);
    assert_eq!(post.len(), zs.len());
    // Same multiset of z-values, now ascending.
    let mut want: Vec<f32> = zs.to_vec();
    want.sort_by(f32::total_cmp);
    assert_eq!(post, want);
}

#[test]
fn sort_through_ascii_decode_then_binary_encode() {
    // ASCII source, shuffled z; sort; re-emit as binary in order.
    let ascii = "\
solid s
facet normal 0 0 1
outer loop
vertex 0 0 3
vertex 1 0 3
vertex 0 1 3
endloop
endfacet
facet normal 0 0 1
outer loop
vertex 0 0 1
vertex 1 0 1
vertex 0 1 1
endloop
endfacet
endsolid s
";
    let mut scene = StlDecoder::new()
        .decode(ascii.as_bytes())
        .expect("ascii decode ok");
    assert_eq!(face_min_zs(&scene), vec![3.0, 1.0]);
    let r = repair_sort_triangles_by_z(&mut scene);
    assert_eq!(r.triangles_reordered, 2);
    assert_eq!(face_min_zs(&scene), vec![1.0, 3.0]);
}

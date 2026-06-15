//! Integration tests for `check_z_sorted` — the non-mutating
//! diagnostic counterpart of `repair_sort_triangles_by_z`, exercised
//! through the public decoder + repair API.
//!
//! The 1989 spec notes: "Sorting the triangles in ascending z-value
//! order is recommended, but not required, in order to optimize
//! performance of the slice program." `check_z_sorted` answers whether
//! a decoded scene already meets that recommendation, without paying
//! for the re-sort.

use oxideav_mesh3d::Mesh3DDecoder;
use oxideav_stl::{check_z_sorted, repair_sort_triangles_by_z, StlDecoder};

fn push_vec3(out: &mut Vec<u8>, v: [f32; 3]) {
    out.extend_from_slice(&v[0].to_le_bytes());
    out.extend_from_slice(&v[1].to_le_bytes());
    out.extend_from_slice(&v[2].to_le_bytes());
}

/// Build a binary STL whose `flat-at-z` facets are emitted in the given
/// z order.
fn build_binary_z(zs: &[f32]) -> Vec<u8> {
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

#[test]
fn already_sorted_binary_reports_sorted() {
    let bytes = build_binary_z(&[1.0, 2.0, 3.0, 4.0]);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = check_z_sorted(&scene);
    assert!(r.is_sorted());
    assert_eq!(r.triangles_inspected, 4);
    assert_eq!(r.out_of_order_pairs, 0);
    assert_eq!(r.first_out_of_order_triangle, None);
}

#[test]
fn shuffled_binary_reports_unsorted_with_first_break() {
    let bytes = build_binary_z(&[5.0, 1.0, 9.0, 3.0, 7.0]);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = check_z_sorted(&scene);
    assert!(!r.is_sorted());
    assert_eq!(r.triangles_inspected, 5);
    // First descent is triangle 2 (z=1 after z=5).
    assert_eq!(r.first_out_of_order_triangle, Some(2));
}

#[test]
fn diagnostic_clears_after_repair() {
    let bytes = build_binary_z(&[9.0, 1.0, 5.0, 2.0, 8.0]);
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    assert!(!check_z_sorted(&scene).is_sorted());
    repair_sort_triangles_by_z(&mut scene);
    // The repair targets exactly the order the diagnostic checks for.
    assert!(check_z_sorted(&scene).is_sorted());
}

#[test]
fn diagnostic_agrees_with_repair_zero_reorder() {
    // Acceptance parity through the decoder: `is_sorted()` is true iff
    // the repair would reorder nothing on the same scene.
    for zs in [
        vec![1.0_f32, 2.0, 3.0],
        vec![3.0, 2.0, 1.0],
        vec![1.0, 3.0, 2.0],
        vec![2.0, 2.0, 2.0],
        vec![4.0, 2.0, 8.0, 6.0],
    ] {
        let bytes = build_binary_z(&zs);
        let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
        let diag = check_z_sorted(&scene);
        let mut clone = scene.clone();
        let rep = repair_sort_triangles_by_z(&mut clone);
        assert_eq!(
            diag.is_sorted(),
            rep.triangles_reordered == 0,
            "mismatch for {zs:?}"
        );
        assert_eq!(diag.triangles_inspected, rep.triangles_inspected);
    }
}

#[test]
fn diagnostic_does_not_mutate_decoded_scene() {
    let bytes = build_binary_z(&[9.0, 2.0, 7.0]);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    // `Scene3D` has no `PartialEq`; observe the shuffled emit order is
    // preserved (the diagnostic never reorders).
    let prim = &scene.meshes[0].primitives[0];
    let before: Vec<f32> = (0..prim.positions.len() / 3)
        .map(|f| prim.positions[f * 3][2])
        .collect();
    let _ = check_z_sorted(&scene);
    let prim = &scene.meshes[0].primitives[0];
    let after: Vec<f32> = (0..prim.positions.len() / 3)
        .map(|f| prim.positions[f * 3][2])
        .collect();
    assert_eq!(before, after);
    assert_eq!(after, vec![9.0, 2.0, 7.0]);
}

#[test]
fn ascii_decoded_unsorted_reports_first_break() {
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
    let scene = StlDecoder::new()
        .decode(ascii.as_bytes())
        .expect("ascii decode ok");
    let r = check_z_sorted(&scene);
    assert!(!r.is_sorted());
    assert_eq!(r.triangles_inspected, 2);
    assert_eq!(r.first_out_of_order_triangle, Some(2));
}

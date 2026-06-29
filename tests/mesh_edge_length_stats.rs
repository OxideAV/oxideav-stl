//! Integration tests for `mesh_edge_length_stats` — the non-mutating
//! triangle-side / facet-size diagnostic, exercised through the public
//! decoder + topology API.
//!
//! The 1989 spec notes the official 3D Systems document specifies data
//! for the "minimum length of triangle side" and "maximum triangle
//! size"; this diagnostic materialises those quantities (plus
//! companions) straight from the geometry.

use oxideav_mesh3d::Mesh3DDecoder;
use oxideav_stl::{mesh_edge_length_stats, StlDecoder};

fn push_vec3(out: &mut Vec<u8>, v: [f32; 3]) {
    out.extend_from_slice(&v[0].to_le_bytes());
    out.extend_from_slice(&v[1].to_le_bytes());
    out.extend_from_slice(&v[2].to_le_bytes());
}

fn push_facet(out: &mut Vec<u8>, n: [f32; 3], a: [f32; 3], b: [f32; 3], c: [f32; 3]) {
    push_vec3(out, n);
    push_vec3(out, a);
    push_vec3(out, b);
    push_vec3(out, c);
    out.extend_from_slice(&[0u8; 2]);
}

fn build_binary(facets: &[([f32; 3], [f32; 3], [f32; 3])]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(84 + facets.len() * 50);
    bytes.extend_from_slice(&[0u8; 80]);
    bytes.extend_from_slice(&(facets.len() as u32).to_le_bytes());
    for &(a, b, c) in facets {
        push_facet(&mut bytes, [0.0, 0.0, 0.0], a, b, c);
    }
    bytes
}

#[test]
fn unit_right_triangle_extents() {
    // Corners (0,0,0), (3,0,0), (0,4,0): sides 3, 5, 4 (a 3-4-5 right
    // triangle), area = ½·3·4 = 6.
    let bytes = build_binary(&[([0., 0., 0.], [3., 0., 0.], [0., 4., 0.])]);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_edge_length_stats(&scene);
    assert_eq!(r.triangles_summed, 1);
    assert_eq!(r.edges_summed, 3);
    assert!(!r.had_non_finite);
    assert!((r.min_edge_length.unwrap() - 3.0).abs() < 1e-6);
    assert!((r.max_edge_length.unwrap() - 5.0).abs() < 1e-6);
    assert!((r.max_triangle_span.unwrap() - 5.0).abs() < 1e-6);
    assert!((r.mean_edge_length().unwrap() - 4.0).abs() < 1e-6); // (3+4+5)/3
    assert!((r.min_face_area.unwrap() - 6.0).abs() < 1e-6);
    assert!((r.max_face_area.unwrap() - 6.0).abs() < 1e-6);
    assert!((r.edge_length_spread().unwrap() - 5.0 / 3.0).abs() < 1e-6);
}

#[test]
fn two_facets_distinct_sizes() {
    // A small facet (side 1) and a large one (side 10).
    let small = ([0., 0., 0.], [1., 0., 0.], [0., 1., 0.]);
    let large = ([0., 0., 0.], [10., 0., 0.], [0., 10., 0.]);
    let bytes = build_binary(&[small, large]);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_edge_length_stats(&scene);
    assert_eq!(r.triangles_summed, 2);
    assert_eq!(r.edges_summed, 6);
    // smallest side anywhere is 1 (the small facet's legs).
    assert!((r.min_edge_length.unwrap() - 1.0).abs() < 1e-6);
    // largest side is the large facet's hypotenuse 10·√2.
    assert!((r.max_edge_length.unwrap() - 10.0 * 2f64.sqrt()).abs() < 1e-4);
    // small area ½, large area 50.
    assert!((r.min_face_area.unwrap() - 0.5).abs() < 1e-6);
    assert!((r.max_face_area.unwrap() - 50.0).abs() < 1e-4);
}

#[test]
fn coincident_corner_yields_zero_min_edge() {
    // Two corners coincide ⇒ a zero-length side ⇒ min edge 0, and the
    // spread is None (can't divide by a zero minimum).
    let bytes = build_binary(&[([1., 1., 1.], [1., 1., 1.], [2., 1., 1.])]);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_edge_length_stats(&scene);
    assert_eq!(r.triangles_summed, 1);
    assert_eq!(r.edges_summed, 3);
    assert_eq!(r.min_edge_length, Some(0.0));
    assert_eq!(r.edge_length_spread(), None);
    // Degenerate facet ⇒ zero area.
    assert_eq!(r.min_face_area, Some(0.0));
}

#[test]
fn translation_invariant() {
    let base = ([0., 0., 0.], [3., 0., 0.], [0., 4., 0.]);
    let shift = [100.0_f32, -50.0, 7.0];
    let s = |p: [f32; 3]| [p[0] + shift[0], p[1] + shift[1], p[2] + shift[2]];
    let bytes = build_binary(&[(s(base.0), s(base.1), s(base.2))]);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_edge_length_stats(&scene);
    // Side lengths and area are translation-invariant.
    assert!((r.min_edge_length.unwrap() - 3.0).abs() < 1e-3);
    assert!((r.max_edge_length.unwrap() - 5.0).abs() < 1e-3);
    assert!((r.max_face_area.unwrap() - 6.0).abs() < 1e-3);
}

#[test]
fn empty_scene_all_none() {
    let bytes = build_binary(&[]);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_edge_length_stats(&scene);
    assert_eq!(r.triangles_summed, 0);
    assert_eq!(r.edges_summed, 0);
    assert_eq!(r.min_edge_length, None);
    assert_eq!(r.max_edge_length, None);
    assert_eq!(r.max_triangle_span, None);
    assert_eq!(r.min_face_area, None);
    assert_eq!(r.max_face_area, None);
    assert_eq!(r.mean_edge_length(), None);
    assert_eq!(r.edge_length_spread(), None);
    assert!(!r.had_non_finite);
}

#[test]
fn max_span_equals_max_edge() {
    // max_triangle_span is documented to equal max_edge_length.
    let bytes = build_binary(&[
        ([0., 0., 0.], [2., 0., 0.], [0., 2., 0.]),
        ([0., 0., 0.], [7., 0., 0.], [0., 1., 0.]),
    ]);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_edge_length_stats(&scene);
    assert_eq!(r.max_triangle_span, r.max_edge_length);
}

#[test]
fn ascii_matches_binary() {
    let facets = [
        ([0., 0., 0.], [3., 0., 0.], [0., 4., 0.]),
        ([1., 1., 1.], [2., 1., 1.], [1., 3., 1.]),
    ];
    let bin = build_binary(&facets);
    let bin_scene = StlDecoder::new().decode(&bin).expect("decode bin");
    let bin_r = mesh_edge_length_stats(&bin_scene);

    let mut ascii = String::from("solid m\n");
    for (a, b, c) in &facets {
        ascii.push_str("facet normal 0 0 0\nouter loop\n");
        for v in [a, b, c] {
            ascii.push_str(&format!("vertex {} {} {}\n", v[0], v[1], v[2]));
        }
        ascii.push_str("endloop\nendfacet\n");
    }
    ascii.push_str("endsolid m\n");
    let ascii_scene = StlDecoder::new()
        .decode(ascii.as_bytes())
        .expect("decode ascii");
    let ascii_r = mesh_edge_length_stats(&ascii_scene);

    assert_eq!(bin_r.triangles_summed, ascii_r.triangles_summed);
    assert!((bin_r.min_edge_length.unwrap() - ascii_r.min_edge_length.unwrap()).abs() < 1e-6);
    assert!((bin_r.max_edge_length.unwrap() - ascii_r.max_edge_length.unwrap()).abs() < 1e-6);
    assert!((bin_r.max_face_area.unwrap() - ascii_r.max_face_area.unwrap()).abs() < 1e-6);
}

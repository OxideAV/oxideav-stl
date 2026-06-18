//! Integration tests for `mesh_surface_area` — the non-mutating total
//! surface-area diagnostic, exercised through the public decoder +
//! topology API.
//!
//! The total area is `Σ ½·|(v1−v0) × (v2−v0)|` over every triangle
//! facet. Because it is a magnitude it is winding- and origin-
//! independent (a facet and its reversed-winding twin have the same
//! area; a rigid translation leaves it unchanged) and is well-defined
//! even for an open sheet, unlike the enclosed volume.

use oxideav_mesh3d::Mesh3DDecoder;
use oxideav_stl::{mesh_surface_area, StlDecoder};

fn push_vec3(out: &mut Vec<u8>, v: [f32; 3]) {
    out.extend_from_slice(&v[0].to_le_bytes());
    out.extend_from_slice(&v[1].to_le_bytes());
    out.extend_from_slice(&v[2].to_le_bytes());
}

/// Append one binary-STL facet (stored normal + three corners + attr).
fn push_facet(out: &mut Vec<u8>, n: [f32; 3], a: [f32; 3], b: [f32; 3], c: [f32; 3]) {
    push_vec3(out, n);
    push_vec3(out, a);
    push_vec3(out, b);
    push_vec3(out, c);
    out.extend_from_slice(&[0u8; 2]);
}

/// Build a binary STL from a list of `(v0, v1, v2)` facets (zero stored
/// normals — `mesh_surface_area` reads positions, not the stored normal).
fn build_binary(facets: &[([f32; 3], [f32; 3], [f32; 3])]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(84 + facets.len() * 50);
    bytes.extend_from_slice(&[0u8; 80]);
    bytes.extend_from_slice(&(facets.len() as u32).to_le_bytes());
    for &(a, b, c) in facets {
        push_facet(&mut bytes, [0.0, 0.0, 0.0], a, b, c);
    }
    bytes
}

/// The 12 facets of an axis-aligned unit cube `[0,1]³`. Surface area is
/// exactly 6.0 (six unit-area faces), regardless of winding.
fn unit_cube() -> Vec<([f32; 3], [f32; 3], [f32; 3])> {
    let v = |x: f32, y: f32, z: f32| [x, y, z];
    let (a, b, c, d) = (v(0., 0., 0.), v(1., 0., 0.), v(1., 1., 0.), v(0., 1., 0.));
    let (e, f, g, h) = (v(0., 0., 1.), v(1., 0., 1.), v(1., 1., 1.), v(0., 1., 1.));
    vec![
        (a, c, b),
        (a, d, c),
        (e, f, g),
        (e, g, h),
        (a, b, f),
        (a, f, e),
        (d, h, g),
        (d, g, c),
        (a, e, h),
        (a, h, d),
        (b, c, g),
        (b, g, f),
    ]
}

#[test]
fn unit_cube_area_is_six() {
    let bytes = build_binary(&unit_cube());
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_surface_area(&scene);
    assert_eq!(r.triangles_summed, 12);
    assert!(!r.had_non_finite);
    assert!(
        (r.total_area - 6.0).abs() < 1e-9,
        "expected 6.0, got {}",
        r.total_area
    );
    // 12 facets, each ½ unit area ⇒ mean 0.5.
    assert!((r.mean_face_area().unwrap() - 0.5).abs() < 1e-9);
}

#[test]
fn winding_does_not_affect_area() {
    // Reverse every facet's winding (swap last two corners). Area is a
    // magnitude, so the total is identical to the un-flipped cube.
    let flipped: Vec<_> = unit_cube().into_iter().map(|(a, b, c)| (a, c, b)).collect();
    let bytes = build_binary(&flipped);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_surface_area(&scene);
    assert!((r.total_area - 6.0).abs() < 1e-9, "got {}", r.total_area);
}

#[test]
fn translation_invariant() {
    let shift = [3.5_f32, -2.0, 7.25];
    let shifted: Vec<_> = unit_cube()
        .into_iter()
        .map(|(a, b, c)| {
            let s = |p: [f32; 3]| [p[0] + shift[0], p[1] + shift[1], p[2] + shift[2]];
            (s(a), s(b), s(c))
        })
        .collect();
    let bytes = build_binary(&shifted);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_surface_area(&scene);
    assert!((r.total_area - 6.0).abs() < 1e-6, "got {}", r.total_area);
}

#[test]
fn scaling_squares_the_area() {
    // A 2× cube has 4× the surface area (area scales with the square of
    // the linear factor).
    let scaled: Vec<_> = unit_cube()
        .into_iter()
        .map(|(a, b, c)| {
            let s = |p: [f32; 3]| [p[0] * 2.0, p[1] * 2.0, p[2] * 2.0];
            (s(a), s(b), s(c))
        })
        .collect();
    let bytes = build_binary(&scaled);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_surface_area(&scene);
    assert!((r.total_area - 24.0).abs() < 1e-6, "got {}", r.total_area);
}

#[test]
fn open_sheet_has_meaningful_area() {
    // A single unit right-triangle (open surface): area ½, and the
    // diagnostic reports it even though no volume is enclosed.
    let bytes = build_binary(&[([0., 0., 0.], [1., 0., 0.], [0., 1., 0.])]);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_surface_area(&scene);
    assert_eq!(r.triangles_summed, 1);
    assert!((r.total_area - 0.5).abs() < 1e-9, "got {}", r.total_area);
    assert!((r.mean_face_area().unwrap() - 0.5).abs() < 1e-9);
}

#[test]
fn degenerate_facet_contributes_zero() {
    // Three collinear corners ⇒ zero-magnitude cross product ⇒ 0 area,
    // but the facet still counts toward triangles_summed.
    let bytes = build_binary(&[([0., 0., 0.], [1., 0., 0.], [2., 0., 0.])]);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_surface_area(&scene);
    assert_eq!(r.triangles_summed, 1);
    assert_eq!(r.total_area, 0.0);
}

#[test]
fn empty_scene_reports_zero() {
    let bytes = build_binary(&[]);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_surface_area(&scene);
    assert_eq!(r.triangles_summed, 0);
    assert_eq!(r.total_area, 0.0);
    assert!(!r.had_non_finite);
    assert_eq!(r.mean_face_area(), None);
}

#[test]
fn ascii_decoded_cube_matches_binary() {
    let mut ascii = String::from("solid cube\n");
    for (a, b, c) in &unit_cube() {
        ascii.push_str("facet normal 0 0 0\nouter loop\n");
        for v in [a, b, c] {
            ascii.push_str(&format!("vertex {} {} {}\n", v[0], v[1], v[2]));
        }
        ascii.push_str("endloop\nendfacet\n");
    }
    ascii.push_str("endsolid cube\n");
    let scene = StlDecoder::new()
        .decode(ascii.as_bytes())
        .expect("ascii decode ok");
    let r = mesh_surface_area(&scene);
    assert_eq!(r.triangles_summed, 12);
    assert!((r.total_area - 6.0).abs() < 1e-6, "got {}", r.total_area);
}

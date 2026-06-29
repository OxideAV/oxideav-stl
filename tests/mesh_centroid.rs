//! Integration tests for `mesh_centroid` — the non-mutating area- and
//! volume-weighted centroid diagnostic, exercised through the public
//! decoder + topology API.
//!
//! The area-weighted surface centroid is well-defined for any non-empty
//! surface; the volume-weighted centroid is a true centre of mass only
//! for a closed mesh. Build-plate centering is the spec-adjacent use
//! case (the positive-octant + ascending-z recommendations are both
//! about placement).

use oxideav_mesh3d::Mesh3DDecoder;
use oxideav_stl::{mesh_centroid, StlDecoder};

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

/// 12 outward-wound facets of an axis-aligned cube `[lo,hi]³`.
fn cube(lo: f32, hi: f32) -> Vec<([f32; 3], [f32; 3], [f32; 3])> {
    let v = |x: f32, y: f32, z: f32| [x, y, z];
    let (a, b, c, d) = (v(lo, lo, lo), v(hi, lo, lo), v(hi, hi, lo), v(lo, hi, lo));
    let (e, f, g, h) = (v(lo, lo, hi), v(hi, lo, hi), v(hi, hi, hi), v(lo, hi, hi));
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

fn approx3(a: [f64; 3], b: [f64; 3], eps: f64) -> bool {
    (a[0] - b[0]).abs() < eps && (a[1] - b[1]).abs() < eps && (a[2] - b[2]).abs() < eps
}

#[test]
fn unit_cube_centroid_is_half() {
    let bytes = build_binary(&cube(0.0, 1.0));
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_centroid(&scene);
    assert_eq!(r.triangles_summed, 12);
    assert!(!r.had_non_finite);
    assert!((r.signed_volume - 1.0).abs() < 1e-6);
    assert!((r.total_area - 6.0).abs() < 1e-6);
    assert!(approx3(r.area_centroid().unwrap(), [0.5, 0.5, 0.5], 1e-6));
    assert!(approx3(r.volume_centroid().unwrap(), [0.5, 0.5, 0.5], 1e-6));
}

#[test]
fn off_origin_cube_centroid_tracks_box() {
    // A cube spanning [2,6]³ centres at (4,4,4).
    let bytes = build_binary(&cube(2.0, 6.0));
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_centroid(&scene);
    assert!(approx3(r.area_centroid().unwrap(), [4.0, 4.0, 4.0], 1e-3));
    assert!(approx3(r.volume_centroid().unwrap(), [4.0, 4.0, 4.0], 1e-3));
    // Volume of a 4-edge cube is 64.
    assert!((r.signed_volume - 64.0).abs() < 1e-2);
}

#[test]
fn open_triangle_has_area_centroid_only() {
    // A single right triangle in z=0: area centroid is its barycentre;
    // it passes through the origin so signed volume is zero ⇒ no volume
    // centroid.
    let bytes = build_binary(&[([0., 0., 0.], [6., 0., 0.], [0., 6., 0.])]);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_centroid(&scene);
    assert_eq!(r.triangles_summed, 1);
    assert!(approx3(r.area_centroid().unwrap(), [2.0, 2.0, 0.0], 1e-6));
    assert_eq!(r.volume_centroid(), None);
}

#[test]
fn winding_flip_preserves_volume_centroid() {
    let normal = build_binary(&cube(1.0, 3.0));
    let flipped: Vec<_> = cube(1.0, 3.0)
        .into_iter()
        .map(|(a, b, c)| (a, c, b))
        .collect();
    let flipped_bytes = build_binary(&flipped);

    let sn = StlDecoder::new().decode(&normal).expect("n");
    let sf = StlDecoder::new().decode(&flipped_bytes).expect("f");
    let rn = mesh_centroid(&sn);
    let rf = mesh_centroid(&sf);
    // Signed volume flips sign; the centroid (a ratio) is unchanged.
    assert!((rn.signed_volume + rf.signed_volume).abs() < 1e-4);
    assert!(approx3(
        rn.volume_centroid().unwrap(),
        rf.volume_centroid().unwrap(),
        1e-5
    ));
}

#[test]
fn empty_scene_no_centroids() {
    let bytes = build_binary(&[]);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_centroid(&scene);
    assert_eq!(r.triangles_summed, 0);
    assert_eq!(r.area_centroid(), None);
    assert_eq!(r.volume_centroid(), None);
    assert!(!r.had_non_finite);
}

#[test]
fn ascii_matches_binary() {
    let facets = cube(0.0, 2.0);
    let bin = build_binary(&facets);
    let bin_r = mesh_centroid(&StlDecoder::new().decode(&bin).expect("bin"));

    let mut ascii = String::from("solid c\n");
    for (a, b, c) in &facets {
        ascii.push_str("facet normal 0 0 0\nouter loop\n");
        for v in [a, b, c] {
            ascii.push_str(&format!("vertex {} {} {}\n", v[0], v[1], v[2]));
        }
        ascii.push_str("endloop\nendfacet\n");
    }
    ascii.push_str("endsolid c\n");
    let ascii_r = mesh_centroid(&StlDecoder::new().decode(ascii.as_bytes()).expect("ascii"));

    assert!(approx3(
        bin_r.area_centroid().unwrap(),
        ascii_r.area_centroid().unwrap(),
        1e-6
    ));
    assert!(approx3(
        bin_r.volume_centroid().unwrap(),
        ascii_r.volume_centroid().unwrap(),
        1e-6
    ));
}

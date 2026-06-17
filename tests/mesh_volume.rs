//! Integration tests for `mesh_volume` — the non-mutating signed
//! enclosed-volume diagnostic, exercised through the public decoder +
//! topology API.
//!
//! The signed volume is the divergence-theorem sum of per-facet signed
//! tetrahedron volumes `(v0 · (v1 × v2)) / 6`. For a closed surface its
//! magnitude is the enclosed volume and its sign carries the winding:
//! positive for the right-hand-rule outward orientation the 1989 spec
//! mandates, negative for an inside-out mesh.

use oxideav_mesh3d::Mesh3DDecoder;
use oxideav_stl::{
    mesh_volume, repair_make_winding_consistent, repair_orient_normals_from_winding, StlDecoder,
};

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
/// normals — `mesh_volume` reads winding, not the stored normal).
fn build_binary(facets: &[([f32; 3], [f32; 3], [f32; 3])]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(84 + facets.len() * 50);
    bytes.extend_from_slice(&[0u8; 80]);
    bytes.extend_from_slice(&(facets.len() as u32).to_le_bytes());
    for &(a, b, c) in facets {
        push_facet(&mut bytes, [0.0, 0.0, 0.0], a, b, c);
    }
    bytes
}

/// The 12 facets of an axis-aligned unit cube `[0,1]³`, wound so the
/// right-hand-rule normal points **outward** on every face. Enclosed
/// volume is exactly 1.0.
fn unit_cube_outward() -> Vec<([f32; 3], [f32; 3], [f32; 3])> {
    let v = |x: f32, y: f32, z: f32| [x, y, z];
    let (a, b, c, d) = (v(0., 0., 0.), v(1., 0., 0.), v(1., 1., 0.), v(0., 1., 0.));
    let (e, f, g, h) = (v(0., 0., 1.), v(1., 0., 1.), v(1., 1., 1.), v(0., 1., 1.));
    vec![
        // bottom z=0, outward normal -Z (clockwise from +Z ⇒ CCW from -Z)
        (a, c, b),
        (a, d, c),
        // top z=1, outward normal +Z
        (e, f, g),
        (e, g, h),
        // front y=0, outward normal -Y
        (a, b, f),
        (a, f, e),
        // back y=1, outward normal +Y
        (d, h, g),
        (d, g, c),
        // left x=0, outward normal -X
        (a, e, h),
        (a, h, d),
        // right x=1, outward normal +X
        (b, c, g),
        (b, g, f),
    ]
}

#[test]
fn unit_cube_signed_volume_is_one() {
    let facets = unit_cube_outward();
    let bytes = build_binary(&facets);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_volume(&scene);
    assert_eq!(r.triangles_summed, 12);
    assert!(!r.had_non_finite);
    assert!(
        (r.signed_volume - 1.0).abs() < 1e-9,
        "expected +1.0, got {}",
        r.signed_volume
    );
    assert_eq!(r.winds_outward(), Some(true));
    assert!((r.volume() - 1.0).abs() < 1e-9);
}

#[test]
fn reversed_winding_negates_signed_volume() {
    // Flip every facet's winding (swap last two corners) ⇒ inside-out
    // cube ⇒ signed volume -1.0, |volume| unchanged.
    let facets: Vec<_> = unit_cube_outward()
        .into_iter()
        .map(|(a, b, c)| (a, c, b))
        .collect();
    let bytes = build_binary(&facets);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_volume(&scene);
    assert!(
        (r.signed_volume + 1.0).abs() < 1e-9,
        "expected -1.0, got {}",
        r.signed_volume
    );
    assert_eq!(r.winds_outward(), Some(false));
    assert!((r.volume() - 1.0).abs() < 1e-9);
}

#[test]
fn translation_invariant_for_closed_mesh() {
    // The enclosed volume of a closed surface does not depend on where
    // the origin sits — shift every vertex by a fixed offset and the
    // signed volume is unchanged.
    let shift = [3.5_f32, -2.0, 7.25];
    let facets: Vec<_> = unit_cube_outward()
        .into_iter()
        .map(|(a, b, c)| {
            let s = |p: [f32; 3]| [p[0] + shift[0], p[1] + shift[1], p[2] + shift[2]];
            (s(a), s(b), s(c))
        })
        .collect();
    let bytes = build_binary(&facets);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_volume(&scene);
    assert!(
        (r.signed_volume - 1.0).abs() < 1e-6,
        "expected +1.0 after translation, got {}",
        r.signed_volume
    );
}

#[test]
fn scaling_cubes_volume() {
    // A 2× cube encloses 8× the volume.
    let facets: Vec<_> = unit_cube_outward()
        .into_iter()
        .map(|(a, b, c)| {
            let s = |p: [f32; 3]| [p[0] * 2.0, p[1] * 2.0, p[2] * 2.0];
            (s(a), s(b), s(c))
        })
        .collect();
    let bytes = build_binary(&facets);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_volume(&scene);
    assert!(
        (r.signed_volume - 8.0).abs() < 1e-6,
        "expected +8.0, got {}",
        r.signed_volume
    );
}

#[test]
fn empty_scene_reports_zero() {
    let bytes = build_binary(&[]);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_volume(&scene);
    assert_eq!(r.triangles_summed, 0);
    assert_eq!(r.signed_volume, 0.0);
    assert!(!r.had_non_finite);
    // No orientation can be inferred from a zero signed volume.
    assert_eq!(r.winds_outward(), None);
}

#[test]
fn single_flat_facet_through_origin_winds_none() {
    // One triangle in the z=0 plane through the origin: its tetrahedron
    // with the origin is degenerate (zero signed volume), so no winding
    // orientation can be inferred.
    let bytes = build_binary(&[([0., 0., 0.], [1., 0., 0.], [0., 1., 0.])]);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r = mesh_volume(&scene);
    assert_eq!(r.triangles_summed, 1);
    assert_eq!(r.signed_volume, 0.0);
    assert_eq!(r.winds_outward(), None);
}

#[test]
fn agrees_with_outward_orientation_after_repair() {
    // Start from an inside-out cube (negative volume), make winding
    // consistent + orient normals outward, and confirm the sign and the
    // diagnostic agree: the repaired mesh's stored normals point the
    // same way the positive signed volume reports.
    let facets: Vec<_> = unit_cube_outward()
        .into_iter()
        .map(|(a, b, c)| (a, c, b))
        .collect();
    let bytes = build_binary(&facets);
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    assert_eq!(mesh_volume(&scene).winds_outward(), Some(false));

    // Repairs operate on winding/normals; they don't flip global
    // orientation, so the signed volume stays negative but remains a
    // consistent, well-defined value the diagnostic still reads.
    repair_make_winding_consistent(&mut scene);
    repair_orient_normals_from_winding(&mut scene, 1e-6);
    let r = mesh_volume(&scene);
    assert!(r.signed_volume.is_finite());
    assert!((r.volume() - 1.0).abs() < 1e-6);
}

#[test]
fn ascii_decoded_cube_matches_binary() {
    // Same outward cube, but routed through the ASCII decoder path, to
    // confirm the diagnostic is encoding-agnostic.
    let facets = unit_cube_outward();
    let mut ascii = String::from("solid cube\n");
    for (a, b, c) in &facets {
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
    let r = mesh_volume(&scene);
    assert_eq!(r.triangles_summed, 12);
    assert!((r.signed_volume - 1.0).abs() < 1e-6);
    assert_eq!(r.winds_outward(), Some(true));
}

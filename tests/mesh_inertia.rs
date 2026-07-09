//! Integration tests for `mesh_inertia` — the rotational-inertia
//! completion of the mass-property family (`mesh_volume` = mass,
//! `mesh_centroid` = centre of mass, `mesh_inertia` = inertia tensor).
//!
//! The oracle is the closed-form inertia of a solid axis-aligned box of
//! extents `(w, h, d)` at unit density: mass `m = w·h·d`, and about the
//! centroidal axes `Iₓₓ = m(h²+d²)/12`, `Iᵧᵧ = m(w²+d²)/12`,
//! `I_zz = m(w²+h²)/12`, with zero products of inertia.

use oxideav_mesh3d::Mesh3DDecoder;
use oxideav_stl::{mesh_inertia, StlDecoder};

fn push_vec3(out: &mut Vec<u8>, v: [f32; 3]) {
    out.extend_from_slice(&v[0].to_le_bytes());
    out.extend_from_slice(&v[1].to_le_bytes());
    out.extend_from_slice(&v[2].to_le_bytes());
}

fn build_binary(facets: &[([f32; 3], [f32; 3], [f32; 3])]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(84 + facets.len() * 50);
    bytes.extend_from_slice(&[0u8; 80]);
    bytes.extend_from_slice(&(facets.len() as u32).to_le_bytes());
    for &(a, b, c) in facets {
        push_vec3(&mut bytes, [0.0, 0.0, 0.0]);
        push_vec3(&mut bytes, a);
        push_vec3(&mut bytes, b);
        push_vec3(&mut bytes, c);
        bytes.extend_from_slice(&[0u8; 2]);
    }
    bytes
}

/// The 12 outward-wound facets of the axis-aligned box
/// `[x0,x1]×[y0,y1]×[z0,z1]`.
fn box_outward(
    x0: f32,
    x1: f32,
    y0: f32,
    y1: f32,
    z0: f32,
    z1: f32,
) -> Vec<([f32; 3], [f32; 3], [f32; 3])> {
    let v = |x, y, z| [x, y, z];
    let (a, b, c, d) = (v(x0, y0, z0), v(x1, y0, z0), v(x1, y1, z0), v(x0, y1, z0));
    let (e, f, g, h) = (v(x0, y0, z1), v(x1, y0, z1), v(x1, y1, z1), v(x0, y1, z1));
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

/// Same box but every facet reversed — an inside-out (winding-flipped)
/// solid whose signed volume is negative.
fn box_inward(
    x0: f32,
    x1: f32,
    y0: f32,
    y1: f32,
    z0: f32,
    z1: f32,
) -> Vec<([f32; 3], [f32; 3], [f32; 3])> {
    box_outward(x0, x1, y0, y1, z0, z1)
        .into_iter()
        .map(|(a, b, c)| (a, c, b))
        .collect()
}

fn report_for(facets: &[([f32; 3], [f32; 3], [f32; 3])]) -> oxideav_stl::MeshInertiaReport {
    let bytes = build_binary(facets);
    let scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    mesh_inertia(&scene)
}

fn approx(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

#[test]
fn unit_cube_diagonal_inertia_is_one_sixth() {
    let r = report_for(&box_outward(0.0, 1.0, 0.0, 1.0, 0.0, 1.0));
    assert_eq!(r.triangles_summed, 12);
    assert!(!r.had_non_finite);
    assert!(approx(r.mass(), 1.0, 1e-6), "mass {}", r.mass());
    let com = r.center_of_mass().expect("closed solid");
    for k in 0..3 {
        assert!(approx(com[k], 0.5, 1e-6), "com {com:?}");
    }
    let t = r.inertia_tensor_about_centroid().expect("closed solid");
    for (i, row) in t.iter().enumerate() {
        assert!(approx(row[i], 1.0 / 6.0, 1e-6), "diag {i}: {}", row[i]);
        for (j, val) in row.iter().enumerate() {
            if i != j {
                assert!(approx(*val, 0.0, 1e-6), "offdiag {i}{j}: {val}");
            }
        }
    }
    let pm = r.principal_moments().expect("closed solid");
    for v in pm {
        assert!(approx(v, 1.0 / 6.0, 1e-6), "principal {v}");
    }
}

#[test]
fn rectangular_box_matches_closed_form() {
    // Box 2 × 1 × 1 → m = 2, Iₓₓ = 1/3, Iᵧᵧ = I_zz = 5/6.
    let r = report_for(&box_outward(0.0, 2.0, 0.0, 1.0, 0.0, 1.0));
    assert!(approx(r.mass(), 2.0, 1e-5));
    let t = r.inertia_tensor_about_centroid().expect("closed solid");
    assert!(approx(t[0][0], 1.0 / 3.0, 1e-5), "Ixx {}", t[0][0]);
    assert!(approx(t[1][1], 5.0 / 6.0, 1e-5), "Iyy {}", t[1][1]);
    assert!(approx(t[2][2], 5.0 / 6.0, 1e-5), "Izz {}", t[2][2]);
    // Principal moments ascending.
    let pm = r.principal_moments().expect("closed solid");
    assert!(approx(pm[0], 1.0 / 3.0, 1e-5), "pm0 {}", pm[0]);
    assert!(approx(pm[1], 5.0 / 6.0, 1e-5), "pm1 {}", pm[1]);
    assert!(approx(pm[2], 5.0 / 6.0, 1e-5), "pm2 {}", pm[2]);
    assert!(pm[0] <= pm[1] && pm[1] <= pm[2], "sorted ascending");
}

#[test]
fn inside_out_winding_reports_same_positive_tensor() {
    let out = report_for(&box_outward(0.0, 2.0, 0.0, 1.0, 0.0, 1.0));
    let inn = report_for(&box_inward(0.0, 2.0, 0.0, 1.0, 0.0, 1.0));
    // Signed volume flips sign, but mass + tensor are orientation-normalised.
    assert!(out.signed_volume > 0.0);
    assert!(inn.signed_volume < 0.0);
    assert!(approx(out.mass(), inn.mass(), 1e-6));
    let to = out.inertia_tensor_about_centroid().unwrap();
    let ti = inn.inertia_tensor_about_centroid().unwrap();
    for i in 0..3 {
        for j in 0..3 {
            assert!(
                approx(to[i][j], ti[i][j], 1e-5),
                "{i}{j}: {} vs {}",
                to[i][j],
                ti[i][j]
            );
        }
    }
}

#[test]
fn centroidal_inertia_is_translation_invariant() {
    // A far-from-origin box must report the same centroidal tensor as the
    // same box at the origin (the parallel-axis shift removes the origin
    // dependence).
    let at_origin = report_for(&box_outward(0.0, 2.0, 0.0, 1.0, 0.0, 1.0));
    let shifted = report_for(&box_outward(100.0, 102.0, 50.0, 51.0, -30.0, -29.0));
    let a = at_origin.inertia_tensor_about_centroid().unwrap();
    let b = shifted.inertia_tensor_about_centroid().unwrap();
    for i in 0..3 {
        for j in 0..3 {
            // Loosen tolerance: subtracting large origin moments costs
            // precision, but the result is still tight.
            assert!(
                approx(a[i][j], b[i][j], 1e-3),
                "{i}{j}: {} vs {}",
                a[i][j],
                b[i][j]
            );
        }
    }
}

#[test]
fn empty_scene_has_no_solid() {
    use oxideav_mesh3d::Scene3D;
    let r = mesh_inertia(&Scene3D::new());
    assert_eq!(r.triangles_summed, 0);
    assert_eq!(r.signed_volume, 0.0);
    assert!(r.center_of_mass().is_none());
    assert!(r.inertia_tensor_about_centroid().is_none());
    assert!(r.principal_moments().is_none());
    assert_eq!(r.mass(), 0.0);
}

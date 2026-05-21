//! T-junction detection — round-trip-through-the-decoder coverage of
//! the new `ValidationOptions::check_t_junctions` rule.
//!
//! The spec's vertex-to-vertex rule says "a vertex of one triangle
//! cannot lie on the side (edge) of another triangle". The watertight
//! edge-use check alone misses this: the offending vertex is *not* an
//! endpoint of the edge it sits on, so canonical edge keys don't
//! collide. The T-junction sub-check supplements it.

use oxideav_mesh3d::Mesh3DDecoder;
use oxideav_stl::{
    validate, StlDecoder, ValidationOptions, DEFAULT_T_JUNCTION_TOLERANCE, MAX_REPORTED_DEFECTS,
};

/// Build an ASCII STL where the bottom edge of one triangle is split
/// into two triangles by a midpoint vertex — the canonical
/// T-junction. Layout:
///
/// ```text
///       (1, 2)
///       /    \
///      /      \
///   (0,0)-(1,0)-(2,0)
///     /    \  /    \
///  (0.5,-1) (1.5,-1)
/// ```
///
/// Top triangle: (0, 0)-(2, 0)-(1, 2). Bottom-left:
/// (0, 0)-(1, 0)-(0.5, -1). Bottom-right: (1, 0)-(2, 0)-(1.5, -1).
/// The (1, 0) corner of the two bottom triangles sits in the middle
/// of the top triangle's (0, 0)-(2, 0) edge — a T-junction.
fn t_junction_ascii() -> &'static str {
    "solid t\n\
        facet normal 0 0 1\n\
        outer loop\n\
        vertex 0 0 0\n\
        vertex 2 0 0\n\
        vertex 1 2 0\n\
        endloop\n\
        endfacet\n\
        facet normal 0 0 1\n\
        outer loop\n\
        vertex 0 0 0\n\
        vertex 1 0 0\n\
        vertex 0.5 -1 0\n\
        endloop\n\
        endfacet\n\
        facet normal 0 0 1\n\
        outer loop\n\
        vertex 1 0 0\n\
        vertex 2 0 0\n\
        vertex 1.5 -1 0\n\
        endloop\n\
        endfacet\n\
        endsolid t\n"
}

#[test]
fn t_junction_default_options_does_not_run_check() {
    let scene = StlDecoder::new()
        .decode(t_junction_ascii().as_bytes())
        .unwrap();
    let r = validate(&scene, &ValidationOptions::default());
    // 3 triangles total — facet orientation + unit normal off-by-
    // default, watertight on. Two of the three edges of each
    // bottom triangle are open boundaries, so the watertight check
    // fires — but T-junctions stays silent.
    assert_eq!(r.triangles_total, 3);
    assert_eq!(r.t_junction_defects, 0);
    assert!(r.t_junction_examples.is_empty());
}

#[test]
fn t_junction_opt_in_flags_split_edge() {
    let scene = StlDecoder::new()
        .decode(t_junction_ascii().as_bytes())
        .unwrap();
    let opts = ValidationOptions {
        check_t_junctions: true,
        // Disable the other rules so the report carries just the
        // T-junction signal — the bottom triangles are open meshes
        // by design.
        check_facet_orientation: false,
        check_unit_normal: false,
        check_watertight: false,
        ..ValidationOptions::default()
    };
    let r = validate(&scene, &opts);
    // The (1, 0, 0) midpoint sits on the top triangle's edge; both
    // bottom triangles own that corner, so both are flagged.
    assert!(r.t_junction_defects >= 2, "report: {r:?}");
    assert!(r.t_junction_examples.len() >= 2);
    // Examples are facet locators inside the decoded scene; the
    // first mesh holds all three triangles in source order
    // (top = face 0, bottom-left = face 1, bottom-right = face 2).
    // Either bottom triangle being in the example list is the test's
    // pass condition.
    let faces: Vec<usize> = r.t_junction_examples.iter().map(|loc| loc.face).collect();
    assert!(faces.contains(&1) || faces.contains(&2), "faces: {faces:?}");
    // is_clean() must drop to false.
    assert!(!r.is_clean());
}

#[test]
fn t_junction_clean_two_triangle_strip_is_clean() {
    // Two triangles sharing a full edge — a corner-on-corner
    // adjacency, NOT a T-junction. The watertight check sees 4
    // boundary edges (open mesh), but the T-junction check stays
    // silent because no vertex sits in the *middle* of any edge.
    let s = "solid strip\n\
        facet normal 0 0 1\n\
        outer loop\n\
        vertex 0 0 0\n\
        vertex 1 0 0\n\
        vertex 0 1 0\n\
        endloop\n\
        endfacet\n\
        facet normal 0 0 1\n\
        outer loop\n\
        vertex 1 0 0\n\
        vertex 1 1 0\n\
        vertex 0 1 0\n\
        endloop\n\
        endfacet\n\
        endsolid strip\n";
    let scene = StlDecoder::new().decode(s.as_bytes()).unwrap();
    let opts = ValidationOptions {
        check_t_junctions: true,
        check_facet_orientation: false,
        check_unit_normal: false,
        check_watertight: false,
        ..ValidationOptions::default()
    };
    let r = validate(&scene, &opts);
    assert_eq!(r.t_junction_defects, 0, "report: {r:?}");
}

#[test]
fn t_junction_default_tolerance_is_one_per_hundred_thousand() {
    // The constant is part of the public API; pin its value so
    // downstream consumers that derive their own tolerances stay
    // in sync.
    assert_eq!(DEFAULT_T_JUNCTION_TOLERANCE, 1.0e-5);
}

#[test]
fn t_junction_example_list_respects_max_cap() {
    // Generate a strip where 100 successive triangles each split
    // the spine of an above triangle, so the T-junction count
    // explodes past the 32-example cap. The example list must
    // saturate at MAX_REPORTED_DEFECTS while the count keeps
    // climbing.
    let mut out = String::from("solid many\n");
    // One big "roof" triangle from (0, 0) to (100, 0) — the long
    // edge is the spine the bottom triangles split.
    out.push_str(
        "facet normal 0 0 1\n\
            outer loop\n\
            vertex 0 0 0\n\
            vertex 100 0 0\n\
            vertex 50 50 0\n\
            endloop\n\
            endfacet\n",
    );
    // 100 strip triangles, each consuming a 1-unit span of the
    // (0, 0)-(100, 0) spine. The first corner of each is a
    // midpoint of the roof's bottom edge — a T-junction.
    for i in 0..100 {
        let xa = i as f32;
        let xb = (i + 1) as f32;
        let xc = i as f32 + 0.5;
        out.push_str(&format!(
            "facet normal 0 0 1\n\
                outer loop\n\
                vertex {xa} 0 0\n\
                vertex {xb} 0 0\n\
                vertex {xc} -1 0\n\
                endloop\n\
                endfacet\n",
        ));
    }
    out.push_str("endsolid many\n");
    let scene = StlDecoder::new().decode(out.as_bytes()).unwrap();
    let opts = ValidationOptions {
        check_t_junctions: true,
        check_facet_orientation: false,
        check_unit_normal: false,
        check_watertight: false,
        ..ValidationOptions::default()
    };
    let r = validate(&scene, &opts);
    // 99 interior midpoints sit on the roof's bottom edge (the (0,0)
    // and (100,0) endpoints are not midpoints). Each midpoint is
    // owned by two strip triangles, so the defect count is 198 ±
    // a small fudge depending on exact overlap detection.
    assert!(r.t_junction_defects > MAX_REPORTED_DEFECTS, "report: {r:?}");
    assert_eq!(r.t_junction_examples.len(), MAX_REPORTED_DEFECTS);
}

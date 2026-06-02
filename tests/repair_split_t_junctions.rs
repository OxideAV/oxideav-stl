//! Integration coverage for `repair_split_t_junctions` — the
//! mutating fix-up for `ValidationOptions::check_t_junctions`.
//!
//! Each test takes a fixture through the decode → repair → re-validate
//! cycle so the repair's effect is measured against the same
//! diagnostic the validate module exposes for the T-junction sub-check
//! (the spec's vertex-to-vertex rule).

use oxideav_mesh3d::Mesh3DDecoder;
use oxideav_stl::{
    repair_split_t_junctions, validate, StlDecoder, ValidationOptions,
    DEFAULT_T_JUNCTION_SPLIT_TOLERANCE, DEFAULT_T_JUNCTION_TOLERANCE,
};

/// Classic T-junction: one big triangle's edge `(0,0,0) → (2,0,0)`
/// is split by the `(1,0,0)` corner shared by two bottom triangles.
/// After the repair, the bottom edge of the top triangle is replaced
/// by two sub-edges that pass through (1,0,0), so the watertight
/// edge-use count + the T-junction sub-check both go clean.
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

fn opts_for_tjunction_only() -> ValidationOptions {
    ValidationOptions {
        check_t_junctions: true,
        check_facet_orientation: false,
        check_unit_normal: false,
        check_watertight: false,
        check_consistent_winding: false,
        ..ValidationOptions::default()
    }
}

#[test]
fn repair_drops_validate_t_junction_count_to_zero() {
    let mut scene = StlDecoder::new()
        .decode(t_junction_ascii().as_bytes())
        .unwrap();
    let opts = opts_for_tjunction_only();

    // Pre-repair: validate reports the classic T-junction defect.
    let pre = validate(&scene, &opts);
    assert!(pre.t_junction_defects > 0, "pre: {pre:?}");

    // Repair.
    let r = repair_split_t_junctions(&mut scene, DEFAULT_T_JUNCTION_SPLIT_TOLERANCE);
    assert!(r.triangles_split > 0, "report: {r:?}");
    assert!(r.split_vertices_inserted > 0);
    assert!(r.triangles_emitted >= r.triangles_split * 2);

    // Post-repair: the same diagnostic at the same tolerance is
    // clean.
    let post = validate(&scene, &opts);
    assert_eq!(
        post.t_junction_defects, 0,
        "validate after repair: {post:?}",
    );
}

#[test]
fn repair_is_idempotent_on_clean_scene() {
    let mut scene = StlDecoder::new()
        .decode(t_junction_ascii().as_bytes())
        .unwrap();
    let r1 = repair_split_t_junctions(&mut scene, DEFAULT_T_JUNCTION_SPLIT_TOLERANCE);
    assert!(r1.triangles_split > 0);
    // Second pass: the fixture is now T-junction-clean; no faces
    // split.
    let r2 = repair_split_t_junctions(&mut scene, DEFAULT_T_JUNCTION_SPLIT_TOLERANCE);
    assert_eq!(r2.triangles_split, 0);
    assert_eq!(r2.split_vertices_inserted, 0);
    assert_eq!(r2.triangles_emitted, 0);
}

#[test]
fn repair_preserves_triangle_topology_count_balance() {
    // Pre: 3 triangles. Repair: 1 split (top triangle) into 2
    // sub-triangles. Total post: 4 triangles.
    let mut scene = StlDecoder::new()
        .decode(t_junction_ascii().as_bytes())
        .unwrap();
    let pre_total: usize = scene
        .meshes
        .iter()
        .flat_map(|m| m.primitives.iter())
        .map(|p| match &p.indices {
            Some(idx) => idx.len() / 3,
            None => p.positions.len() / 3,
        })
        .sum();
    assert_eq!(pre_total, 3);
    let _ = repair_split_t_junctions(&mut scene, DEFAULT_T_JUNCTION_SPLIT_TOLERANCE);
    let post_total: usize = scene
        .meshes
        .iter()
        .flat_map(|m| m.primitives.iter())
        .map(|p| match &p.indices {
            Some(idx) => idx.len() / 3,
            None => p.positions.len() / 3,
        })
        .sum();
    assert_eq!(post_total, 4);
}

#[test]
fn repair_default_tolerance_matches_validate_default() {
    // The constants `DEFAULT_T_JUNCTION_SPLIT_TOLERANCE` (repair) and
    // `DEFAULT_T_JUNCTION_TOLERANCE` (validate) are documented as
    // equal so a scene that detects-and-repairs-and-re-detects at
    // either default is consistent.
    assert_eq!(
        DEFAULT_T_JUNCTION_SPLIT_TOLERANCE, DEFAULT_T_JUNCTION_TOLERANCE,
        "default tolerances must match for the diagnostic↔repair pairing"
    );
}

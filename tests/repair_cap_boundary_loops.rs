//! Integration coverage for `repair_cap_boundary_loops` — the
//! mutating fix-up that triangulates closed naked-edge (boundary)
//! loops, restoring the 1989 spec's closed-surface invariant ("each
//! facet is part of the boundary between the interior and the exterior
//! of the object").
//!
//! Each test takes a fixture through the decode → cap → re-validate
//! cycle so the repair's effect is measured against the same
//! watertight diagnostic the validate module exposes and the same
//! `boundary_loops` extraction the cap is the fix-up for.

use oxideav_mesh3d::Mesh3DDecoder;
use oxideav_stl::{
    boundary_loops, repair_cap_boundary_loops, validate, StlDecoder, ValidationOptions,
};

/// A tetrahedron (A,B,C,D) missing its `(B,C,D)` face: three
/// outward-wound faces leave a single triangular hole bounded by the
/// three edges `B–C`, `C–D`, `D–B`. Capping that loop restores the
/// fourth face, making the solid watertight.
///
///   A = (0,0,0)  B = (1,0,0)  C = (0,1,0)  D = (0,0,1)
fn open_tetra_ascii() -> &'static str {
    "solid tetra\n\
        facet normal 0 0 -1\n\
        outer loop\n\
        vertex 0 0 0\n\
        vertex 0 1 0\n\
        vertex 1 0 0\n\
        endloop\n\
        endfacet\n\
        facet normal 0 -1 0\n\
        outer loop\n\
        vertex 0 0 0\n\
        vertex 1 0 0\n\
        vertex 0 0 1\n\
        endloop\n\
        endfacet\n\
        facet normal -1 0 0\n\
        outer loop\n\
        vertex 0 0 0\n\
        vertex 0 0 1\n\
        vertex 0 1 0\n\
        endloop\n\
        endfacet\n\
        endsolid tetra\n"
}

fn opts_watertight_only() -> ValidationOptions {
    ValidationOptions {
        check_watertight: true,
        check_facet_orientation: false,
        check_unit_normal: false,
        check_consistent_winding: false,
        ..ValidationOptions::default()
    }
}

#[test]
fn cap_closes_triangular_hole_to_watertight() {
    let mut scene = StlDecoder::new()
        .decode(open_tetra_ascii().as_bytes())
        .unwrap();
    let opts = opts_watertight_only();

    // Pre: three faces, a triangular hole → 3 boundary edges, not
    // watertight, exactly one closed boundary loop of 3 edges.
    let pre = validate(&scene, &opts);
    assert!(!pre.watertight, "pre: {pre:?}");
    assert_eq!(pre.boundary_edges, 3, "pre: {pre:?}");
    let loops = boundary_loops(&scene);
    assert_eq!(loops.len(), 1);
    assert!(loops[0].closed);
    assert_eq!(loops[0].edge_count(), 3);

    // Cap.
    let r = repair_cap_boundary_loops(&mut scene);
    assert_eq!(r.loops_capped, 1, "report: {r:?}");
    // A 3-edge loop caps to 3 - 2 = 1 fan triangle.
    assert_eq!(r.cap_triangles_emitted, 1, "report: {r:?}");
    assert_eq!(r.open_chains_skipped, 0);
    assert_eq!(r.degenerate_loops_skipped, 0);

    // Post: closed surface, no boundary edges, no boundary loops.
    let post = validate(&scene, &opts);
    assert!(post.watertight, "post: {post:?}");
    assert_eq!(post.boundary_edges, 0, "post: {post:?}");
    assert!(boundary_loops(&scene).is_empty());

    // Face count went from 3 to 4.
    let faces: usize = scene
        .meshes
        .iter()
        .flat_map(|m| m.primitives.iter())
        .map(|p| match &p.indices {
            Some(idx) => idx.len() / 3,
            None => p.positions.len() / 3,
        })
        .sum();
    assert_eq!(faces, 4);
}

#[test]
fn cap_is_idempotent_on_closed_scene() {
    let mut scene = StlDecoder::new()
        .decode(open_tetra_ascii().as_bytes())
        .unwrap();
    let r1 = repair_cap_boundary_loops(&mut scene);
    assert_eq!(r1.loops_capped, 1);
    // Second pass: the solid is now closed; nothing to cap.
    let r2 = repair_cap_boundary_loops(&mut scene);
    assert_eq!(r2.loops_capped, 0, "report: {r2:?}");
    assert_eq!(r2.cap_triangles_emitted, 0);
    assert_eq!(r2.open_chains_skipped, 0);
}

/// A square hole (4 boundary edges) caps to `4 - 2 = 2` fan triangles.
/// Build an open "box lid" hole: a cube with its top face removed has
/// a square boundary loop along the top rim. Here we use the minimal
/// shape that produces a single 4-edge closed loop: a square pyramid
/// (apex + 4 base corners) with its square base left open — the four
/// triangular side faces leave the base rim as one 4-edge loop.
///
///   apex T = (0.5, 0.5, 1)
///   base  P0=(0,0,0) P1=(1,0,0) P2=(1,1,0) P3=(0,1,0)
fn open_pyramid_ascii() -> &'static str {
    "solid pyr\n\
        facet normal 0 0 0\n\
        outer loop\n\
        vertex 0 0 0\n\
        vertex 1 0 0\n\
        vertex 0.5 0.5 1\n\
        endloop\n\
        endfacet\n\
        facet normal 0 0 0\n\
        outer loop\n\
        vertex 1 0 0\n\
        vertex 1 1 0\n\
        vertex 0.5 0.5 1\n\
        endloop\n\
        endfacet\n\
        facet normal 0 0 0\n\
        outer loop\n\
        vertex 1 1 0\n\
        vertex 0 1 0\n\
        vertex 0.5 0.5 1\n\
        endloop\n\
        endfacet\n\
        facet normal 0 0 0\n\
        outer loop\n\
        vertex 0 1 0\n\
        vertex 0 0 0\n\
        vertex 0.5 0.5 1\n\
        endloop\n\
        endfacet\n\
        endsolid pyr\n"
}

#[test]
fn cap_closes_square_hole_with_two_triangles() {
    let mut scene = StlDecoder::new()
        .decode(open_pyramid_ascii().as_bytes())
        .unwrap();
    let opts = opts_watertight_only();

    let pre = validate(&scene, &opts);
    assert!(!pre.watertight, "pre: {pre:?}");
    assert_eq!(pre.boundary_edges, 4, "pre: {pre:?}");
    let loops = boundary_loops(&scene);
    assert_eq!(loops.len(), 1);
    assert!(loops[0].closed);
    assert_eq!(loops[0].edge_count(), 4);

    let r = repair_cap_boundary_loops(&mut scene);
    assert_eq!(r.loops_capped, 1, "report: {r:?}");
    assert_eq!(r.cap_triangles_emitted, 2, "report: {r:?}");

    let post = validate(&scene, &opts);
    assert!(post.watertight, "post: {post:?}");
    assert_eq!(post.boundary_edges, 0, "post: {post:?}");

    let faces: usize = scene
        .meshes
        .iter()
        .flat_map(|m| m.primitives.iter())
        .map(|p| match &p.indices {
            Some(idx) => idx.len() / 3,
            None => p.positions.len() / 3,
        })
        .sum();
    // 4 sides + 2 cap = 6.
    assert_eq!(faces, 6);
}

#[test]
fn cap_leaves_watertight_scene_untouched() {
    // A closed tetrahedron (all four faces) has no boundary loops, so
    // the cap is a no-op from the start.
    let closed = "solid t\n\
        facet normal 0 0 -1\n\
        outer loop\n\
        vertex 0 0 0\nvertex 0 1 0\nvertex 1 0 0\n\
        endloop\nendfacet\n\
        facet normal 0 -1 0\n\
        outer loop\n\
        vertex 0 0 0\nvertex 1 0 0\nvertex 0 0 1\n\
        endloop\nendfacet\n\
        facet normal -1 0 0\n\
        outer loop\n\
        vertex 0 0 0\nvertex 0 0 1\nvertex 0 1 0\n\
        endloop\nendfacet\n\
        facet normal 1 1 1\n\
        outer loop\n\
        vertex 1 0 0\nvertex 0 1 0\nvertex 0 0 1\n\
        endloop\nendfacet\n\
        endsolid t\n";
    let mut scene = StlDecoder::new().decode(closed.as_bytes()).unwrap();
    assert!(boundary_loops(&scene).is_empty());
    let r = repair_cap_boundary_loops(&mut scene);
    assert_eq!(r.loops_capped, 0, "report: {r:?}");
    assert_eq!(r.cap_triangles_emitted, 0);
}

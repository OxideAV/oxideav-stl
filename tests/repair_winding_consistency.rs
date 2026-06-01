//! Integration tests for `repair_make_winding_consistent` exercised
//! through the public decoder + repair + validate APIs.
//!
//! The 1989 spec's facet-orientation rule (§6.5) says the three
//! vertices of every triangle are "listed in counterclockwise order
//! when looking at the object from the outside (right-hand rule)" and
//! that the two pieces of orientation information "must be
//! consistent". `validate` surfaces facets whose winding disagrees
//! with a manifold-edge neighbour's under
//! `inconsistent_winding_edges`; this repair is the matching
//! mutating fix-up.
//!
//! End-to-end: a two-triangle binary STL whose second face is wound
//! backwards relative to its neighbour decodes; validate flags 1
//! inconsistent edge; the repair flips the second face; validate
//! re-runs cleanly.

use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};
use oxideav_stl::{
    repair_make_winding_consistent, validate, StlDecoder, StlEncoder, ValidationOptions,
};

fn push_vec3(out: &mut Vec<u8>, v: [f32; 3]) {
    out.extend_from_slice(&v[0].to_le_bytes());
    out.extend_from_slice(&v[1].to_le_bytes());
    out.extend_from_slice(&v[2].to_le_bytes());
}

fn push_triangle(out: &mut Vec<u8>, n: [f32; 3], v0: [f32; 3], v1: [f32; 3], v2: [f32; 3]) {
    push_vec3(out, n);
    push_vec3(out, v0);
    push_vec3(out, v1);
    push_vec3(out, v2);
    out.extend_from_slice(&[0u8; 2]); // attribute byte count
}

/// Two-triangle binary STL forming a unit-square quad in the XY
/// plane split along the (0,0,0)−(1,1,0) diagonal. Tri 0 is
/// canonically wound; tri 1 is FLIPPED (walks the shared diagonal in
/// the same direction as tri 0, which the validate-module's
/// `inconsistent_winding` rule flags).
fn build_binary_flipped_quad() -> Vec<u8> {
    let mut bytes = Vec::with_capacity(84 + 100);
    bytes.extend_from_slice(&[0u8; 80]); // header
    bytes.extend_from_slice(&2u32.to_le_bytes()); // triangle count
                                                  // tri 0 — CCW around +Z. Walks diagonal (1,1,0) → (0,0,0).
    push_triangle(
        &mut bytes,
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
    );
    // tri 1 — INTENTIONALLY flipped. Also walks (1,1,0) → (0,0,0).
    push_triangle(
        &mut bytes,
        [0.0, 0.0, 1.0],
        [1.0, 1.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    );
    bytes
}

#[test]
fn winding_consistency_after_binary_decode_flips_flipped_neighbour() {
    let bytes = build_binary_flipped_quad();
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");

    // Pre-condition: validate flags one inconsistent edge.
    let opts = ValidationOptions {
        // Per-facet orientation depends on the stored normal vs winding.
        // We're using a +Z normal which matches tri 0 but mismatches the
        // flipped tri 1 — disable that rule to isolate the mesh-wide
        // winding check.
        check_facet_orientation: false,
        check_unit_normal: false,
        ..Default::default()
    };
    let pre = validate(&scene, &opts);
    assert_eq!(
        pre.inconsistent_winding_edges, 1,
        "expected 1 inconsistent edge pre-repair, got {pre:?}"
    );

    // Apply the repair.
    let r = repair_make_winding_consistent(&mut scene);
    assert_eq!(r.triangles_inspected, 2);
    assert_eq!(r.triangles_flipped, 1, "report: {r:?}");
    assert_eq!(r.conflicting_edges, 0);

    // Post-condition: validate flags zero inconsistent edges.
    let post = validate(&scene, &opts);
    assert_eq!(
        post.inconsistent_winding_edges, 0,
        "expected zero inconsistent edges post-repair, got {post:?}"
    );
}

#[test]
fn winding_consistency_round_trips_through_binary_encoder() {
    let bytes = build_binary_flipped_quad();
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let _ = repair_make_winding_consistent(&mut scene);

    // Re-encode + re-decode + re-validate.
    let out = StlEncoder::new_binary().encode(&scene).expect("encode ok");
    let scene2 = StlDecoder::new().decode(&out).expect("decode ok");
    let opts = ValidationOptions {
        check_facet_orientation: false,
        check_unit_normal: false,
        ..Default::default()
    };
    let r = validate(&scene2, &opts);
    assert_eq!(r.inconsistent_winding_edges, 0, "report: {r:?}");
}

#[test]
fn winding_consistency_face_count_preserved() {
    // The pass never adds or removes a triangle — it's a pure flip of
    // existing vertex slots. Confirm through the decoder → repair →
    // encoder pipeline.
    let bytes = build_binary_flipped_quad();
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let pre_total: usize = scene
        .meshes
        .iter()
        .flat_map(|m| m.primitives.iter())
        .map(|p| match &p.indices {
            Some(idx) => idx.len() / 3,
            None => p.positions.len() / 3,
        })
        .sum();
    let _ = repair_make_winding_consistent(&mut scene);
    let post_total: usize = scene
        .meshes
        .iter()
        .flat_map(|m| m.primitives.iter())
        .map(|p| match &p.indices {
            Some(idx) => idx.len() / 3,
            None => p.positions.len() / 3,
        })
        .sum();
    assert_eq!(pre_total, post_total);
    assert_eq!(post_total, 2);
}

#[test]
fn winding_consistency_second_pass_is_noop() {
    let bytes = build_binary_flipped_quad();
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");
    let r1 = repair_make_winding_consistent(&mut scene);
    assert_eq!(r1.triangles_flipped, 1);
    let r2 = repair_make_winding_consistent(&mut scene);
    assert_eq!(r2.triangles_flipped, 0, "report: {r2:?}");
    assert_eq!(r2.conflicting_edges, 0);
}

#[test]
fn winding_consistency_already_consistent_scene_is_noop() {
    // Clean two-triangle quad: tri 1 walks the shared diagonal in the
    // OPPOSITE direction to tri 0 → already consistent.
    let mut bytes = Vec::with_capacity(84 + 100);
    bytes.extend_from_slice(&[0u8; 80]);
    bytes.extend_from_slice(&2u32.to_le_bytes());
    push_triangle(
        &mut bytes,
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
    );
    push_triangle(
        &mut bytes,
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
    );
    let mut scene = StlDecoder::new().decode(&bytes).expect("decode ok");

    let r = repair_make_winding_consistent(&mut scene);
    assert_eq!(r.triangles_inspected, 2);
    assert_eq!(r.triangles_flipped, 0);
    assert_eq!(r.components_visited, 1);
}

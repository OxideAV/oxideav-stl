//! Tolerance-based vertex dedup helpers.
//!
//! [`StlEncoder::stats`] reports a *bit-exact* unique-vertex count
//! that's well-defined for floats but does not collapse vertices whose
//! positions differ by floating-point noise. Real-world meshes from
//! CAD/scanner pipelines routinely emit corners that are "the same
//! corner" in design intent but differ by 1e-6 due to round-off in an
//! upstream transform stack. The tolerance helpers
//! ([`EncodeStats::with_tolerance`] +
//! [`StlEncoder::unique_vertices_with_tolerance`]) expose a separate
//! ε-equality view of the same scene, leaving the bit-exact path
//! untouched.

use std::collections::HashMap;

use oxideav_mesh3d::{Indices, Mesh, Node, Primitive, Scene3D, Topology};
use oxideav_stl::{EncodeStats, StlEncoder};

fn build_indexed_cube() -> Scene3D {
    // 8 unique corners + 12 triangles via a u32 index buffer.
    let positions: Vec<[f32; 3]> = vec![
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [1.0, 1.0, 1.0],
        [0.0, 1.0, 1.0],
    ];
    let indices: Vec<u32> = vec![
        0, 2, 1, 0, 3, 2, // bottom
        4, 5, 6, 4, 6, 7, // top
        0, 1, 5, 0, 5, 4, // front
        2, 3, 7, 2, 7, 6, // back
        1, 2, 6, 1, 6, 5, // right
        0, 4, 7, 0, 7, 3, // left
    ];
    let mesh = Mesh {
        name: Some("cube".into()),
        primitives: vec![Primitive {
            topology: Topology::Triangles,
            positions,
            normals: None,
            tangents: None,
            uvs: Vec::new(),
            colors: Vec::new(),
            joints: None,
            weights: None,
            indices: Some(Indices::U32(indices)),
            material: None,
            extras: HashMap::new(),
        }],
    };
    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(mesh);
    let mut node = Node::new();
    node.mesh = Some(mid);
    let nid = scene.add_node(node);
    scene.add_root(nid);
    scene
}

/// Build an unindexed three-triangle scene where each triangle is a
/// noisy copy of the same logical triangle. The first triangle's
/// corners are clean; the second + third copies are perturbed by a
/// uniform 1e-6 on every component.
fn build_noisy_repeated_triangle() -> Scene3D {
    let canonical = [[0.0_f32, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    let perturb = |[x, y, z]: [f32; 3], delta: f32| [x + delta, y + delta, z + delta];
    let mut positions: Vec<[f32; 3]> = Vec::new();
    positions.extend(canonical);
    positions.extend(canonical.iter().map(|v| perturb(*v, 1e-6)));
    positions.extend(canonical.iter().map(|v| perturb(*v, -1e-6)));
    let prim = Primitive {
        topology: Topology::Triangles,
        positions,
        normals: None,
        tangents: None,
        uvs: Vec::new(),
        colors: Vec::new(),
        joints: None,
        weights: None,
        indices: None,
        material: None,
        extras: HashMap::new(),
    };
    let mesh = Mesh {
        name: None,
        primitives: vec![prim],
    };
    let mut scene = Scene3D::new();
    scene.add_mesh(mesh);
    scene
}

#[test]
fn eps_zero_matches_bit_exact_path() {
    let scene = build_indexed_cube();
    let bit_exact = StlEncoder::stats(&scene);
    let tol = EncodeStats::with_tolerance(&scene, 0.0);
    // triangles + emitted_vertices come straight from the bit-exact
    // path; tolerance only affects unique_vertices.
    assert_eq!(tol.triangles, bit_exact.triangles);
    assert_eq!(tol.emitted_vertices, bit_exact.emitted_vertices);
    // Bit-exact and ε=0 must agree for finite positions.
    assert_eq!(tol.unique_vertices, bit_exact.unique_vertices);
    assert_eq!(tol.unique_vertices, 8);
}

#[test]
fn dedup_map_length_equals_emitted_vertex_count() {
    let scene = build_indexed_cube();
    let (_, dmap) = StlEncoder::unique_vertices_with_tolerance(&scene, 0.0);
    let stats = StlEncoder::stats(&scene);
    assert_eq!(dmap.len(), stats.emitted_vertices);
    assert_eq!(dmap.len(), 36);
}

#[test]
fn dedup_map_assigns_canonical_slot_indices_in_first_seen_order() {
    // Three repeated unique points → emitted_vertices = 9, unique = 3,
    // and the dedup_map should be [0, 1, 2, 0, 1, 2, 0, 1, 2] when the
    // walk hits the same three corners three times.
    let positions: Vec<[f32; 3]> = vec![
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ];
    let prim = Primitive {
        topology: Topology::Triangles,
        positions,
        normals: None,
        tangents: None,
        uvs: Vec::new(),
        colors: Vec::new(),
        joints: None,
        weights: None,
        indices: None,
        material: None,
        extras: HashMap::new(),
    };
    let mesh = Mesh {
        name: None,
        primitives: vec![prim],
    };
    let mut scene = Scene3D::new();
    scene.add_mesh(mesh);
    let (unique, dmap) = StlEncoder::unique_vertices_with_tolerance(&scene, 0.0);
    assert_eq!(unique, 3);
    assert_eq!(dmap, vec![0, 1, 2, 0, 1, 2, 0, 1, 2]);
}

#[test]
fn tight_tolerance_collapses_floating_point_noise() {
    let scene = build_noisy_repeated_triangle();
    let bit_exact = StlEncoder::stats(&scene);
    // Without tolerance the perturbed copies are distinct corners.
    assert_eq!(bit_exact.unique_vertices, 9);
    // ε = 1e-5 covers the ±1e-6 perturbation comfortably.
    let tol = EncodeStats::with_tolerance(&scene, 1.0e-5);
    assert_eq!(tol.triangles, 3);
    assert_eq!(tol.emitted_vertices, 9);
    assert_eq!(tol.unique_vertices, 3);
}

#[test]
fn tolerance_smaller_than_noise_does_not_collapse() {
    let scene = build_noisy_repeated_triangle();
    // ε = 1e-9 is far tighter than the 1e-6 perturbation — no merge.
    let tol = EncodeStats::with_tolerance(&scene, 1.0e-9);
    assert_eq!(tol.unique_vertices, 9);
}

#[test]
fn loose_tolerance_collapses_distinct_corners() {
    // With ε ≥ 1.0 even genuinely different unit-cube corners merge,
    // because every corner pair differs by at most 1.0 on each axis.
    let scene = build_indexed_cube();
    let tol = EncodeStats::with_tolerance(&scene, 1.0);
    // Every cube corner is within 1.0 of every other corner under the
    // chebyshev (component-max) metric, so they all collapse to one.
    assert_eq!(tol.unique_vertices, 1);
    // share_factor scales accordingly.
    assert!((tol.share_factor() - 36.0).abs() < 1e-6);
}

#[test]
fn dedup_map_under_tolerance_groups_noisy_copies() {
    let scene = build_noisy_repeated_triangle();
    let (unique, dmap) = StlEncoder::unique_vertices_with_tolerance(&scene, 1.0e-5);
    assert_eq!(unique, 3);
    // The walk visits the three triangles in order so each corner
    // index is assigned in the order [v0, v1, v2] of the FIRST
    // triangle, then re-used for both perturbed copies.
    assert_eq!(dmap.len(), 9);
    assert_eq!(dmap[0..3], [0, 1, 2]);
    // Perturbed copies should map back to the same canonicals.
    assert_eq!(dmap[3..6], [0, 1, 2]);
    assert_eq!(dmap[6..9], [0, 1, 2]);
}

#[test]
fn negative_or_nan_eps_clamps_to_bit_exact() {
    let scene = build_noisy_repeated_triangle();
    let bit_exact = StlEncoder::stats(&scene);
    for eps in [-1.0_f32, f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        let tol = EncodeStats::with_tolerance(&scene, eps);
        assert_eq!(
            tol.unique_vertices, bit_exact.unique_vertices,
            "eps {eps} should clamp to 0.0 (bit-exact)"
        );
    }
}

#[test]
fn empty_scene_produces_zero_unique_and_empty_dedup_map() {
    let scene = Scene3D::new();
    let (unique, dmap) = StlEncoder::unique_vertices_with_tolerance(&scene, 1.0e-3);
    assert_eq!(unique, 0);
    assert!(dmap.is_empty());
    let tol = EncodeStats::with_tolerance(&scene, 1.0e-3);
    assert_eq!(tol, EncodeStats::default());
}

#[test]
fn tolerance_walk_respects_index_buffers() {
    // Indexed cube — emit count is 36, tolerance under any sensible
    // ε ≤ 0.5 leaves the 8 unique corners distinct (every cube
    // edge has length 1.0, so ε must be < 1.0 to avoid collapse).
    let scene = build_indexed_cube();
    let (unique, dmap) = StlEncoder::unique_vertices_with_tolerance(&scene, 1.0e-3);
    assert_eq!(unique, 8);
    assert_eq!(dmap.len(), 36);
    // Every dedup-map entry is within the canonical-slot range.
    for slot in &dmap {
        assert!(*slot < 8);
    }
}

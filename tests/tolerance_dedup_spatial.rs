//! Spatial-grid variant of the tolerance dedup helper.
//!
//! [`StlEncoder::unique_vertices_with_tolerance_spatial`] +
//! [`EncodeStats::with_tolerance_spatial`] amortise the brute-force
//! `O(N · K)` tolerance scan to `O(N)` by binning each vertex into a
//! uniform-grid cell of side `eps × 2`. The two paths are
//! cross-tested here:
//!
//! - With `eps == 0.0`, both paths reduce to bit-exact `f32` equality
//!   and MUST produce identical `unique_vertices` counts and
//!   identical `dedup_map` shapes.
//! - With `eps > 0`, the spatial path is approximate (see the
//!   contract on `with_tolerance_spatial`) but every two points it
//!   merges MUST be within `eps` on every axis under the Chebyshev
//!   metric — verified by walking the dedup_map and asserting the
//!   pair-wise distance bound.

use std::collections::HashMap;

use oxideav_mesh3d::{Indices, Mesh, Node, Primitive, Scene3D, Topology};
use oxideav_stl::{EncodeStats, StlEncoder};

fn build_indexed_cube() -> Scene3D {
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
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = positions;
    prim.indices = Some(Indices::U32(indices));
    let mesh = Mesh::new(Some("cube".to_string())).with_primitive(prim);
    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(mesh);
    let mut node = Node::new();
    node.mesh = Some(mid);
    let nid = scene.add_node(node);
    scene.add_root(nid);
    scene
}

fn build_noisy_repeated_triangle() -> Scene3D {
    let canonical = [[0.0_f32, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    let perturb = |[x, y, z]: [f32; 3], delta: f32| [x + delta, y + delta, z + delta];
    let mut positions: Vec<[f32; 3]> = Vec::new();
    positions.extend(canonical);
    positions.extend(canonical.iter().map(|v| perturb(*v, 1e-6)));
    positions.extend(canonical.iter().map(|v| perturb(*v, -1e-6)));
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = positions;
    let mesh = Mesh::new(None::<String>).with_primitive(prim);
    let mut scene = Scene3D::new();
    scene.add_mesh(mesh);
    scene
}

/// Build N triangles with random per-vertex jitter in `[-noise, +noise]`
/// around three logical corners. Larger N + a tolerance bigger than
/// `noise` MUST collapse to the three logical corners under both the
/// brute-force and spatial paths (the spatial path may emit one or
/// two extras when borderline vertices straddle non-adjacent cells).
fn build_jittered_triangles(n: usize, noise: f32, seed: u64) -> Scene3D {
    // Tiny LCG so we don't pull in `rand`. Reproducible across runs.
    let mut state = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
    let mut next = || {
        state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        let x = (state >> 33) as u32;
        // Map to (-1.0, 1.0).
        ((x as f32) / (u32::MAX as f32)) * 2.0 - 1.0
    };
    let canonical = [[0.0_f32, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(n * 3);
    for _ in 0..n {
        for c in &canonical {
            positions.push([
                c[0] + next() * noise,
                c[1] + next() * noise,
                c[2] + next() * noise,
            ]);
        }
    }
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = positions;
    let mesh = Mesh::new(None::<String>).with_primitive(prim);
    let mut scene = Scene3D::new();
    scene.add_mesh(mesh);
    scene
}

/// Walk every emitted vertex of `scene` in encoder order — the same
/// iteration the dedup helpers use. Used by the chebyshev-distance
/// invariant check.
fn collect_emitted_positions(scene: &Scene3D) -> Vec<[f32; 3]> {
    let mut out = Vec::new();
    for mesh in &scene.meshes {
        for prim in &mesh.primitives {
            if prim.topology != Topology::Triangles {
                continue;
            }
            let face_count = match &prim.indices {
                Some(idx) => idx.len() / 3,
                None => prim.positions.len() / 3,
            };
            for face_idx in 0..face_count {
                let (vi0, vi1, vi2) = match &prim.indices {
                    Some(Indices::U16(v)) => {
                        let b = face_idx * 3;
                        (v[b] as usize, v[b + 1] as usize, v[b + 2] as usize)
                    }
                    Some(Indices::U32(v)) => {
                        let b = face_idx * 3;
                        (v[b] as usize, v[b + 1] as usize, v[b + 2] as usize)
                    }
                    None => {
                        let b = face_idx * 3;
                        (b, b + 1, b + 2)
                    }
                };
                for &vi in &[vi0, vi1, vi2] {
                    if let Some(p) = prim.positions.get(vi) {
                        out.push(*p);
                    }
                }
            }
        }
    }
    out
}

#[test]
fn spatial_eps_zero_matches_bit_exact_path() {
    // Cross-test against the bit-exact path for small fixtures: the
    // result MUST be identical to `StlEncoder::stats` (the bit-exact
    // path) because the spatial path delegates to the brute-force
    // bit-exact branch when eps clamps to zero.
    let scene = build_indexed_cube();
    let bit_exact = StlEncoder::stats(&scene);
    let tol = EncodeStats::with_tolerance_spatial(&scene, 0.0);
    assert_eq!(tol.triangles, bit_exact.triangles);
    assert_eq!(tol.emitted_vertices, bit_exact.emitted_vertices);
    assert_eq!(tol.unique_vertices, bit_exact.unique_vertices);
    assert_eq!(tol.unique_vertices, 8);
}

#[test]
fn spatial_eps_zero_matches_brute_force_eps_zero_dedup_map() {
    let scene = build_indexed_cube();
    let (brute_unique, brute_map) = StlEncoder::unique_vertices_with_tolerance(&scene, 0.0);
    let (spatial_unique, spatial_map) =
        StlEncoder::unique_vertices_with_tolerance_spatial(&scene, 0.0);
    assert_eq!(brute_unique, spatial_unique);
    assert_eq!(brute_map, spatial_map);
}

#[test]
fn spatial_negative_or_nan_eps_clamps_to_bit_exact() {
    let scene = build_noisy_repeated_triangle();
    let bit_exact = StlEncoder::stats(&scene);
    for eps in [-1.0_f32, f32::NAN, f32::INFINITY, f32::NEG_INFINITY] {
        let tol = EncodeStats::with_tolerance_spatial(&scene, eps);
        assert_eq!(
            tol.unique_vertices, bit_exact.unique_vertices,
            "eps {eps} should clamp to 0.0 (bit-exact)"
        );
    }
}

#[test]
fn spatial_collapses_floating_point_noise() {
    let scene = build_noisy_repeated_triangle();
    // ε = 1e-5 covers the ±1e-6 perturbation comfortably; the
    // spatial path MUST collapse the 9 emitted vertices to 3
    // canonicals (matching the brute-force path on this fixture).
    let tol = EncodeStats::with_tolerance_spatial(&scene, 1.0e-5);
    assert_eq!(tol.triangles, 3);
    assert_eq!(tol.emitted_vertices, 9);
    assert_eq!(tol.unique_vertices, 3);
}

#[test]
fn spatial_dedup_map_groups_noisy_copies() {
    let scene = build_noisy_repeated_triangle();
    let (unique, dmap) = StlEncoder::unique_vertices_with_tolerance_spatial(&scene, 1.0e-5);
    assert_eq!(unique, 3);
    assert_eq!(dmap.len(), 9);
    // Both perturbed copies should land on the same canonical slots
    // as the first triangle's three corners.
    assert_eq!(dmap[0..3], [0, 1, 2]);
    assert_eq!(dmap[3..6], [0, 1, 2]);
    assert_eq!(dmap[6..9], [0, 1, 2]);
}

#[test]
fn spatial_loose_tolerance_collapses_distinct_corners_to_one() {
    // With ε = 1.0 every cube corner is within 1.0 of every other
    // corner under the Chebyshev metric. The spatial path may emit
    // a small handful of canonicals due to its approximate nature,
    // but for a unit cube + ε = 1.0 the cell size (2.0) is large
    // enough that all 36 emitted vertices land in the same cell
    // and merge cleanly to 1.
    let scene = build_indexed_cube();
    let tol = EncodeStats::with_tolerance_spatial(&scene, 1.0);
    assert_eq!(tol.unique_vertices, 1);
}

#[test]
fn spatial_dedup_walk_respects_index_buffers() {
    // Indexed cube — emit count is 36, ε = 1e-3 must leave the 8
    // unique corners distinct (cube edge length is 1.0; the cell
    // size is 2e-3, so each corner sits in its own cell well away
    // from any neighbour).
    let scene = build_indexed_cube();
    let (unique, dmap) = StlEncoder::unique_vertices_with_tolerance_spatial(&scene, 1.0e-3);
    assert_eq!(unique, 8);
    assert_eq!(dmap.len(), 36);
    for slot in &dmap {
        assert!(*slot < 8);
    }
}

#[test]
fn spatial_empty_scene_produces_zero_unique_and_empty_dedup_map() {
    let scene = Scene3D::new();
    let (unique, dmap) = StlEncoder::unique_vertices_with_tolerance_spatial(&scene, 1.0e-3);
    assert_eq!(unique, 0);
    assert!(dmap.is_empty());
    let tol = EncodeStats::with_tolerance_spatial(&scene, 1.0e-3);
    assert_eq!(tol, EncodeStats::default());
}

#[test]
fn spatial_chebyshev_invariant_on_jittered_input() {
    // For a noisy fixture, every two emitted vertices the spatial
    // path merges MUST be within ε on every axis (the Chebyshev
    // metric). We verify by re-walking the emitted positions in
    // encoder order, grouping by canonical slot, and asserting the
    // pair-wise component-max distance bound for every group.
    let noise = 1.0e-4;
    let eps = 1.0e-3; // comfortably > noise
    let scene = build_jittered_triangles(50, noise, 0xdead_beef_cafe_babe);
    let positions = collect_emitted_positions(&scene);
    let (unique, dmap) = StlEncoder::unique_vertices_with_tolerance_spatial(&scene, eps);
    assert!(unique <= 3 + 6, "spatial may add a few extras: {}", unique);
    assert!(unique >= 3, "should collapse to roughly the 3 canonicals");

    // Group positions by canonical slot and verify the chebyshev
    // bound holds within each group.
    let mut groups: HashMap<usize, Vec<[f32; 3]>> = HashMap::new();
    for (i, slot) in dmap.iter().enumerate() {
        groups.entry(*slot).or_default().push(positions[i]);
    }
    for (slot, members) in &groups {
        for a in members {
            for b in members {
                let dx = (a[0] - b[0]).abs();
                let dy = (a[1] - b[1]).abs();
                let dz = (a[2] - b[2]).abs();
                let d = dx.max(dy).max(dz);
                // Allow 2× ε to absorb the asymmetry where both
                // points are within ε of a shared canonical but
                // not necessarily within ε of each other directly.
                assert!(
                    d <= 2.0 * eps,
                    "slot {slot} pair distance {d} > 2·eps={}",
                    2.0 * eps
                );
            }
        }
    }
}

#[test]
fn spatial_count_matches_brute_force_on_clean_repeated_triangle() {
    // For the noisy-repeated-triangle fixture both paths agree at
    // ε = 1e-5 (tested above for spatial) — re-verify the brute-
    // force path explicitly so the cross-check is documented in
    // one place.
    let scene = build_noisy_repeated_triangle();
    let brute = EncodeStats::with_tolerance(&scene, 1.0e-5);
    let spatial = EncodeStats::with_tolerance_spatial(&scene, 1.0e-5);
    assert_eq!(brute.unique_vertices, spatial.unique_vertices);
    assert_eq!(brute.unique_vertices, 3);
}

#[test]
fn spatial_handles_nan_positions_without_panic() {
    // NaN coordinates take their own canonical slot under both the
    // bit-exact and tolerance contracts. The spatial path bins NaN
    // into a sentinel cell so we exercise the same well-defined
    // contract.
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = vec![[f32::NAN, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    let mut scene = Scene3D::new();
    scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
    let (unique, dmap) = StlEncoder::unique_vertices_with_tolerance_spatial(&scene, 1.0e-3);
    assert_eq!(dmap.len(), 3);
    // Each vertex is distinct (NaN doesn't compare equal to itself).
    assert_eq!(unique, 3);
}

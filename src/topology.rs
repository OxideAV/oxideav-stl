//! Mesh topology utilities — opt-in, non-mutating analysis built on
//! the same bit-exact edge model used by [`crate::validate`].
//!
//! STL files have no native concept of connected components: the
//! format is a flat triangle soup with implicit position-equality
//! deciding which triangles "touch". Real-world files routinely
//! contain multiple disjoint shells (a print bed with three loose
//! objects on it; a CAD assembly exported as a single `.stl`), and
//! consumers downstream of the decoder regularly need to:
//!
//! 1. Split the soup into its connected components ("shells").
//! 2. Compute the Euler characteristic `χ = V − E + F` per shell so
//!    closed-genus diagnostics can fire on assemblies whose
//!    individual shells are each closed even though the file as a
//!    whole is not "watertight" by [`crate::validate`]'s global
//!    edge-use count.
//! 3. Apply trivial bit-exact-position dedup repair when a producer
//!    has emitted the *same* corner with multiple slot indices —
//!    common when an exporter writes a triangle soup without
//!    sharing vertices and a CAD pipeline downstream wants a shared
//!    index buffer back.
//!
//! This module owns those three utilities. All are pure-functional;
//! `repair_weld_vertices` is the only mutating helper and it takes
//! `&mut Scene3D` explicitly.
//!
//! ## Vertex-equality model
//!
//! Adjacency uses **bit-exact** `f32` position equality, matching
//! the rest of the crate ([`crate::EncodeStats`], the
//! `validate::watertight` rule). Callers whose corners differ by
//! floating-point noise should pre-deduplicate via
//! [`crate::StlEncoder::unique_vertices_with_tolerance_spatial`]
//! (or its brute-force sibling) before invoking these helpers —
//! that produces a `dedup_map` whose canonical slots are by
//! definition bit-exact-equal.
//!
//! ## Non-`Triangles` primitives
//!
//! Silently skipped (mirrors [`crate::validate::validate`] and the
//! rest of the crate). Encoding rejects them; counting them in a
//! topology report would be misleading.

use std::collections::HashMap;

use oxideav_mesh3d::{Indices, Primitive, Scene3D, Topology};

/// Per-shell topology summary returned by [`shells`].
///
/// A *shell* is a maximal set of triangles connected through shared
/// bit-exact vertex positions. The `face_indices` vector lists the
/// triangle locators that make up the shell in the order they were
/// first visited by the BFS; consumers can re-walk the original
/// `Scene3D` via those locators to extract per-shell geometry.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Shell {
    /// Triangle locators (mesh / primitive / face) that belong to
    /// this shell, in BFS discovery order.
    pub face_indices: Vec<FaceLocator>,
    /// Number of unique vertex positions (by bit-exact `f32` match)
    /// touched by this shell. Matches `V` in `χ = V − E + F`.
    pub vertices: usize,
    /// Number of unique undirected edges in this shell. Matches `E`.
    pub edges: usize,
    /// Number of triangles in this shell — equals `face_indices.len()`
    /// and matches `F`.
    pub faces: usize,
    /// Number of unique edges in this shell that appear in exactly
    /// one triangle (boundary edges within the shell). A shell with
    /// `boundary_edges == 0` and `non_manifold_edges == 0` is
    /// closed-and-manifold ("watertight" in the local sense).
    pub boundary_edges: usize,
    /// Number of unique edges in this shell that appear in three or
    /// more triangles (non-manifold within the shell).
    pub non_manifold_edges: usize,
}

impl Shell {
    /// Euler characteristic `χ = V − E + F`. For a closed orientable
    /// surface of genus `g`, `χ = 2 − 2g`: `χ = 2` is a topological
    /// sphere, `χ = 0` a torus, `χ = −2` a double-torus, etc. For an
    /// open surface (one with `boundary_edges > 0`), `χ` is
    /// `2 − 2g − b` where `b` is the boundary component count, which
    /// this helper does not separately enumerate.
    ///
    /// Returns `i64` because intermediate products of `usize` fields
    /// would overflow on 32-bit targets for million-triangle inputs;
    /// the difference is bounded but the operands are not.
    pub fn euler_characteristic(&self) -> i64 {
        self.vertices as i64 - self.edges as i64 + self.faces as i64
    }

    /// Whether the shell is closed and manifold — every edge shared
    /// by exactly two triangles.
    pub fn is_closed_manifold(&self) -> bool {
        self.boundary_edges == 0 && self.non_manifold_edges == 0 && self.faces > 0
    }

    /// Estimated genus for a closed orientable shell. Returns
    /// `None` when the shell is not closed-manifold (where the
    /// formula `g = (2 − χ) / 2` does not apply). The estimate
    /// silently assumes orientability — STL files with inverted
    /// normals can in principle violate this, but the genus number
    /// is still useful as a rough complexity descriptor.
    pub fn genus(&self) -> Option<i64> {
        if !self.is_closed_manifold() {
            return None;
        }
        let chi = self.euler_characteristic();
        // 2 - 2g = chi ⇒ g = (2 - chi) / 2. We only return when the
        // numerator is even — odd values indicate a non-orientable
        // or otherwise pathological surface where this formula is
        // not meaningful.
        let num = 2 - chi;
        if num % 2 == 0 {
            Some(num / 2)
        } else {
            None
        }
    }
}

/// Triangle locator within a scene — mirrors
/// [`crate::validate::FaceLocator`] but lives in this module so
/// `topology` is usable standalone.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct FaceLocator {
    /// Index into [`Scene3D::meshes`].
    pub mesh: usize,
    /// Index into the mesh's `primitives` vec.
    pub primitive: usize,
    /// Index of the triangle within the primitive (0-based,
    /// post-index-buffer resolution).
    pub face: usize,
}

/// Walk `scene` and return one [`Shell`] per connected component,
/// in the BFS-discovery order of the seed triangle (which itself
/// follows scene-graph + primitive iteration order).
///
/// Two triangles are in the same shell iff they share at least one
/// vertex position by bit-exact `f32` match (a more permissive rule
/// than "share an edge" — STL's flat triangle-soup model often has
/// shells that connect through single-corner touches, especially
/// after CSG operations, and treating them as separate would
/// mis-report the shell count). For an "edge-connected" definition,
/// post-process the result: split a shell whose every triangle
/// shares two vertices with at least one neighbour.
///
/// Returns an empty vec for an empty scene or one whose primitives
/// are all non-`Triangles`.
pub fn shells(scene: &Scene3D) -> Vec<Shell> {
    let tris = collect_triangles(scene);
    if tris.is_empty() {
        return Vec::new();
    }

    // vert_to_tris: bit-exact vertex key -> triangle indices that
    // touch it. Used both for BFS adjacency and the per-shell
    // unique-vertex / unique-edge counts.
    let mut vert_to_tris: HashMap<VertKey, Vec<usize>> = HashMap::new();
    for (ti, tri) in tris.iter().enumerate() {
        for k in &tri.keys {
            vert_to_tris.entry(*k).or_default().push(ti);
        }
    }

    let mut visited = vec![false; tris.len()];
    let mut shells = Vec::new();
    for seed in 0..tris.len() {
        if visited[seed] {
            continue;
        }
        // BFS — collect every triangle reachable through any
        // shared vertex.
        let mut queue: Vec<usize> = vec![seed];
        let mut shell_tris: Vec<usize> = Vec::new();
        visited[seed] = true;
        while let Some(t) = queue.pop() {
            shell_tris.push(t);
            for k in &tris[t].keys {
                if let Some(neighbours) = vert_to_tris.get(k) {
                    for &n in neighbours {
                        if !visited[n] {
                            visited[n] = true;
                            queue.push(n);
                        }
                    }
                }
            }
        }
        // Sort to give the report a stable shape (BFS pop order is
        // implementation-dependent — sorting by triangle index puts
        // it in original scene-graph order, which is what callers
        // would expect when re-walking the scene with locators).
        shell_tris.sort_unstable();

        // Compute V / E / boundary / non-manifold for this shell.
        let mut shell_verts: HashMap<VertKey, ()> = HashMap::new();
        let mut edge_uses: HashMap<(VertKey, VertKey), usize> = HashMap::new();
        for &t in &shell_tris {
            let tri = &tris[t];
            for k in &tri.keys {
                shell_verts.insert(*k, ());
            }
            // Canonical (lo, hi) edges so reverse-orientation
            // duplicates collide.
            let pairs = [
                (tri.keys[0], tri.keys[1]),
                (tri.keys[1], tri.keys[2]),
                (tri.keys[2], tri.keys[0]),
            ];
            for (a, b) in pairs {
                let key = if a <= b { (a, b) } else { (b, a) };
                *edge_uses.entry(key).or_insert(0) += 1;
            }
        }
        let edges = edge_uses.len();
        let mut boundary = 0usize;
        let mut non_manifold = 0usize;
        for &uses in edge_uses.values() {
            match uses {
                1 => boundary += 1,
                2 => {}
                _ if uses >= 3 => non_manifold += 1,
                _ => {}
            }
        }

        let face_indices: Vec<FaceLocator> = shell_tris.iter().map(|&t| tris[t].locator).collect();
        shells.push(Shell {
            faces: face_indices.len(),
            face_indices,
            vertices: shell_verts.len(),
            edges,
            boundary_edges: boundary,
            non_manifold_edges: non_manifold,
        });
    }
    shells
}

/// Outcome of a [`repair_weld_vertices`] pass.
///
/// Counters are summed across every `Triangles` primitive in the
/// scene. A run on an already-welded scene produces
/// `positions_collapsed == 0` (the idempotency signal); the gross
/// `slots_collapsed` field still reflects the emit-order soup →
/// indexed compression ratio, which is non-zero on every welded
/// primitive because STL's flat triangle list emits `face_count *
/// 3` vertex slots regardless of upstream sharing.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct WeldReport {
    /// Triangles inspected.
    pub triangles_inspected: usize,
    /// Number of *emitted* vertex slots that collapsed into a
    /// canonical slot. With `N` emitted slots (= `triangles * 3`)
    /// and `U` canonical positions after weld, this is `N − U`.
    /// Non-zero on any welded primitive with vertex sharing, so a
    /// fresh run on a triangle soup of 12 triangles → `36 − 8 =
    /// 28` for a shared cube. Use [`Self::positions_collapsed`] to
    /// detect "this pass actually changed something".
    pub slots_collapsed: usize,
    /// Net change in the primitive's `positions.len()` —
    /// `positions_before − positions_after`, summed across every
    /// touched primitive. Equals zero iff every primitive was
    /// already shared-indexed with one slot per unique corner; the
    /// canonical idempotency signal for a follow-up pass.
    pub positions_collapsed: usize,
    /// Number of degenerate triangles found after welding —
    /// triangles whose three indices include a duplicate (the
    /// triangle has zero area). Reported, not removed; callers who
    /// want them dropped can post-process the resulting scene.
    pub degenerate_triangles: usize,
}

/// Rewrite every `Triangles` primitive in `scene` to use a shared
/// `Indices::U32` buffer keyed on bit-exact `f32` positions.
///
/// For each primitive:
/// - Walks the *effective* triangle stream (resolving any present
///   index buffer first), so a soup-style "no index buffer + 3
///   positions per triangle" primitive becomes shared-indexed
///   afterwards.
/// - Builds a single `positions` vector of unique bit-exact
///   positions (preserving first-occurrence order).
/// - Re-assigns `prim.normals` (and `prim.indices = Some(U32)`)
///   onto the welded vertex set. Per-vertex normals from the
///   *first* occurrence of each canonical position win; subsequent
///   conflicting normals are silently dropped (the welded model
///   has a single normal per vertex, by definition).
///
/// Non-`Triangles` primitives are left untouched. The pass does
/// NOT alter `prim.extras`, `mesh.name`, or the scene-graph
/// `nodes` / `roots` structure.
///
/// Returns a single [`WeldReport`] summed across every primitive
/// touched.
///
/// ## When NOT to use
///
/// Welding by bit-exact position equality is the safe minimum. If
/// your producer emits positions that differ by floating-point
/// noise (CAD pipelines, 3D-scan tools), pre-process via
/// [`crate::StlEncoder::unique_vertices_with_tolerance_spatial`]:
/// it returns a `dedup_map` whose canonical slot indices can be
/// applied with [`super::repair_weld_vertices`]'s output as a
/// drop-in — bit-exact equality after tolerance dedup is exactly
/// what this routine expects.
pub fn repair_weld_vertices(scene: &mut Scene3D) -> WeldReport {
    let mut report = WeldReport::default();
    for mesh in &mut scene.meshes {
        for prim in &mut mesh.primitives {
            if prim.topology != Topology::Triangles {
                continue;
            }
            weld_primitive(prim, &mut report);
        }
    }
    report
}

fn weld_primitive(prim: &mut Primitive, report: &mut WeldReport) {
    // Walk the effective triangle stream and capture (position, normal)
    // pairs in emit order, alongside a per-vertex bit-exact key.
    let face_count = match &prim.indices {
        Some(idx) => idx.len() / 3,
        None => prim.positions.len() / 3,
    };
    if face_count == 0 {
        return;
    }
    report.triangles_inspected += face_count;
    let positions_before = prim.positions.len();

    let normals_match = prim
        .normals
        .as_ref()
        .map(|ns| ns.len() == prim.positions.len())
        .unwrap_or(false);

    let mut canonical_positions: Vec<[f32; 3]> = Vec::new();
    let mut canonical_normals: Vec<Option<[f32; 3]>> = Vec::new();
    let mut key_to_slot: HashMap<VertKey, u32> = HashMap::new();
    let mut new_indices: Vec<u32> = Vec::with_capacity(face_count * 3);

    let mut emitted_slots = 0usize;
    for face_idx in 0..face_count {
        let (vi0, vi1, vi2) = resolve_face(&prim.indices, face_idx);
        let mut tri_slots: [u32; 3] = [0; 3];
        for (slot_idx, vi) in [vi0, vi1, vi2].iter().enumerate() {
            let p = match prim.positions.get(*vi) {
                Some(p) => *p,
                None => continue,
            };
            emitted_slots += 1;
            let key = VertKey::from(p);
            let slot = if let Some(&existing) = key_to_slot.get(&key) {
                existing
            } else {
                let new_slot = canonical_positions.len() as u32;
                canonical_positions.push(p);
                let n = if normals_match {
                    prim.normals.as_ref().and_then(|ns| ns.get(*vi).copied())
                } else {
                    None
                };
                canonical_normals.push(n);
                key_to_slot.insert(key, new_slot);
                new_slot
            };
            tri_slots[slot_idx] = slot;
        }
        // Degenerate detection — any two of the three slots equal
        // means the welded triangle has zero area.
        if tri_slots[0] == tri_slots[1]
            || tri_slots[1] == tri_slots[2]
            || tri_slots[0] == tri_slots[2]
        {
            report.degenerate_triangles += 1;
        }
        new_indices.extend_from_slice(&tri_slots);
    }

    report.slots_collapsed += emitted_slots.saturating_sub(canonical_positions.len());
    report.positions_collapsed += positions_before.saturating_sub(canonical_positions.len());

    prim.positions = canonical_positions;
    if normals_match {
        // Fill every canonical slot — `None` slots get a zero
        // sentinel ("recompute from winding", which the STL
        // encoder already handles).
        let normals: Vec<[f32; 3]> = canonical_normals
            .into_iter()
            .map(|n| n.unwrap_or([0.0, 0.0, 0.0]))
            .collect();
        prim.normals = Some(normals);
    }
    prim.indices = Some(Indices::U32(new_indices));
}

fn resolve_face(indices: &Option<Indices>, face_idx: usize) -> (usize, usize, usize) {
    match indices {
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
    }
}

/// Internal collected-triangle representation used by [`shells`].
struct CollectedTri {
    locator: FaceLocator,
    keys: [VertKey; 3],
}

fn collect_triangles(scene: &Scene3D) -> Vec<CollectedTri> {
    let mut out = Vec::new();
    for (mesh_idx, mesh) in scene.meshes.iter().enumerate() {
        for (prim_idx, prim) in mesh.primitives.iter().enumerate() {
            if prim.topology != Topology::Triangles {
                continue;
            }
            let face_count = match &prim.indices {
                Some(idx) => idx.len() / 3,
                None => prim.positions.len() / 3,
            };
            for face_idx in 0..face_count {
                let (vi0, vi1, vi2) = resolve_face(&prim.indices, face_idx);
                let v0 = match prim.positions.get(vi0) {
                    Some(p) => *p,
                    None => continue,
                };
                let v1 = match prim.positions.get(vi1) {
                    Some(p) => *p,
                    None => continue,
                };
                let v2 = match prim.positions.get(vi2) {
                    Some(p) => *p,
                    None => continue,
                };
                out.push(CollectedTri {
                    locator: FaceLocator {
                        mesh: mesh_idx,
                        primitive: prim_idx,
                        face: face_idx,
                    },
                    keys: [VertKey::from(v0), VertKey::from(v1), VertKey::from(v2)],
                });
            }
        }
    }
    out
}

/// Bit-exact `f32` triple key. Matches the rest of the crate's
/// well-defined NaN semantics (every NaN bit pattern is a distinct
/// key).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
struct VertKey(u32, u32, u32);

impl From<[f32; 3]> for VertKey {
    fn from(p: [f32; 3]) -> Self {
        VertKey(p[0].to_bits(), p[1].to_bits(), p[2].to_bits())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_mesh3d::{Mesh, Scene3D};

    fn one_triangle(positions: [[f32; 3]; 3]) -> Primitive {
        let mut p = Primitive::new(Topology::Triangles);
        p.positions = positions.to_vec();
        p.normals = Some(vec![[0.0, 0.0, 1.0]; 3]);
        p
    }

    fn scene_with_primitives(prims: Vec<Primitive>) -> Scene3D {
        let mut mesh = Mesh::new(None::<String>);
        for p in prims {
            mesh.primitives.push(p);
        }
        let mut scene = Scene3D::new();
        scene.add_mesh(mesh);
        scene
    }

    fn unit_cube_soup_primitive() -> Primitive {
        // 12-triangle cube laid out as a flat soup (no index buffer).
        // Bit-exact positions on the 8 corners; emit order matches the
        // canonical winding the rest of the crate uses for cubes.
        let mut prim = Primitive::new(Topology::Triangles);
        let c = [
            [0.0_f32, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
            [0.0, 1.0, 1.0],
        ];
        let tris: [[usize; 3]; 12] = [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [2, 3, 7],
            [2, 7, 6],
            [1, 2, 6],
            [1, 6, 5],
            [0, 4, 7],
            [0, 7, 3],
        ];
        for t in &tris {
            prim.positions.push(c[t[0]]);
            prim.positions.push(c[t[1]]);
            prim.positions.push(c[t[2]]);
        }
        prim.normals = Some(vec![[0.0, 0.0, 0.0]; prim.positions.len()]);
        prim
    }

    #[test]
    fn empty_scene_has_zero_shells() {
        let scene = Scene3D::new();
        assert!(shells(&scene).is_empty());
    }

    #[test]
    fn single_triangle_is_one_shell_with_chi_one() {
        let scene = scene_with_primitives(vec![one_triangle([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ])]);
        let s = shells(&scene);
        assert_eq!(s.len(), 1);
        assert_eq!(s[0].faces, 1);
        assert_eq!(s[0].vertices, 3);
        assert_eq!(s[0].edges, 3);
        assert_eq!(s[0].boundary_edges, 3);
        assert_eq!(s[0].non_manifold_edges, 0);
        // V - E + F = 3 - 3 + 1 = 1
        assert_eq!(s[0].euler_characteristic(), 1);
        assert!(!s[0].is_closed_manifold());
        assert!(s[0].genus().is_none());
    }

    #[test]
    fn unit_cube_soup_is_one_shell_chi_two_genus_zero() {
        let scene = scene_with_primitives(vec![unit_cube_soup_primitive()]);
        let s = shells(&scene);
        assert_eq!(s.len(), 1);
        assert_eq!(s[0].faces, 12);
        assert_eq!(s[0].vertices, 8);
        assert_eq!(s[0].edges, 18);
        // V - E + F = 8 - 18 + 12 = 2 (sphere = genus 0).
        assert_eq!(s[0].euler_characteristic(), 2);
        assert_eq!(s[0].boundary_edges, 0);
        assert_eq!(s[0].non_manifold_edges, 0);
        assert!(s[0].is_closed_manifold());
        assert_eq!(s[0].genus(), Some(0));
    }

    #[test]
    fn two_disjoint_triangles_are_two_shells() {
        let scene = scene_with_primitives(vec![
            one_triangle([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
            one_triangle([[10.0, 10.0, 10.0], [11.0, 10.0, 10.0], [10.0, 11.0, 10.0]]),
        ]);
        let s = shells(&scene);
        assert_eq!(s.len(), 2);
        assert_eq!(s[0].faces, 1);
        assert_eq!(s[1].faces, 1);
        // Locators distinguish the two primitives.
        assert_eq!(s[0].face_indices[0].primitive, 0);
        assert_eq!(s[1].face_indices[0].primitive, 1);
    }

    #[test]
    fn corner_touching_triangles_form_one_shell() {
        // Two triangles sharing only the origin vertex — under the
        // "share any vertex" rule they are one shell.
        let scene = scene_with_primitives(vec![
            one_triangle([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
            one_triangle([[0.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]),
        ]);
        let s = shells(&scene);
        assert_eq!(s.len(), 1);
        assert_eq!(s[0].faces, 2);
        // 5 unique vertices (shared origin), 6 unique edges.
        assert_eq!(s[0].vertices, 5);
        assert_eq!(s[0].edges, 6);
    }

    #[test]
    fn weld_collapses_unindexed_cube_soup_to_eight_corners() {
        let prim = unit_cube_soup_primitive();
        let mut scene = scene_with_primitives(vec![prim]);
        let r = repair_weld_vertices(&mut scene);
        assert_eq!(r.triangles_inspected, 12);
        // 36 emitted slots collapse to 8 canonical corners.
        assert_eq!(r.slots_collapsed, 28);
        // No two of any triangle's corners coincide in the cube
        // (each face is a non-degenerate quad-half).
        assert_eq!(r.degenerate_triangles, 0);
        let p = &scene.meshes[0].primitives[0];
        assert_eq!(p.positions.len(), 8);
        match &p.indices {
            Some(Indices::U32(idx)) => assert_eq!(idx.len(), 36),
            _ => panic!("indices must be U32 after weld"),
        }
    }

    #[test]
    fn weld_is_idempotent_on_already_welded_primitive() {
        let prim = unit_cube_soup_primitive();
        let mut scene = scene_with_primitives(vec![prim]);
        let r1 = repair_weld_vertices(&mut scene);
        let r2 = repair_weld_vertices(&mut scene);
        // First pass collapsed 24 positions (24 = 36 − 8 − 4, but
        // unindexed soup has positions.len() == 36 going in and 8
        // out, so positions_collapsed == 28).
        assert_eq!(r1.positions_collapsed, 28);
        // Second pass collapses zero further positions — the
        // canonical "this pass did nothing" signal.
        assert_eq!(r2.positions_collapsed, 0);
        // slots_collapsed remains gross emit-vs-canonical and is
        // non-zero on both passes because the welded scene still
        // emits 36 slots through the index buffer:
        assert_eq!(r2.slots_collapsed, 28);
        // Idempotency for the canonical-vertex count.
        assert_eq!(scene.meshes[0].primitives[0].positions.len(), 8);
        assert_eq!(r1.triangles_inspected, r2.triangles_inspected);
    }

    #[test]
    fn weld_marks_zero_area_triangle_as_degenerate() {
        // A triangle with two vertices at the same position: after
        // welding, two of its three indices coincide.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]];
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_weld_vertices(&mut scene);
        assert_eq!(r.triangles_inspected, 1);
        assert_eq!(r.degenerate_triangles, 1);
    }

    #[test]
    fn weld_skips_non_triangles_primitive() {
        // A Lines primitive must remain untouched.
        let mut prim = Primitive::new(Topology::Lines);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]];
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_weld_vertices(&mut scene);
        assert_eq!(r.triangles_inspected, 0);
        // Indices remain None.
        assert!(scene.meshes[0].primitives[0].indices.is_none());
    }
}

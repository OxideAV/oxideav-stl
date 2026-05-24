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
//! This module owns those utilities. All shell / report builders are
//! pure-functional; the `repair_*` family is the mutating side and
//! every entry point takes `&mut Scene3D` explicitly.
//!
//! ## Repair pipeline order
//!
//! The mutating helpers are independent — call any combination in any
//! order — but the natural pipeline for a freshly-decoded STL scene
//! (triangle soup, per-vertex normals as 3 copies of the spec'd
//! per-face value) is:
//!
//! 1. [`repair_weld_vertices`] — collapse the soup into a shared index
//!    buffer (bit-exact `f32` position equality).
//! 2. [`repair_drop_degenerate_triangles`] — cull post-weld zero-area
//!    triangles (any two corner indices coincident).
//! 3. [`repair_recompute_zero_normals`] — fill in face normals for any
//!    facet whose stored normal is the spec's all-zero "recompute from
//!    winding" sentinel.
//! 4. [`repair_orient_normals_from_winding`] — for any facet whose
//!    stored normal disagrees with the right-hand-rule cross product of
//!    its winding (negative dot product), rewrite the stored normal to
//!    match the winding. The 1989 spec says facet orientation is
//!    "specified redundantly in two ways which must be consistent";
//!    this repair makes winding the authoritative source.
//! 5. [`repair_normalize_unit_normals`] — rescale any non-unit stored
//!    normal to unit length, matching the spec's "unit normal" rule.
//!    Skips the all-zero sentinel (handled by step 3) and below-eps
//!    cross-product degenerates.
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

/// Outcome of a [`repair_drop_degenerate_triangles`] pass.
///
/// Counters are summed across every `Triangles` primitive in the
/// scene. A run that finds nothing to drop produces
/// `dropped_triangles == 0` — the canonical idempotency signal.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DegenerateDropReport {
    /// Total triangle slots inspected (post-index-buffer resolution)
    /// across every touched primitive.
    pub triangles_inspected: usize,
    /// Number of degenerate triangles removed from the scene. Equals
    /// zero on an already-clean scene.
    pub dropped_triangles: usize,
}

/// Remove zero-area triangles in-place from every `Triangles`
/// primitive.
///
/// A triangle is considered degenerate when any two of its three
/// corner *positions* coincide by bit-exact `f32` match — the same
/// equality model the rest of the crate uses. Run
/// [`repair_weld_vertices`] first if your producer emitted the same
/// logical vertex with multiple slot indices and you want index-based
/// duplicate detection instead of position-based.
///
/// For each touched primitive:
/// - If `prim.indices` is `Some`, the index buffer is rewritten with
///   the surviving triangle slots and its `Indices` discriminant is
///   preserved (`U16` stays `U16`, `U32` stays `U32`).
/// - If `prim.indices` is `None`, the unindexed `positions` (and
///   `normals` when present and matched 1:1 in length) are compacted
///   in place — each surviving triangle's 3 corners (and matching
///   normals) are copied forward, dropped triangles disappear.
///
/// Non-`Triangles` primitives are left untouched. The pass does NOT
/// alter `prim.extras`, `mesh.name`, or the scene-graph structure.
///
/// Returns a single [`DegenerateDropReport`] summed across every
/// touched primitive.
///
/// ## Why position-equality, not zero-cross-product?
///
/// The two definitions agree on the common case (a "duplicated
/// corner" producer bug) but disagree on the "three collinear but
/// distinct corners" pathology. The collinear case is genuinely
/// zero-area and can show up after numeric scaling, but dropping it
/// blind would alter the visible silhouette of CAD-pipeline meshes
/// that intentionally include hairline strips between thicker
/// regions. The strict bit-exact rule lets callers run this repair
/// without that risk; callers who specifically want zero-cross
/// culling can pre-filter via a manual walk of [`crate::validate`]'s
/// orientation report.
pub fn repair_drop_degenerate_triangles(scene: &mut Scene3D) -> DegenerateDropReport {
    let mut report = DegenerateDropReport::default();
    for mesh in &mut scene.meshes {
        for prim in &mut mesh.primitives {
            if prim.topology != Topology::Triangles {
                continue;
            }
            drop_degenerate_in_primitive(prim, &mut report);
        }
    }
    report
}

fn drop_degenerate_in_primitive(prim: &mut Primitive, report: &mut DegenerateDropReport) {
    let face_count = match &prim.indices {
        Some(idx) => idx.len() / 3,
        None => prim.positions.len() / 3,
    };
    if face_count == 0 {
        return;
    }
    report.triangles_inspected += face_count;

    // Decide per-face survival on resolved (position-key) corners so
    // both indexed and unindexed primitives are judged consistently.
    let mut keep = Vec::with_capacity(face_count);
    let mut dropped_local = 0usize;
    for face_idx in 0..face_count {
        let (vi0, vi1, vi2) = resolve_face(&prim.indices, face_idx);
        let p0 = prim.positions.get(vi0).copied();
        let p1 = prim.positions.get(vi1).copied();
        let p2 = prim.positions.get(vi2).copied();
        let (a, b, c) = match (p0, p1, p2) {
            (Some(a), Some(b), Some(c)) => (a, b, c),
            // A face whose index is out of range is silently dropped
            // — it would crash any downstream consumer anyway, and
            // counting it as "kept" would mis-balance the report.
            _ => {
                dropped_local += 1;
                keep.push(false);
                continue;
            }
        };
        let degen = vert_key_eq(a, b) || vert_key_eq(b, c) || vert_key_eq(a, c);
        if degen {
            dropped_local += 1;
            keep.push(false);
        } else {
            keep.push(true);
        }
    }
    if dropped_local == 0 {
        return;
    }
    report.dropped_triangles += dropped_local;

    // Rewrite the buffers.
    let normals_match = prim
        .normals
        .as_ref()
        .map(|ns| ns.len() == prim.positions.len())
        .unwrap_or(false);
    match prim.indices.take() {
        Some(Indices::U16(idx)) => {
            let mut new_idx = Vec::with_capacity(face_count * 3 - dropped_local * 3);
            for (face_idx, keep_face) in keep.iter().enumerate() {
                if !*keep_face {
                    continue;
                }
                let b = face_idx * 3;
                new_idx.extend_from_slice(&idx[b..b + 3]);
            }
            prim.indices = Some(Indices::U16(new_idx));
        }
        Some(Indices::U32(idx)) => {
            let mut new_idx = Vec::with_capacity(face_count * 3 - dropped_local * 3);
            for (face_idx, keep_face) in keep.iter().enumerate() {
                if !*keep_face {
                    continue;
                }
                let b = face_idx * 3;
                new_idx.extend_from_slice(&idx[b..b + 3]);
            }
            prim.indices = Some(Indices::U32(new_idx));
        }
        None => {
            // Unindexed: compact positions + (when matched 1:1) normals.
            let mut new_pos: Vec<[f32; 3]> = Vec::with_capacity(prim.positions.len());
            let mut new_norms: Vec<[f32; 3]> = if normals_match {
                Vec::with_capacity(prim.positions.len())
            } else {
                Vec::new()
            };
            for (face_idx, keep_face) in keep.iter().enumerate() {
                if !*keep_face {
                    continue;
                }
                let b = face_idx * 3;
                new_pos.push(prim.positions[b]);
                new_pos.push(prim.positions[b + 1]);
                new_pos.push(prim.positions[b + 2]);
                if normals_match {
                    if let Some(ns) = prim.normals.as_ref() {
                        new_norms.push(ns[b]);
                        new_norms.push(ns[b + 1]);
                        new_norms.push(ns[b + 2]);
                    }
                }
            }
            prim.positions = new_pos;
            if normals_match {
                prim.normals = Some(new_norms);
            }
        }
    }
}

fn vert_key_eq(a: [f32; 3], b: [f32; 3]) -> bool {
    a[0].to_bits() == b[0].to_bits()
        && a[1].to_bits() == b[1].to_bits()
        && a[2].to_bits() == b[2].to_bits()
}

/// Outcome of a [`repair_recompute_zero_normals`] pass.
///
/// Counters are summed across every `Triangles` primitive in the
/// scene. `recomputed_triangles == 0` is the idempotency signal.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NormalRecomputeReport {
    /// Total triangle slots inspected (post-index-buffer resolution)
    /// across every touched primitive.
    pub triangles_inspected: usize,
    /// Number of triangles whose per-vertex normals were rewritten
    /// from the right-hand-rule cross product of their positions.
    pub recomputed_triangles: usize,
    /// Number of triangles left alone because the recomputed normal
    /// was below the cross-product epsilon (a true degenerate
    /// triangle — see [`repair_drop_degenerate_triangles`] to remove
    /// them).
    pub skipped_degenerate: usize,
    /// Number of primitives where a missing `normals` array was
    /// freshly created and populated. Primitives whose `normals` is
    /// already `Some(_)` with a length mismatch are left untouched
    /// and reported under [`Self::skipped_length_mismatch`].
    pub primitives_populated: usize,
    /// Number of primitives skipped because their existing `normals`
    /// length did not match `positions.len()`. Callers that hit this
    /// likely have a producer bug upstream; the safe action is to
    /// not silently rewrite the buffer.
    pub skipped_length_mismatch: usize,
}

/// Fill in zero-stored ("recompute from winding") triangle normals
/// from the right-hand-rule cross product of their positions.
///
/// Per the STL spec (§6.5 of the Marshall Burns transcription), each
/// facet's normal must obey the right-hand rule against the vertex
/// order, but an all-zero stored normal is the spec's documented
/// sentinel for "the consumer should recompute". Producers
/// occasionally emit zero normals across the board to mark
/// "unverified orientation"; this repair walks the scene and
/// rewrites those triangles in-place.
///
/// Per-triangle decision: only triangles whose three *current*
/// per-vertex normals all have a magnitude `≤ eps` are rewritten.
/// Triangles where some corners carry non-zero normals and others do
/// not — a tell that the producer mixed face-normal and vertex-normal
/// data — are left untouched.
///
/// If a primitive's `normals` field is `None`, this routine creates
/// it sized to match `positions.len()` and populates per-face triples
/// for every non-degenerate face. Primitives whose existing `normals`
/// length disagrees with `positions.len()` are skipped and counted
/// under [`NormalRecomputeReport::skipped_length_mismatch`].
///
/// `eps == 0.0` (or any negative / non-finite value, clamped to
/// `0.0`) means "exact zero only" — the strict spec sentinel rule.
/// A small positive value (e.g. `1e-6`) catches producers that emit
/// floating-point-noise-zero normals. The cross-product magnitude is
/// also compared against `eps` to skip mathematically-degenerate
/// triangles (counted under
/// [`NormalRecomputeReport::skipped_degenerate`]).
///
/// Non-`Triangles` primitives are left untouched. The pass does NOT
/// alter `prim.extras`, `mesh.name`, the scene-graph structure, or
/// any non-normal vertex attribute.
pub fn repair_recompute_zero_normals(scene: &mut Scene3D, eps: f32) -> NormalRecomputeReport {
    let mut report = NormalRecomputeReport::default();
    let eps = if eps.is_finite() && eps > 0.0 {
        eps
    } else {
        0.0
    };
    for mesh in &mut scene.meshes {
        for prim in &mut mesh.primitives {
            if prim.topology != Topology::Triangles {
                continue;
            }
            recompute_normals_in_primitive(prim, eps, &mut report);
        }
    }
    report
}

fn recompute_normals_in_primitive(
    prim: &mut Primitive,
    eps: f32,
    report: &mut NormalRecomputeReport,
) {
    let face_count = match &prim.indices {
        Some(idx) => idx.len() / 3,
        None => prim.positions.len() / 3,
    };
    if face_count == 0 {
        return;
    }
    report.triangles_inspected += face_count;

    // Ensure normals has the right shape, or skip.
    let positions_len = prim.positions.len();
    let needs_populate = match &prim.normals {
        None => true,
        Some(ns) if ns.len() != positions_len => {
            report.skipped_length_mismatch += 1;
            return;
        }
        Some(_) => false,
    };
    if needs_populate {
        prim.normals = Some(vec![[0.0, 0.0, 0.0]; positions_len]);
        report.primitives_populated += 1;
    }

    // Reborrow once we know the field is Some.
    let normals = prim
        .normals
        .as_mut()
        .expect("normals just populated or already Some");

    for face_idx in 0..face_count {
        let (vi0, vi1, vi2) = resolve_face(&prim.indices, face_idx);
        // All three index lookups must succeed. Out-of-range -> skip.
        let p0 = prim.positions.get(vi0).copied();
        let p1 = prim.positions.get(vi1).copied();
        let p2 = prim.positions.get(vi2).copied();
        let (a, b, c) = match (p0, p1, p2) {
            (Some(a), Some(b), Some(c)) => (a, b, c),
            _ => continue,
        };
        // Read the current per-vertex normals for this face's three
        // slots. Triangles with any non-zero stored normal are left
        // alone — only "all zero" triples are rewritten.
        let n0 = normals.get(vi0).copied().unwrap_or([0.0; 3]);
        let n1 = normals.get(vi1).copied().unwrap_or([0.0; 3]);
        let n2 = normals.get(vi2).copied().unwrap_or([0.0; 3]);
        if !(normal_is_zero(n0, eps) && normal_is_zero(n1, eps) && normal_is_zero(n2, eps)) {
            continue;
        }
        let recomputed = recompute_face_normal(a, b, c, eps);
        match recomputed {
            Some(n) => {
                // Per-face normal duplicated across the three vertex
                // slots, matching the rest of the crate's convention.
                if let Some(slot) = normals.get_mut(vi0) {
                    *slot = n;
                }
                if let Some(slot) = normals.get_mut(vi1) {
                    *slot = n;
                }
                if let Some(slot) = normals.get_mut(vi2) {
                    *slot = n;
                }
                report.recomputed_triangles += 1;
            }
            None => {
                report.skipped_degenerate += 1;
            }
        }
    }
}

fn normal_is_zero(n: [f32; 3], eps: f32) -> bool {
    // Component-wise absolute check against `eps`. The encoder + the
    // validate module treat an all-zero triple as the spec sentinel,
    // and that's what we recover here; a small positive `eps` widens
    // it to catch float-noise zeros.
    n[0].abs() <= eps && n[1].abs() <= eps && n[2].abs() <= eps
}

fn recompute_face_normal(a: [f32; 3], b: [f32; 3], c: [f32; 3], eps: f32) -> Option<[f32; 3]> {
    let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let cx = u[1] * v[2] - u[2] * v[1];
    let cy = u[2] * v[0] - u[0] * v[2];
    let cz = u[0] * v[1] - u[1] * v[0];
    let len = (cx * cx + cy * cy + cz * cz).sqrt();
    // For `eps == 0.0`, the only rejection is a numerically-exact
    // zero cross product (collinear or coincident corners). For
    // positive `eps`, any cross-product magnitude at or below `eps`
    // counts as "degenerate" — same threshold semantics the
    // zero-normal detection uses.
    let threshold = eps.max(f32::EPSILON);
    if len.is_finite() && len > threshold {
        Some([cx / len, cy / len, cz / len])
    } else {
        None
    }
}

/// Outcome of a [`repair_orient_normals_from_winding`] pass.
///
/// Counters are summed across every `Triangles` primitive in the
/// scene. `flipped_normals == 0` is the idempotency signal — a
/// scene whose every facet already has its stored normal aligned
/// with its winding is left untouched.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct OrientReport {
    /// Total triangle slots inspected (post-index-buffer resolution)
    /// across every touched primitive.
    pub triangles_inspected: usize,
    /// Number of triangles whose stored normal was flipped to align
    /// with the right-hand-rule cross product of their positions
    /// (dot product strictly < 0 between stored and recomputed).
    pub flipped_normals: usize,
    /// Number of triangles left alone because the stored normal was
    /// the all-zero spec sentinel — those are
    /// [`repair_recompute_zero_normals`]'s job, not this one's.
    pub skipped_zero_normal: usize,
    /// Number of triangles left alone because the cross product
    /// magnitude was at or below `eps` (collinear / coincident
    /// corners). Use [`repair_drop_degenerate_triangles`] to remove
    /// them.
    pub skipped_degenerate: usize,
    /// Number of primitives whose `normals` field length disagreed
    /// with `positions.len()`. Skipped without modification.
    pub skipped_length_mismatch: usize,
    /// Number of primitives skipped because their `normals` field
    /// was `None`. Run [`repair_recompute_zero_normals`] first to
    /// populate; this pass only reorients existing normals.
    pub skipped_missing_normals: usize,
}

/// Reorient every stored facet normal to agree with its winding.
///
/// The 1989 spec says facet orientation is "specified redundantly in
/// two ways which must be consistent": (1) the direction of the
/// normal is outward; (2) the vertices are listed in counter-clockwise
/// order when viewed from outside (right-hand rule). When a producer
/// emits a stored normal whose direction *disagrees* with the
/// right-hand-rule cross product of its winding (dot product < 0),
/// this repair rewrites the stored normal to the cross-product
/// direction — i.e. winding wins.
///
/// Per-triangle decision:
/// - Compute the right-hand-rule cross product `(v1 − v0) × (v2 − v0)`.
/// - If the cross-product magnitude is `≤ eps`, the triangle is
///   geometrically degenerate; count under
///   [`OrientReport::skipped_degenerate`] and move on (see
///   [`repair_drop_degenerate_triangles`]).
/// - If any of the three stored per-vertex normals are the all-zero
///   spec sentinel (component magnitudes all `≤ eps`), this pass
///   does NOT rewrite them — that's [`repair_recompute_zero_normals`]'s
///   job. Count under [`OrientReport::skipped_zero_normal`].
/// - Otherwise compute `dot(stored, recomputed)` against the first
///   stored normal. If strictly negative, rewrite all three slots to
///   the (unit-normalised) cross product — same per-face-normal
///   duplicated-across-3-slots convention the rest of the crate uses.
///   If non-negative, leave the stored values alone (a tiny shrunk
///   or overlong normal in the right direction is the
///   [`repair_normalize_unit_normals`] pass's concern, not this one's).
///
/// Primitives whose `normals` field is `None` are skipped (use
/// [`repair_recompute_zero_normals`] to populate first). Primitives
/// whose `normals` length disagrees with `positions.len()` are
/// reported under [`OrientReport::skipped_length_mismatch`].
///
/// `eps == 0.0` (or any negative / non-finite value, clamped to
/// `0.0`) means "exact zero only" for the cross-product degeneracy
/// gate; a small positive value catches near-degenerates.
///
/// Non-`Triangles` primitives are left untouched. The pass does NOT
/// alter `prim.extras`, `mesh.name`, the scene-graph structure, or
/// any non-normal vertex attribute.
pub fn repair_orient_normals_from_winding(scene: &mut Scene3D, eps: f32) -> OrientReport {
    let mut report = OrientReport::default();
    let eps = if eps.is_finite() && eps > 0.0 {
        eps
    } else {
        0.0
    };
    for mesh in &mut scene.meshes {
        for prim in &mut mesh.primitives {
            if prim.topology != Topology::Triangles {
                continue;
            }
            orient_normals_in_primitive(prim, eps, &mut report);
        }
    }
    report
}

fn orient_normals_in_primitive(prim: &mut Primitive, eps: f32, report: &mut OrientReport) {
    let face_count = match &prim.indices {
        Some(idx) => idx.len() / 3,
        None => prim.positions.len() / 3,
    };
    if face_count == 0 {
        return;
    }
    report.triangles_inspected += face_count;

    let positions_len = prim.positions.len();
    let normals = match prim.normals.as_mut() {
        None => {
            report.skipped_missing_normals += 1;
            return;
        }
        Some(ns) if ns.len() != positions_len => {
            report.skipped_length_mismatch += 1;
            return;
        }
        Some(ns) => ns,
    };

    for face_idx in 0..face_count {
        let (vi0, vi1, vi2) = resolve_face(&prim.indices, face_idx);
        let p0 = prim.positions.get(vi0).copied();
        let p1 = prim.positions.get(vi1).copied();
        let p2 = prim.positions.get(vi2).copied();
        let (a, b, c) = match (p0, p1, p2) {
            (Some(a), Some(b), Some(c)) => (a, b, c),
            _ => continue,
        };
        let stored = match normals.get(vi0).copied() {
            Some(n) => n,
            None => continue,
        };
        // All-zero spec sentinel — defer to repair_recompute_zero_normals.
        if normal_is_zero(stored, eps) {
            report.skipped_zero_normal += 1;
            continue;
        }
        let recomputed = match recompute_face_normal(a, b, c, eps) {
            Some(n) => n,
            None => {
                report.skipped_degenerate += 1;
                continue;
            }
        };
        let dot = stored[0] * recomputed[0] + stored[1] * recomputed[1] + stored[2] * recomputed[2];
        if dot < 0.0 {
            // Flip — rewrite all three slots to the unit-normalised
            // cross product (same convention used by recompute).
            if let Some(slot) = normals.get_mut(vi0) {
                *slot = recomputed;
            }
            if let Some(slot) = normals.get_mut(vi1) {
                *slot = recomputed;
            }
            if let Some(slot) = normals.get_mut(vi2) {
                *slot = recomputed;
            }
            report.flipped_normals += 1;
        }
    }
}

/// Outcome of a [`repair_normalize_unit_normals`] pass.
///
/// Counters are summed across every `Triangles` primitive in the
/// scene. `rescaled_normals == 0` is the idempotency signal.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NormalizeReport {
    /// Total triangle slots inspected (post-index-buffer resolution)
    /// across every touched primitive.
    pub triangles_inspected: usize,
    /// Number of triangles whose three per-vertex normal slots were
    /// rescaled to unit length (deviation from `1.0` exceeded
    /// `unit_tolerance`).
    pub rescaled_normals: usize,
    /// Number of triangles left alone because the stored normal was
    /// the all-zero spec sentinel — those are
    /// [`repair_recompute_zero_normals`]'s job, not this one's.
    pub skipped_zero_normal: usize,
    /// Number of primitives whose `normals` field length disagreed
    /// with `positions.len()`. Skipped without modification.
    pub skipped_length_mismatch: usize,
    /// Number of primitives skipped because their `normals` field
    /// was `None` — there are no stored normals to rescale.
    pub skipped_missing_normals: usize,
}

/// Rescale every non-zero stored facet normal to unit length.
///
/// Per the 1989 spec, each facet's normal is a *unit* vector. The
/// validate module surfaces non-unit normals via
/// `ValidationReport::non_unit_normal_defects` with the same
/// tolerance constant; this repair is the mutating fix-up.
///
/// Per-triangle decision (only the first slot of each face is
/// inspected; the per-face / 3-copy convention the rest of the crate
/// uses means slots 1 and 2 carry the same value):
/// - Read the stored normal.
/// - If it is the all-zero spec sentinel (all three components have
///   absolute magnitude `≤ unit_tolerance`), leave it alone — that's
///   [`repair_recompute_zero_normals`]'s job. Counted under
///   [`NormalizeReport::skipped_zero_normal`].
/// - Otherwise compute `len = sqrt(x² + y² + z²)`. If
///   `|len − 1.0| > unit_tolerance` (and `len` is finite and
///   strictly positive), divide every slot's components by `len` so
///   the resulting vector has unit length. Counted under
///   [`NormalizeReport::rescaled_normals`].
/// - If `|len − 1.0| ≤ unit_tolerance`, the normal is already
///   unit-length within tolerance; do not touch.
///
/// Primitives whose `normals` field is `None` are skipped (use
/// [`repair_recompute_zero_normals`] to populate first). Primitives
/// whose `normals` length disagrees with `positions.len()` are
/// reported under [`NormalizeReport::skipped_length_mismatch`].
///
/// `unit_tolerance` defaults match the validate module's
/// `DEFAULT_UNIT_NORMAL_TOLERANCE` (`1e-3`); negative / non-finite
/// values are clamped to `1e-3` (the validate-module default). Pass
/// `0.0` for strict bit-exact unit-length detection.
///
/// Non-`Triangles` primitives are left untouched. The pass does NOT
/// alter `prim.extras`, `mesh.name`, the scene-graph structure, or
/// any non-normal vertex attribute.
pub fn repair_normalize_unit_normals(scene: &mut Scene3D, unit_tolerance: f32) -> NormalizeReport {
    let mut report = NormalizeReport::default();
    // Match the validate module's default; clamp non-finite / negative
    // values to that default rather than panicking.
    let unit_tolerance = if unit_tolerance.is_finite() && unit_tolerance >= 0.0 {
        unit_tolerance
    } else {
        crate::validate::DEFAULT_UNIT_NORMAL_TOLERANCE
    };
    for mesh in &mut scene.meshes {
        for prim in &mut mesh.primitives {
            if prim.topology != Topology::Triangles {
                continue;
            }
            normalize_normals_in_primitive(prim, unit_tolerance, &mut report);
        }
    }
    report
}

fn normalize_normals_in_primitive(
    prim: &mut Primitive,
    unit_tolerance: f32,
    report: &mut NormalizeReport,
) {
    let face_count = match &prim.indices {
        Some(idx) => idx.len() / 3,
        None => prim.positions.len() / 3,
    };
    if face_count == 0 {
        return;
    }
    report.triangles_inspected += face_count;

    let positions_len = prim.positions.len();
    let normals = match prim.normals.as_mut() {
        None => {
            report.skipped_missing_normals += 1;
            return;
        }
        Some(ns) if ns.len() != positions_len => {
            report.skipped_length_mismatch += 1;
            return;
        }
        Some(ns) => ns,
    };

    for face_idx in 0..face_count {
        let (vi0, vi1, vi2) = resolve_face(&prim.indices, face_idx);
        let stored = match normals.get(vi0).copied() {
            Some(n) => n,
            None => continue,
        };
        // All-zero spec sentinel — defer to repair_recompute_zero_normals.
        if normal_is_zero(stored, unit_tolerance) {
            report.skipped_zero_normal += 1;
            continue;
        }
        let len2 = stored[0] * stored[0] + stored[1] * stored[1] + stored[2] * stored[2];
        let len = len2.sqrt();
        if !len.is_finite() || len <= 0.0 {
            // Pathological — caller passed NaN / Inf coordinates.
            // Leave alone; the zero-sentinel path above already
            // handled the genuinely-zero case.
            continue;
        }
        if (len - 1.0).abs() <= unit_tolerance {
            continue;
        }
        let inv = 1.0 / len;
        let rescaled = [stored[0] * inv, stored[1] * inv, stored[2] * inv];
        if let Some(slot) = normals.get_mut(vi0) {
            *slot = rescaled;
        }
        if let Some(slot) = normals.get_mut(vi1) {
            *slot = rescaled;
        }
        if let Some(slot) = normals.get_mut(vi2) {
            *slot = rescaled;
        }
        report.rescaled_normals += 1;
    }
}

/// Outcome of a [`repair_sort_triangles_by_z`] pass.
///
/// Counters are summed across every `Triangles` primitive in the
/// scene. `triangles_reordered == 0` is the idempotency signal — a
/// second pass over an already-sorted scene leaves every triangle in
/// place and reports zero.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SortByZReport {
    /// Total triangle slots inspected (post-index-buffer resolution)
    /// across every touched primitive.
    pub triangles_inspected: usize,
    /// Number of triangles whose position in the emit order changed as
    /// a result of the sort. Equals zero on an already-sorted scene.
    pub triangles_reordered: usize,
}

/// Reorder every `Triangles` primitive's triangles into ascending
/// z-value order, in place.
///
/// The 1989 spec notes: *"Sorting the triangles in ascending z-value
/// order is recommended, but not required, in order to optimize
/// performance of the slice program."* A slicer sweeps a cutting plane
/// from the bottom of the object upward; presenting facets in the
/// order their lowest corner enters the sweep lets the slicer stream
/// triangles instead of re-scanning the whole soup at each layer. This
/// repair materialises that recommendation.
///
/// ## Sort key
///
/// Each triangle is keyed on its three corner z-values sorted
/// ascending: `(min_z, mid_z, max_z)`. The primary key is the lowest
/// corner (when the slice plane first reaches the facet); `mid_z` then
/// `max_z` are deterministic tie-breakers so facets sharing a floor
/// still get a total, stable order. Comparison uses
/// [`f32::total_cmp`], giving a total order over all `f32` values
/// (including signed zero and NaN — NaN sorts to the high end, so a
/// facet with a non-finite z-coordinate lands last rather than
/// scrambling the finite facets around it).
///
/// The sort is **stable**: triangles whose keys compare equal keep
/// their original relative emit order, so re-running the pass is
/// idempotent (`triangles_reordered == 0` on the second call).
///
/// ## Per-primitive behaviour
///
/// - Indexed primitives (`prim.indices` is `Some`) have their index
///   buffer rewritten in the sorted face order; the `Indices`
///   discriminant is preserved (`U16` stays `U16`, `U32` stays `U32`)
///   and the shared `positions` / `normals` arrays are untouched.
/// - Unindexed primitives have their `positions` (and `normals`, when
///   present and matched 1:1 in length) re-laid-out three corners at a
///   time in the sorted face order. When `normals` is absent or its
///   length disagrees with `positions`, only `positions` is reordered
///   (the mismatch is left to [`repair_recompute_zero_normals`] /
///   [`repair_normalize_unit_normals`] to surface).
///
/// Non-`Triangles` primitives are left untouched. The pass does NOT
/// alter `prim.extras`, `mesh.name`, or the scene-graph structure, and
/// it never adds or removes a triangle — it is a pure reordering.
///
/// A face whose index buffer references an out-of-range position is
/// kept (sorting never drops geometry — that is
/// [`repair_drop_degenerate_triangles`]' job) and sorted to the end
/// via the NaN-high sentinel, so a malformed facet does not perturb
/// the order of the well-formed ones.
///
/// Returns a single [`SortByZReport`] summed across every touched
/// primitive.
pub fn repair_sort_triangles_by_z(scene: &mut Scene3D) -> SortByZReport {
    let mut report = SortByZReport::default();
    for mesh in &mut scene.meshes {
        for prim in &mut mesh.primitives {
            if prim.topology != Topology::Triangles {
                continue;
            }
            sort_by_z_in_primitive(prim, &mut report);
        }
    }
    report
}

/// Sorted-ascending z triple for one triangle. `f32::NAN` is used as
/// the high sentinel for a corner the index buffer could not resolve,
/// so malformed faces sort last under [`f32::total_cmp`].
fn triangle_z_key(prim: &Primitive, face_idx: usize) -> [f32; 3] {
    let (vi0, vi1, vi2) = resolve_face(&prim.indices, face_idx);
    let z = |vi: usize| prim.positions.get(vi).map(|p| p[2]).unwrap_or(f32::NAN);
    let mut zs = [z(vi0), z(vi1), z(vi2)];
    zs.sort_by(f32::total_cmp);
    zs
}

fn sort_by_z_in_primitive(prim: &mut Primitive, report: &mut SortByZReport) {
    let face_count = match &prim.indices {
        Some(idx) => idx.len() / 3,
        None => prim.positions.len() / 3,
    };
    if face_count == 0 {
        return;
    }
    report.triangles_inspected += face_count;

    // Build (original_face_idx, key) and stably sort by key. A stable
    // sort means equal-key faces keep their relative order, which is
    // what makes a second pass idempotent.
    let mut order: Vec<usize> = (0..face_count).collect();
    let keys: Vec<[f32; 3]> = (0..face_count).map(|f| triangle_z_key(prim, f)).collect();
    order.sort_by(|&a, &b| {
        let ka = keys[a];
        let kb = keys[b];
        ka[0]
            .total_cmp(&kb[0])
            .then_with(|| ka[1].total_cmp(&kb[1]))
            .then_with(|| ka[2].total_cmp(&kb[2]))
    });

    // Count how many faces actually moved (identity permutation ⇒ 0).
    let reordered = order.iter().enumerate().filter(|(i, &o)| *i != o).count();
    if reordered == 0 {
        return;
    }
    report.triangles_reordered += reordered;

    // Apply the permutation to the relevant buffers.
    match prim.indices.take() {
        Some(Indices::U16(idx)) => {
            let mut new_idx = Vec::with_capacity(idx.len());
            for &face_idx in &order {
                let b = face_idx * 3;
                new_idx.extend_from_slice(&idx[b..b + 3]);
            }
            prim.indices = Some(Indices::U16(new_idx));
        }
        Some(Indices::U32(idx)) => {
            let mut new_idx = Vec::with_capacity(idx.len());
            for &face_idx in &order {
                let b = face_idx * 3;
                new_idx.extend_from_slice(&idx[b..b + 3]);
            }
            prim.indices = Some(Indices::U32(new_idx));
        }
        None => {
            let normals_match = prim
                .normals
                .as_ref()
                .map(|ns| ns.len() == prim.positions.len())
                .unwrap_or(false);
            let mut new_pos: Vec<[f32; 3]> = Vec::with_capacity(prim.positions.len());
            let mut new_norms: Vec<[f32; 3]> = if normals_match {
                Vec::with_capacity(prim.positions.len())
            } else {
                Vec::new()
            };
            for &face_idx in &order {
                let b = face_idx * 3;
                new_pos.push(prim.positions[b]);
                new_pos.push(prim.positions[b + 1]);
                new_pos.push(prim.positions[b + 2]);
                if normals_match {
                    if let Some(ns) = prim.normals.as_ref() {
                        new_norms.push(ns[b]);
                        new_norms.push(ns[b + 1]);
                        new_norms.push(ns[b + 2]);
                    }
                }
            }
            prim.positions = new_pos;
            if normals_match {
                prim.normals = Some(new_norms);
            }
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

    // ----- repair_drop_degenerate_triangles -----

    #[test]
    fn drop_degenerate_unindexed_compacts_in_place() {
        // Two unindexed triangles: one healthy, one with two
        // coincident corners. The pass keeps the healthy one and
        // drops the degenerate one.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![
            // Healthy triangle.
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            // Degenerate: 1st and 2nd corner identical.
            [2.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 1.0, 0.0],
        ];
        prim.normals = Some(vec![
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
        ]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_drop_degenerate_triangles(&mut scene);
        assert_eq!(r.triangles_inspected, 2);
        assert_eq!(r.dropped_triangles, 1);
        let p = &scene.meshes[0].primitives[0];
        assert_eq!(p.positions.len(), 3);
        assert_eq!(p.positions[0], [0.0, 0.0, 0.0]);
        assert_eq!(p.positions[1], [1.0, 0.0, 0.0]);
        assert_eq!(p.positions[2], [0.0, 1.0, 0.0]);
        let ns = p.normals.as_ref().expect("normals preserved");
        assert_eq!(ns.len(), 3);
        assert_eq!(ns[0], [0.0, 0.0, 1.0]);
    }

    #[test]
    fn drop_degenerate_indexed_u32_rewrites_index_buffer() {
        // Three faces in a shared-vertex index buffer; face 1 has
        // two identical corner indices (canonical post-weld degenerate).
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
        ];
        prim.indices = Some(Indices::U32(vec![
            0, 1, 2, // healthy
            0, 0, 3, // degenerate (idx 0 twice)
            1, 2, 3, // healthy
        ]));
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_drop_degenerate_triangles(&mut scene);
        assert_eq!(r.triangles_inspected, 3);
        assert_eq!(r.dropped_triangles, 1);
        let p = &scene.meshes[0].primitives[0];
        // Positions are untouched on an indexed primitive.
        assert_eq!(p.positions.len(), 4);
        match &p.indices {
            Some(Indices::U32(idx)) => assert_eq!(idx, &vec![0, 1, 2, 1, 2, 3]),
            _ => panic!("U32 discriminant must be preserved"),
        }
    }

    #[test]
    fn drop_degenerate_indexed_u16_preserves_discriminant() {
        // Same test as U32 path but the input index buffer is U16.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
        ];
        prim.indices = Some(Indices::U16(vec![0, 1, 2, 0, 0, 3, 1, 2, 3]));
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_drop_degenerate_triangles(&mut scene);
        assert_eq!(r.dropped_triangles, 1);
        match &scene.meshes[0].primitives[0].indices {
            Some(Indices::U16(idx)) => assert_eq!(idx, &vec![0u16, 1, 2, 1, 2, 3]),
            _ => panic!("U16 discriminant must be preserved"),
        }
    }

    #[test]
    fn drop_degenerate_idempotent_on_clean_scene() {
        let mut scene = scene_with_primitives(vec![unit_cube_soup_primitive()]);
        let r1 = repair_drop_degenerate_triangles(&mut scene);
        let r2 = repair_drop_degenerate_triangles(&mut scene);
        assert_eq!(r1.triangles_inspected, 12);
        assert_eq!(r1.dropped_triangles, 0);
        assert_eq!(r2.dropped_triangles, 0);
    }

    #[test]
    fn drop_degenerate_skips_non_triangles() {
        let mut prim = Primitive::new(Topology::Lines);
        prim.positions = vec![[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]];
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_drop_degenerate_triangles(&mut scene);
        assert_eq!(r.triangles_inspected, 0);
        assert_eq!(r.dropped_triangles, 0);
        // Positions untouched.
        assert_eq!(scene.meshes[0].primitives[0].positions.len(), 2);
    }

    #[test]
    fn drop_degenerate_composes_with_weld() {
        // Producer emits the canonical "two coincident corners
        // hidden as three distinct slot indices" bug. Weld collapses
        // the duplicates to one slot, then drop culls the resulting
        // zero-area face.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0], // bit-exact duplicate of slot 0
            [1.0, 0.0, 0.0],
        ];
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let weld = repair_weld_vertices(&mut scene);
        assert_eq!(weld.degenerate_triangles, 1);
        let drop = repair_drop_degenerate_triangles(&mut scene);
        assert_eq!(drop.dropped_triangles, 1);
        // Index buffer now empty (the one face was the degenerate
        // one); positions array still carries the two unique slots.
        let p = &scene.meshes[0].primitives[0];
        match &p.indices {
            Some(Indices::U32(idx)) => assert!(idx.is_empty()),
            _ => panic!("weld left a U32 index buffer"),
        }
    }

    // ----- repair_recompute_zero_normals -----

    #[test]
    fn recompute_zero_normals_fills_in_face_normal_for_zero_triple() {
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        prim.normals = Some(vec![[0.0; 3]; 3]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_recompute_zero_normals(&mut scene, 0.0);
        assert_eq!(r.triangles_inspected, 1);
        assert_eq!(r.recomputed_triangles, 1);
        assert_eq!(r.skipped_degenerate, 0);
        let ns = scene.meshes[0].primitives[0].normals.as_ref().unwrap();
        // RHR cross of (1,0,0)×(0,1,0) = (0,0,1).
        for n in ns {
            assert!((n[2] - 1.0).abs() < 1e-6, "expected +Z, got {:?}", n);
            assert!(n[0].abs() < 1e-6);
            assert!(n[1].abs() < 1e-6);
        }
    }

    #[test]
    fn recompute_zero_normals_populates_missing_normals_field() {
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        prim.normals = None;
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_recompute_zero_normals(&mut scene, 0.0);
        assert_eq!(r.primitives_populated, 1);
        assert_eq!(r.recomputed_triangles, 1);
        let ns = scene.meshes[0].primitives[0].normals.as_ref().unwrap();
        assert_eq!(ns.len(), 3);
    }

    #[test]
    fn recompute_zero_normals_skips_triangle_with_partial_nonzero() {
        // Three vertex slots, first carries a non-zero normal; the
        // other two are zero. Mixed = leave alone.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        prim.normals = Some(vec![[1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_recompute_zero_normals(&mut scene, 0.0);
        assert_eq!(r.recomputed_triangles, 0);
        let ns = scene.meshes[0].primitives[0].normals.as_ref().unwrap();
        // Pre-existing values preserved exactly.
        assert_eq!(ns[0], [1.0, 0.0, 0.0]);
        assert_eq!(ns[1], [0.0, 0.0, 0.0]);
        assert_eq!(ns[2], [0.0, 0.0, 0.0]);
    }

    #[test]
    fn recompute_zero_normals_skips_degenerate_triangle() {
        // Collinear corners — cross product is zero. Pass must
        // count the triangle under skipped_degenerate and leave the
        // stored zeros alone.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]];
        prim.normals = Some(vec![[0.0; 3]; 3]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_recompute_zero_normals(&mut scene, 0.0);
        assert_eq!(r.recomputed_triangles, 0);
        assert_eq!(r.skipped_degenerate, 1);
        let ns = scene.meshes[0].primitives[0].normals.as_ref().unwrap();
        assert_eq!(ns[0], [0.0, 0.0, 0.0]);
    }

    #[test]
    fn recompute_zero_normals_eps_widens_zero_detection() {
        // Stored normal is float-noise zero — strict `eps == 0.0`
        // refuses to touch it, `eps = 1e-3` lets it through.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        prim.normals = Some(vec![[1e-5, -1e-5, 1e-5]; 3]);

        let mut strict = Scene3D::new();
        strict.add_mesh(Mesh::new(None::<String>).with_primitive(prim.clone()));
        let r_strict = repair_recompute_zero_normals(&mut strict, 0.0);
        assert_eq!(r_strict.recomputed_triangles, 0);

        let mut loose = Scene3D::new();
        loose.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r_loose = repair_recompute_zero_normals(&mut loose, 1e-3);
        assert_eq!(r_loose.recomputed_triangles, 1);
        let ns = loose.meshes[0].primitives[0].normals.as_ref().unwrap();
        assert!((ns[0][2] - 1.0).abs() < 1e-3);
    }

    #[test]
    fn recompute_zero_normals_skips_length_mismatch() {
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        // Deliberately too short.
        prim.normals = Some(vec![[0.0; 3]; 2]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_recompute_zero_normals(&mut scene, 0.0);
        assert_eq!(r.skipped_length_mismatch, 1);
        assert_eq!(r.recomputed_triangles, 0);
        // Untouched.
        assert_eq!(
            scene.meshes[0].primitives[0]
                .normals
                .as_ref()
                .unwrap()
                .len(),
            2
        );
    }

    #[test]
    fn recompute_zero_normals_idempotent_on_clean_scene() {
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        prim.normals = Some(vec![[0.0; 3]; 3]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r1 = repair_recompute_zero_normals(&mut scene, 0.0);
        assert_eq!(r1.recomputed_triangles, 1);
        // Second pass sees the now-populated normals and does nothing.
        let r2 = repair_recompute_zero_normals(&mut scene, 0.0);
        assert_eq!(r2.recomputed_triangles, 0);
    }

    #[test]
    fn recompute_zero_normals_skips_non_triangles_primitive() {
        let mut prim = Primitive::new(Topology::Lines);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]];
        prim.normals = Some(vec![[0.0; 3]; 2]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_recompute_zero_normals(&mut scene, 0.0);
        assert_eq!(r.triangles_inspected, 0);
        assert_eq!(r.recomputed_triangles, 0);
        assert_eq!(r.primitives_populated, 0);
    }

    #[test]
    fn recompute_zero_normals_negative_eps_clamps_to_zero() {
        // Identical behaviour to eps==0.0: refuse to touch float-noise.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        prim.normals = Some(vec![[1e-5, 0.0, 0.0]; 3]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_recompute_zero_normals(&mut scene, -1.0);
        assert_eq!(r.recomputed_triangles, 0);
    }

    // ----- repair_orient_normals_from_winding -----

    #[test]
    fn orient_normals_flips_inverted_normal() {
        // Triangle in the XY plane with CCW winding viewed from +Z:
        // RHR cross product is (0,0,1). Stored normal is the opposite
        // direction (0,0,-1) — the repair must flip it.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        prim.normals = Some(vec![[0.0, 0.0, -1.0]; 3]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_orient_normals_from_winding(&mut scene, 0.0);
        assert_eq!(r.triangles_inspected, 1);
        assert_eq!(r.flipped_normals, 1);
        let ns = scene.meshes[0].primitives[0].normals.as_ref().unwrap();
        for n in ns {
            assert!((n[2] - 1.0).abs() < 1e-6, "expected +Z, got {:?}", n);
        }
    }

    #[test]
    fn orient_normals_leaves_aligned_normal_alone() {
        // RHR cross product is (0,0,1); stored normal matches.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        prim.normals = Some(vec![[0.0, 0.0, 1.0]; 3]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_orient_normals_from_winding(&mut scene, 0.0);
        assert_eq!(r.flipped_normals, 0);
        let ns = scene.meshes[0].primitives[0].normals.as_ref().unwrap();
        // Stored value preserved exactly.
        for n in ns {
            assert_eq!(*n, [0.0, 0.0, 1.0]);
        }
    }

    #[test]
    fn orient_normals_leaves_slightly_off_normal_alone() {
        // Stored normal is a non-unit but same-direction normal —
        // dot(stored, recomputed) > 0, so orientation matches. The
        // unit-length fix-up is `repair_normalize_unit_normals`'s
        // job, not this one's.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        prim.normals = Some(vec![[0.0, 0.0, 3.0]; 3]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_orient_normals_from_winding(&mut scene, 0.0);
        assert_eq!(r.flipped_normals, 0);
        let ns = scene.meshes[0].primitives[0].normals.as_ref().unwrap();
        for n in ns {
            assert_eq!(*n, [0.0, 0.0, 3.0]);
        }
    }

    #[test]
    fn orient_normals_skips_zero_sentinel() {
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        prim.normals = Some(vec![[0.0; 3]; 3]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_orient_normals_from_winding(&mut scene, 0.0);
        assert_eq!(r.flipped_normals, 0);
        assert_eq!(r.skipped_zero_normal, 1);
        // Stored zeros preserved (recompute is the other pass's job).
        let ns = scene.meshes[0].primitives[0].normals.as_ref().unwrap();
        for n in ns {
            assert_eq!(*n, [0.0, 0.0, 0.0]);
        }
    }

    #[test]
    fn orient_normals_skips_degenerate_triangle() {
        // Collinear corners — cross product is zero. Stored normal is
        // non-zero so the zero-sentinel branch does not fire; the
        // degenerate-skip branch must.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]];
        prim.normals = Some(vec![[1.0, 0.0, 0.0]; 3]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_orient_normals_from_winding(&mut scene, 0.0);
        assert_eq!(r.flipped_normals, 0);
        assert_eq!(r.skipped_degenerate, 1);
    }

    #[test]
    fn orient_normals_skips_missing_normals_field() {
        // None normals = nothing to reorient. Run
        // repair_recompute_zero_normals first to populate.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        prim.normals = None;
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_orient_normals_from_winding(&mut scene, 0.0);
        assert_eq!(r.flipped_normals, 0);
        assert_eq!(r.skipped_missing_normals, 1);
    }

    #[test]
    fn orient_normals_skips_length_mismatch() {
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        prim.normals = Some(vec![[0.0, 0.0, 1.0]; 2]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_orient_normals_from_winding(&mut scene, 0.0);
        assert_eq!(r.skipped_length_mismatch, 1);
        assert_eq!(r.flipped_normals, 0);
    }

    #[test]
    fn orient_normals_idempotent_on_clean_scene() {
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        prim.normals = Some(vec![[0.0, 0.0, -1.0]; 3]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r1 = repair_orient_normals_from_winding(&mut scene, 0.0);
        assert_eq!(r1.flipped_normals, 1);
        let r2 = repair_orient_normals_from_winding(&mut scene, 0.0);
        // Second pass sees the now-aligned normal and does nothing.
        assert_eq!(r2.flipped_normals, 0);
    }

    #[test]
    fn orient_normals_skips_non_triangles_primitive() {
        let mut prim = Primitive::new(Topology::Lines);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]];
        prim.normals = Some(vec![[0.0, 0.0, -1.0]; 2]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_orient_normals_from_winding(&mut scene, 0.0);
        assert_eq!(r.triangles_inspected, 0);
        assert_eq!(r.flipped_normals, 0);
    }

    // ----- repair_normalize_unit_normals -----

    #[test]
    fn normalize_rescales_overlong_normal_to_unit() {
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        // Length-3 stored normal along +Z.
        prim.normals = Some(vec![[0.0, 0.0, 3.0]; 3]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_normalize_unit_normals(&mut scene, 1e-3);
        assert_eq!(r.triangles_inspected, 1);
        assert_eq!(r.rescaled_normals, 1);
        let ns = scene.meshes[0].primitives[0].normals.as_ref().unwrap();
        for n in ns {
            let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            assert!((len - 1.0).abs() < 1e-6, "len = {len}");
        }
    }

    #[test]
    fn normalize_rescales_undersize_normal_to_unit() {
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        prim.normals = Some(vec![[0.0, 0.0, 0.25]; 3]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_normalize_unit_normals(&mut scene, 1e-3);
        assert_eq!(r.rescaled_normals, 1);
        let ns = scene.meshes[0].primitives[0].normals.as_ref().unwrap();
        for n in ns {
            assert!((n[2] - 1.0).abs() < 1e-6);
        }
    }

    #[test]
    fn normalize_leaves_unit_normal_alone() {
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        prim.normals = Some(vec![[0.0, 0.0, 1.0]; 3]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_normalize_unit_normals(&mut scene, 1e-3);
        assert_eq!(r.rescaled_normals, 0);
        let ns = scene.meshes[0].primitives[0].normals.as_ref().unwrap();
        for n in ns {
            assert_eq!(*n, [0.0, 0.0, 1.0]);
        }
    }

    #[test]
    fn normalize_skips_zero_sentinel() {
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        prim.normals = Some(vec![[0.0; 3]; 3]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_normalize_unit_normals(&mut scene, 1e-3);
        assert_eq!(r.rescaled_normals, 0);
        assert_eq!(r.skipped_zero_normal, 1);
        // Stored zeros preserved.
        let ns = scene.meshes[0].primitives[0].normals.as_ref().unwrap();
        for n in ns {
            assert_eq!(*n, [0.0, 0.0, 0.0]);
        }
    }

    #[test]
    fn normalize_skips_missing_normals_field() {
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        prim.normals = None;
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_normalize_unit_normals(&mut scene, 1e-3);
        assert_eq!(r.rescaled_normals, 0);
        assert_eq!(r.skipped_missing_normals, 1);
    }

    #[test]
    fn normalize_skips_length_mismatch() {
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        prim.normals = Some(vec![[0.0, 0.0, 5.0]; 2]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_normalize_unit_normals(&mut scene, 1e-3);
        assert_eq!(r.skipped_length_mismatch, 1);
        assert_eq!(r.rescaled_normals, 0);
    }

    #[test]
    fn normalize_idempotent_on_already_unit_scene() {
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        prim.normals = Some(vec![[0.0, 0.0, 2.5]; 3]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r1 = repair_normalize_unit_normals(&mut scene, 1e-3);
        assert_eq!(r1.rescaled_normals, 1);
        let r2 = repair_normalize_unit_normals(&mut scene, 1e-3);
        assert_eq!(r2.rescaled_normals, 0);
    }

    #[test]
    fn normalize_skips_non_triangles_primitive() {
        let mut prim = Primitive::new(Topology::Lines);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]];
        prim.normals = Some(vec![[0.0, 0.0, 7.0]; 2]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_normalize_unit_normals(&mut scene, 1e-3);
        assert_eq!(r.triangles_inspected, 0);
        assert_eq!(r.rescaled_normals, 0);
    }

    #[test]
    fn normalize_negative_tolerance_clamps_to_default() {
        // Negative tolerance is clamped to the validate-module
        // default (1e-3), so an already-unit normal is left alone.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        prim.normals = Some(vec![[0.0, 0.0, 1.0]; 3]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = repair_normalize_unit_normals(&mut scene, -1.0);
        assert_eq!(r.rescaled_normals, 0);
    }

    #[test]
    fn normalize_composes_with_orient() {
        // Inverted-and-overlong stored normal: orient flips it (now
        // correctly pointing +Z), normalize rescales to unit length.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        prim.normals = Some(vec![[0.0, 0.0, -4.0]; 3]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let orient = repair_orient_normals_from_winding(&mut scene, 0.0);
        assert_eq!(orient.flipped_normals, 1);
        let n_after_orient = scene.meshes[0].primitives[0].normals.as_ref().unwrap()[0];
        // After orient: the recomputed unit normal is (0,0,1), so
        // the flipped value is exactly unit-length (orient passes
        // through `recompute_face_normal` which already normalises).
        assert!((n_after_orient[2] - 1.0).abs() < 1e-6);
        let r = repair_normalize_unit_normals(&mut scene, 1e-3);
        // No further rescale because orient already left it unit.
        assert_eq!(r.rescaled_normals, 0);
    }

    // ---- repair_sort_triangles_by_z ----

    // Build an unindexed primitive from explicit per-face corner
    // triples (z-tagged so the sort order is obvious).
    fn soup_from_faces(faces: &[[[f32; 3]; 3]]) -> Primitive {
        let mut prim = Primitive::new(Topology::Triangles);
        for f in faces {
            for v in f {
                prim.positions.push(*v);
            }
        }
        prim
    }

    // The min-corner z of every face in emit order, for assertions.
    fn face_min_zs(prim: &Primitive) -> Vec<f32> {
        let face_count = match &prim.indices {
            Some(idx) => idx.len() / 3,
            None => prim.positions.len() / 3,
        };
        (0..face_count)
            .map(|f| triangle_z_key(prim, f)[0])
            .collect()
    }

    #[test]
    fn sort_by_z_empty_scene_is_noop() {
        let mut scene = Scene3D::new();
        let r = repair_sort_triangles_by_z(&mut scene);
        assert_eq!(r, SortByZReport::default());
    }

    #[test]
    fn sort_by_z_unindexed_orders_ascending() {
        // Three flat triangles at z = 5, 1, 3 (emitted in that order).
        let flat = |z: f32| [[0.0, 0.0, z], [1.0, 0.0, z], [0.0, 1.0, z]];
        let mut scene =
            scene_with_primitives(vec![soup_from_faces(&[flat(5.0), flat(1.0), flat(3.0)])]);
        let r = repair_sort_triangles_by_z(&mut scene);
        assert_eq!(r.triangles_inspected, 3);
        // All three faces move (none was already in its sorted slot:
        // 5→slot2, 1→slot0, 3→slot1).
        assert_eq!(r.triangles_reordered, 3);
        let zs = face_min_zs(&scene.meshes[0].primitives[0]);
        assert_eq!(zs, vec![1.0, 3.0, 5.0]);
    }

    #[test]
    fn sort_by_z_already_sorted_is_idempotent_zero() {
        let flat = |z: f32| [[0.0, 0.0, z], [1.0, 0.0, z], [0.0, 1.0, z]];
        let mut scene =
            scene_with_primitives(vec![soup_from_faces(&[flat(1.0), flat(2.0), flat(3.0)])]);
        let r = repair_sort_triangles_by_z(&mut scene);
        assert_eq!(r.triangles_inspected, 3);
        assert_eq!(r.triangles_reordered, 0);
        // Buffer unchanged.
        let zs = face_min_zs(&scene.meshes[0].primitives[0]);
        assert_eq!(zs, vec![1.0, 2.0, 3.0]);
    }

    #[test]
    fn sort_by_z_second_pass_reorders_nothing() {
        let flat = |z: f32| [[0.0, 0.0, z], [1.0, 0.0, z], [0.0, 1.0, z]];
        let mut scene =
            scene_with_primitives(vec![soup_from_faces(&[flat(9.0), flat(2.0), flat(7.0)])]);
        let first = repair_sort_triangles_by_z(&mut scene);
        assert!(first.triangles_reordered > 0);
        let second = repair_sort_triangles_by_z(&mut scene);
        assert_eq!(second.triangles_reordered, 0);
    }

    #[test]
    fn sort_by_z_keys_on_min_corner_not_max() {
        // Tilted triangles: face A spans z 0..10, face B spans z 1..2.
        // A's min (0) < B's min (1), so A must come first even though
        // A's max is higher.
        let a = [[0.0, 0.0, 0.0], [1.0, 0.0, 10.0], [0.0, 1.0, 5.0]];
        let b = [[0.0, 0.0, 2.0], [1.0, 0.0, 1.0], [0.0, 1.0, 1.5]];
        let mut scene = scene_with_primitives(vec![soup_from_faces(&[b, a])]);
        repair_sort_triangles_by_z(&mut scene);
        let prim = &scene.meshes[0].primitives[0];
        // Face 0 should now be A (min z = 0), face 1 should be B.
        assert_eq!(triangle_z_key(prim, 0)[0], 0.0);
        assert_eq!(triangle_z_key(prim, 1)[0], 1.0);
    }

    #[test]
    fn sort_by_z_stable_for_equal_keys() {
        // Two faces with identical z-key triples but distinguishable
        // by their x — a stable sort preserves emit order. Tag x = 100
        // and x = 200 to identify them post-sort.
        let mk = |x: f32| [[x, 0.0, 1.0], [x + 1.0, 0.0, 1.0], [x, 1.0, 1.0]];
        let mut scene = scene_with_primitives(vec![soup_from_faces(&[mk(100.0), mk(200.0)])]);
        let r = repair_sort_triangles_by_z(&mut scene);
        // Equal keys ⇒ identity permutation ⇒ nothing moves.
        assert_eq!(r.triangles_reordered, 0);
        let prim = &scene.meshes[0].primitives[0];
        assert_eq!(prim.positions[0][0], 100.0);
        assert_eq!(prim.positions[3][0], 200.0);
    }

    #[test]
    fn sort_by_z_indexed_preserves_u16_discriminant() {
        // Three flat triangles z = 3, 1, 2 via a U16 index buffer over
        // a shared (here trivially distinct) position list.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![
            [0.0, 0.0, 3.0],
            [1.0, 0.0, 3.0],
            [0.0, 1.0, 3.0], // face 0 @ z3
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [0.0, 1.0, 1.0], // face 1 @ z1
            [0.0, 0.0, 2.0],
            [1.0, 0.0, 2.0],
            [0.0, 1.0, 2.0], // face 2 @ z2
        ];
        prim.indices = Some(Indices::U16(vec![0, 1, 2, 3, 4, 5, 6, 7, 8]));
        let mut scene = scene_with_primitives(vec![prim]);
        let r = repair_sort_triangles_by_z(&mut scene);
        assert_eq!(r.triangles_inspected, 3);
        assert_eq!(r.triangles_reordered, 3);
        let prim = &scene.meshes[0].primitives[0];
        // Positions array untouched; only the index buffer reordered.
        assert_eq!(prim.positions[0][2], 3.0);
        match prim.indices.as_ref().unwrap() {
            Indices::U16(idx) => {
                // Sorted face order: z1 (3,4,5), z2 (6,7,8), z3 (0,1,2).
                assert_eq!(idx, &vec![3, 4, 5, 6, 7, 8, 0, 1, 2]);
            }
            other => panic!("discriminant changed: {other:?}"),
        }
    }

    #[test]
    fn sort_by_z_indexed_preserves_u32_discriminant() {
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![
            [0.0, 0.0, 9.0],
            [1.0, 0.0, 9.0],
            [0.0, 1.0, 9.0],
            [0.0, 0.0, 4.0],
            [1.0, 0.0, 4.0],
            [0.0, 1.0, 4.0],
        ];
        prim.indices = Some(Indices::U32(vec![0, 1, 2, 3, 4, 5]));
        let mut scene = scene_with_primitives(vec![prim]);
        repair_sort_triangles_by_z(&mut scene);
        let prim = &scene.meshes[0].primitives[0];
        match prim.indices.as_ref().unwrap() {
            Indices::U32(idx) => assert_eq!(idx, &vec![3, 4, 5, 0, 1, 2]),
            other => panic!("discriminant changed: {other:?}"),
        }
    }

    #[test]
    fn sort_by_z_unindexed_carries_normals_along() {
        // Two faces at z = 5 then z = 1, each with a distinctive normal
        // so we can confirm the normals follow their face after sort.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![
            [0.0, 0.0, 5.0],
            [1.0, 0.0, 5.0],
            [0.0, 1.0, 5.0], // face hi
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [0.0, 1.0, 1.0], // face lo
        ];
        prim.normals = Some(vec![
            [0.5, 0.0, 0.0],
            [0.5, 0.0, 0.0],
            [0.5, 0.0, 0.0], // hi-face normals
            [0.0, 0.9, 0.0],
            [0.0, 0.9, 0.0],
            [0.0, 0.9, 0.0], // lo-face normals
        ]);
        let mut scene = scene_with_primitives(vec![prim]);
        repair_sort_triangles_by_z(&mut scene);
        let prim = &scene.meshes[0].primitives[0];
        // After sort, lo face (z=1) is first; its normal (0,0.9,0)
        // must lead the normals array.
        assert_eq!(prim.positions[0][2], 1.0);
        let ns = prim.normals.as_ref().unwrap();
        assert_eq!(ns[0], [0.0, 0.9, 0.0]);
        assert_eq!(ns[3], [0.5, 0.0, 0.0]);
    }

    #[test]
    fn sort_by_z_skips_non_triangles() {
        let mut points = Primitive::new(Topology::Points);
        points.positions = vec![[0.0, 0.0, 9.0], [0.0, 0.0, 1.0]];
        let mut scene = scene_with_primitives(vec![points]);
        let r = repair_sort_triangles_by_z(&mut scene);
        assert_eq!(r.triangles_inspected, 0);
        assert_eq!(r.triangles_reordered, 0);
        // Points primitive untouched.
        assert_eq!(scene.meshes[0].primitives[0].positions[0][2], 9.0);
    }

    #[test]
    fn sort_by_z_all_nan_face_sorts_last() {
        // A face whose three corners all carry a non-finite z has a NaN
        // *minimum* key; total_cmp ranks NaN highest, so it sorts after
        // every finite face rather than scrambling them. A face with a
        // *single* NaN corner still sorts on its finite minimum (NaN is
        // pushed to the high end of the per-face z triple), which is
        // why this test uses an all-NaN face for the "sorts last" claim.
        let flat = |z: f32| [[0.0, 0.0, z], [1.0, 0.0, z], [0.0, 1.0, z]];
        let nan_face = [
            [0.0, 0.0, f32::NAN],
            [1.0, 0.0, f32::NAN],
            [0.0, 1.0, f32::NAN],
        ];
        let mut scene =
            scene_with_primitives(vec![soup_from_faces(&[nan_face, flat(8.0), flat(2.0)])]);
        repair_sort_triangles_by_z(&mut scene);
        let prim = &scene.meshes[0].primitives[0];
        // Finite faces first (z=2 then z=8), all-NaN face last.
        assert_eq!(triangle_z_key(prim, 0)[0], 2.0);
        assert_eq!(triangle_z_key(prim, 1)[0], 8.0);
        assert!(triangle_z_key(prim, 2)[0].is_nan());
    }

    #[test]
    fn sort_by_z_single_nan_corner_keys_on_finite_min() {
        // A face with one NaN corner but two finite ones keys on the
        // smaller finite z (the NaN is pushed to the top of the per-
        // face triple by total_cmp). Here the partly-NaN face's min is
        // 2.0, so it ties with — and (stable) precedes — the flat z=2
        // face emitted after it, and both precede z=8.
        let flat = |z: f32| [[0.0, 0.0, z], [1.0, 0.0, z], [0.0, 1.0, z]];
        let part_nan = [[0.0, 0.0, f32::NAN], [1.0, 0.0, 2.0], [0.0, 1.0, 3.0]];
        let mut scene =
            scene_with_primitives(vec![soup_from_faces(&[flat(8.0), part_nan, flat(2.0)])]);
        repair_sort_triangles_by_z(&mut scene);
        let prim = &scene.meshes[0].primitives[0];
        // Slot 0 + slot 1 both have min z 2.0; slot 2 is z=8.
        assert_eq!(triangle_z_key(prim, 0)[0], 2.0);
        assert_eq!(triangle_z_key(prim, 1)[0], 2.0);
        assert_eq!(triangle_z_key(prim, 2)[0], 8.0);
    }

    #[test]
    fn sort_by_z_preserves_triangle_count() {
        let flat = |z: f32| [[0.0, 0.0, z], [1.0, 0.0, z], [0.0, 1.0, z]];
        let faces: Vec<_> = (0..20).map(|i| flat((20 - i) as f32)).collect();
        let mut scene = scene_with_primitives(vec![soup_from_faces(&faces)]);
        let before = scene.meshes[0].primitives[0].positions.len();
        let r = repair_sort_triangles_by_z(&mut scene);
        assert_eq!(r.triangles_inspected, 20);
        assert_eq!(scene.meshes[0].primitives[0].positions.len(), before);
        let zs = face_min_zs(&scene.meshes[0].primitives[0]);
        let mut sorted = zs.clone();
        sorted.sort_by(f32::total_cmp);
        assert_eq!(zs, sorted);
    }
}

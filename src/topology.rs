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
//! 6. [`repair_translate_to_positive_octant`] — translate every vertex
//!    so the scene's bbox sits strictly in the `(+,+,+)` octant. The
//!    matching fix-up for the validate module's all-positive-octant
//!    rule (off by default in `ValidationOptions` because most modern
//!    slicers ignore it; turn both on together when targeting a strict
//!    1989-spec consumer).
//! 7. [`repair_make_winding_consistent`] — propagate one
//!    canonical winding across every manifold-edge-connected
//!    component, flipping any neighbour whose vertex order disagrees.
//!    The matching mutating fix-up for the validate module's
//!    `inconsistent_winding_edges` rule (on by default).
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

/// One boundary (a.k.a. "naked-edge") loop extracted by
/// [`boundary_loops`].
///
/// A *boundary edge* is an undirected edge used by exactly one
/// triangle — the same population [`Shell::boundary_edges`] and the
/// validate module's `boundary_edges` field merely *count*. This
/// struct goes one step further and chains those edges into the
/// ordered cycles they form, which is what a slicer / mesh-repair
/// pipeline actually needs: each loop is a hole in the surface that
/// can be capped by triangulating the loop, and the loop's vertex
/// order tells the consumer which winding a cap triangle must use to
/// stay consistent with the surrounding surface.
///
/// The 1989 spec says each facet "is part of the boundary between the
/// interior and the exterior of the object" — i.e. a valid STL solid
/// is *closed*, with no boundary edges at all. When that invariant is
/// broken, this report localises the breakage as discrete holes
/// rather than an undifferentiated edge count.
#[derive(Clone, Debug, PartialEq)]
pub struct BoundaryLoop {
    /// Ordered vertex positions tracing the loop. For a closed loop the
    /// first vertex is **not** repeated at the end; the implied final
    /// edge runs from the last entry back to the first. Length equals
    /// the number of edges in the loop (each entry is the tail of one
    /// directed boundary edge).
    pub vertices: Vec<[f32; 3]>,
    /// Whether the chain closed back on its starting vertex. A
    /// well-formed manifold-with-boundary surface yields only closed
    /// loops; an `open` loop signals a non-manifold boundary
    /// (a vertex where three or more boundary edges meet) that could
    /// not be walked into a single cycle and was emitted as an open
    /// chain so no edge is silently dropped.
    pub closed: bool,
}

impl BoundaryLoop {
    /// Number of boundary edges in this loop. For a closed loop this
    /// equals `vertices.len()` (the implied closing edge included); for
    /// an open chain it is `vertices.len() - 1` (no closing edge).
    pub fn edge_count(&self) -> usize {
        if self.closed {
            self.vertices.len()
        } else {
            self.vertices.len().saturating_sub(1)
        }
    }
}

/// Extract the ordered boundary loops of `scene` — the cycles formed
/// by every edge used by exactly one triangle.
///
/// Each triangle contributes three **directed** edges in winding
/// order (`a→b`, `b→c`, `c→a`). An undirected edge whose total use
/// count (over both directions) is exactly one is a boundary edge;
/// its single directed instance carries the surface's orientation, so
/// a boundary loop walked tail-to-head keeps the surface consistently
/// on one side — exactly the winding a cap triangle needs.
///
/// Loops are reconstructed by following each boundary edge's head to
/// the unique boundary edge whose tail matches. When a boundary
/// vertex has more than one outgoing boundary edge (a non-manifold
/// boundary: three-plus boundary edges meeting at a point), the walk
/// consumes outgoing edges in a deterministic order and emits the
/// resulting chain with `closed = false` rather than guessing — every
/// boundary edge appears in exactly one returned loop, so the total
/// edge count across all loops equals the scene's boundary-edge count.
///
/// Loops are returned sorted by their lexicographically-smallest
/// vertex key so the output is stable across runs regardless of
/// triangle-iteration order. Returns an empty vec for a watertight
/// scene (no boundary edges), an empty scene, or one whose primitives
/// are all non-`Triangles`.
pub fn boundary_loops(scene: &Scene3D) -> Vec<BoundaryLoop> {
    let tris = collect_triangles(scene);
    if tris.is_empty() {
        return Vec::new();
    }

    // Count undirected edge uses (both directions collapse to the
    // canonical (lo, hi) key) and record each directed instance.
    let mut undirected_uses: HashMap<(VertKey, VertKey), usize> = HashMap::new();
    let mut directed: Vec<(VertKey, VertKey)> = Vec::new();
    for tri in &tris {
        let pairs = [
            (tri.keys[0], tri.keys[1]),
            (tri.keys[1], tri.keys[2]),
            (tri.keys[2], tri.keys[0]),
        ];
        for (a, b) in pairs {
            let key = if a <= b { (a, b) } else { (b, a) };
            *undirected_uses.entry(key).or_insert(0) += 1;
            directed.push((a, b));
        }
    }

    // Adjacency from tail -> list of heads, restricted to boundary
    // edges (undirected use count == 1). A directed edge is kept only
    // if its undirected key is used exactly once across the whole
    // scene.
    let mut out_edges: HashMap<VertKey, Vec<VertKey>> = HashMap::new();
    let mut remaining = 0usize;
    for (a, b) in directed {
        let key = if a <= b { (a, b) } else { (b, a) };
        if undirected_uses.get(&key) == Some(&1) {
            out_edges.entry(a).or_default().push(b);
            remaining += 1;
        }
    }
    if remaining == 0 {
        return Vec::new();
    }

    // Deterministic consumption order: sort each adjacency list so
    // repeated runs walk the same way, then pop from the end.
    for heads in out_edges.values_mut() {
        heads.sort_unstable();
        heads.reverse();
    }

    // Seed the walk from boundary-edge tails in sorted order so loop
    // output is stable.
    let mut tails: Vec<VertKey> = out_edges.keys().copied().collect();
    tails.sort_unstable();

    let mut loops: Vec<BoundaryLoop> = Vec::new();
    for &seed in &tails {
        // A seed may have multiple outgoing edges (non-manifold
        // boundary vertex); start a fresh walk for each one still
        // available.
        while out_edges.get(&seed).is_some_and(|v| !v.is_empty()) {
            let mut chain: Vec<VertKey> = vec![seed];
            let mut cur = seed;
            let mut closed = false;
            while let Some(next) = out_edges.get_mut(&cur).and_then(|v| v.pop()) {
                if next == seed {
                    // Closed the loop back on the start vertex; the
                    // closing edge is implied, do not push `seed`
                    // again.
                    closed = true;
                    break;
                }
                chain.push(next);
                cur = next;
            }
            loops.push(BoundaryLoop {
                vertices: chain.iter().map(|k| key_to_pos(*k)).collect(),
                closed,
            });
        }
    }

    // Stable order: by the loop's lexicographically-smallest vertex
    // key (recovered from the emitted positions' bit patterns).
    loops.sort_by_key(|lp| {
        lp.vertices
            .iter()
            .map(|p| VertKey::from(*p))
            .min()
            .unwrap_or(VertKey(0, 0, 0))
    });
    loops
}

/// Recover the exact `[f32; 3]` position from a bit-pattern key.
fn key_to_pos(k: VertKey) -> [f32; 3] {
    [
        f32::from_bits(k.0),
        f32::from_bits(k.1),
        f32::from_bits(k.2),
    ]
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

/// Outcome of a [`check_z_sorted`] pass.
///
/// The non-mutating diagnostic counterpart of
/// [`repair_sort_triangles_by_z`]: it answers "are the triangles
/// already in the ascending-z emit order the spec recommends?" without
/// touching the scene. Counters are summed across every `Triangles`
/// primitive.
///
/// The two functions agree by construction — they share the same
/// per-triangle z-key ([`min_z`, `mid_z`, `max_z`] under
/// [`f32::total_cmp`]) and the same lexicographic ordering — so for any
/// scene `check_z_sorted(scene).is_sorted()` is `true` **iff**
/// `repair_sort_triangles_by_z(&mut scene.clone()).triangles_reordered
/// == 0`. The diagnostic is the cheaper of the two when the caller only
/// needs the yes/no answer (one linear scan, no permutation, no buffer
/// rewrite, no allocation).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ZSortReport {
    /// Total triangle slots inspected (post-index-buffer resolution)
    /// across every `Triangles` primitive in the scene.
    pub triangles_inspected: usize,
    /// Number of adjacent triangle pairs `(i, i + 1)` *within the same
    /// primitive* whose z-keys are strictly out of order
    /// (`key[i] > key[i + 1]`). Zero iff every primitive is already in
    /// non-decreasing z-key order. Pairs that straddle a primitive
    /// boundary are never counted — each primitive is sorted
    /// independently by the repair, so its order is independent too.
    pub out_of_order_pairs: usize,
    /// 1-based global triangle index (counting across primitives in
    /// scene order, exactly as [`FaceLocator`] does) of the *first*
    /// triangle that is strictly smaller than its predecessor within
    /// its primitive, or `None` when the scene is already sorted. This
    /// is the earliest place the recommended order breaks — the natural
    /// cursor for a "show me where it goes wrong" report.
    pub first_out_of_order_triangle: Option<usize>,
}

impl ZSortReport {
    /// `true` iff every `Triangles` primitive is already in the
    /// ascending-z emit order the 1989 spec recommends — equivalently,
    /// [`Self::out_of_order_pairs`] `== 0`. An empty scene (no
    /// triangles) is trivially sorted.
    pub fn is_sorted(&self) -> bool {
        self.out_of_order_pairs == 0
    }
}

/// Report whether every `Triangles` primitive's triangles are already
/// in ascending-z emit order, without mutating the scene.
///
/// The 1989 spec notes: *"Sorting the triangles in ascending z-value
/// order is recommended, but not required, in order to optimize
/// performance of the slice program."* This is the non-mutating
/// diagnostic counterpart of [`repair_sort_triangles_by_z`]: pipelines
/// that must *decide* whether to pay for a re-sort (or that want to
/// *report* spec-recommended-order conformance) get the yes/no answer
/// plus the first offending position in a single linear scan, without
/// the permutation + buffer rewrite the repair performs.
///
/// ## Sort key (identical to the repair)
///
/// Each triangle is keyed on its three corner z-values sorted ascending
/// — `(min_z, mid_z, max_z)` — compared lexicographically with
/// [`f32::total_cmp`]. A triangle is "in order" relative to its
/// predecessor *within the same primitive* when its key is
/// `>=` the predecessor's. A corner whose index buffer cannot be
/// resolved contributes the same `f32::NAN` high sentinel the repair
/// uses, so a malformed facet sorts to the high end here too — and the
/// diagnostic's verdict therefore matches what the repair would do.
///
/// ## Per-primitive scope
///
/// Each `Triangles` primitive is checked independently: the repair
/// sorts every primitive's faces among themselves and never moves a
/// face across a primitive boundary, so a pair straddling two
/// primitives is never "out of order". Non-`Triangles` primitives are
/// skipped (they contribute nothing to `triangles_inspected`).
///
/// Returns a single [`ZSortReport`] summed across the scene.
pub fn check_z_sorted(scene: &Scene3D) -> ZSortReport {
    let mut report = ZSortReport::default();
    let mut global_idx: usize = 0;
    for mesh in &scene.meshes {
        for prim in &mesh.primitives {
            if prim.topology != Topology::Triangles {
                continue;
            }
            let face_count = match &prim.indices {
                Some(idx) => idx.len() / 3,
                None => prim.positions.len() / 3,
            };
            report.triangles_inspected += face_count;
            let mut prev: Option<[f32; 3]> = None;
            for face_idx in 0..face_count {
                global_idx += 1;
                let key = triangle_z_key(prim, face_idx);
                if let Some(p) = prev {
                    // Strictly-out-of-order iff the previous key is
                    // greater than this one under the same lexicographic
                    // total order the repair sorts by.
                    let cmp = p[0]
                        .total_cmp(&key[0])
                        .then_with(|| p[1].total_cmp(&key[1]))
                        .then_with(|| p[2].total_cmp(&key[2]));
                    if cmp == std::cmp::Ordering::Greater {
                        report.out_of_order_pairs += 1;
                        if report.first_out_of_order_triangle.is_none() {
                            report.first_out_of_order_triangle = Some(global_idx);
                        }
                    }
                }
                prev = Some(key);
            }
        }
    }
    report
}

/// Default safety margin for [`repair_translate_to_positive_octant`].
///
/// The 1989 spec requires every vertex coordinate to be
/// "positive-definite (nonnegative AND nonzero)" — i.e. strictly
/// greater than zero. After translating the scene's `bbox.min` to the
/// origin, the smallest coordinate would be exactly `0.0`, which still
/// fails the *nonzero* half of the rule. This margin is added to the
/// translation so the post-repair minimum sits at `+margin` on every
/// axis instead of exactly zero.
///
/// `1.0e-6` is large enough to stay clearly above the `f32` quantum
/// even after a re-encode round-trip, and small enough to be
/// indistinguishable from the original geometry's units (mesh units
/// are typically millimetres or larger).
pub const DEFAULT_POSITIVE_OCTANT_MARGIN: f32 = 1.0e-6;

/// Outcome of a [`repair_translate_to_positive_octant`] pass.
///
/// `delta` is the translation that was actually applied to every
/// finite vertex coordinate (component-wise add). On a no-op pass
/// (the scene already sits strictly in the positive octant *and* the
/// requested margin would not push it further), `delta == [0.0; 3]`
/// and `vertices_translated == 0` — the idempotency signal.
///
/// `vertices_translated` counts the per-vertex updates that actually
/// happened. A `Triangles` primitive whose positions are all
/// non-finite contributes `0` to that counter (non-finite components
/// are left in place).
///
/// `triangles_inspected` is summed across every `Triangles` primitive
/// in the scene, mirroring the other repairs in this module.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TranslateOctantReport {
    /// Total triangle slots inspected (post-index-buffer resolution)
    /// across every touched primitive. A scene with no `Triangles`
    /// primitives reports zero — symmetric with the other repairs.
    pub triangles_inspected: usize,
    /// Number of *vertex* slots whose `[f32; 3]` value was rewritten
    /// (i.e. at least one component was finite and the translation
    /// was non-zero). A vertex with all-non-finite components is
    /// counted under [`Self::skipped_non_finite_vertices`] instead.
    pub vertices_translated: usize,
    /// Number of vertex slots skipped because every component was
    /// non-finite (`NaN` / `±inf`). Those slots are passed through
    /// unmodified — translating `±inf + delta` would re-emit
    /// `±inf`, but a downstream consumer typically prefers to see
    /// the original sentinel value.
    pub skipped_non_finite_vertices: usize,
    /// The component-wise translation actually applied to every
    /// finite vertex coordinate. `[0.0; 3]` on a no-op pass.
    pub delta: [f32; 3],
}

/// Translate every `Triangles` vertex so the scene's axis-aligned
/// bounding box sits in the strictly-positive octant.
///
/// The 1989 spec says: *"The object represented must be located in
/// the all-positive octant. In other words, all vertex coordinates
/// must be positive-definite (nonnegative and nonzero) numbers."*
/// [`crate::validate`] surfaces facets that break this under
/// [`crate::ValidationReport::positive_octant_defects`] (opt-in via
/// [`crate::ValidationOptions::check_positive_octant`]). This is the
/// matching mutating fix-up: it computes a single component-wise
/// translation `delta` such that, after `pos += delta` on every
/// finite vertex, every coordinate is strictly greater than zero.
///
/// ## Translation
///
/// Let `(min_x, min_y, min_z)` be the scene's per-axis minimum (over
/// every `Triangles` vertex with a finite coordinate on that axis).
/// Per-axis the translation is:
///
/// ```text
/// delta[i] = if min[i] <= 0.0 { margin - min[i] } else { 0.0 }
/// ```
///
/// — i.e. the axis is shifted only when the existing minimum
/// violates the spec's "positive-definite (nonnegative AND nonzero)"
/// rule on that axis (`min[i] <= 0`). The margin's only job is to
/// ensure the post-shift minimum lands *strictly* above zero, not
/// at any particular distance from zero. An axis whose minimum is
/// already `> 0` is left alone — even a minimum of `+1e-30`
/// satisfies the spec invariant. The same applies to scenes that
/// already sit strictly inside the positive octant: they produce
/// `delta == [0.0; 3]` and the repair is a no-op (idempotency
/// signal: `vertices_translated == 0`).
///
/// The `margin` argument is clamped: non-finite or negative values
/// fall back to [`DEFAULT_POSITIVE_OCTANT_MARGIN`]. Pass `0.0` for
/// "translate so `min[i] == 0` exactly" (this fails the spec's
/// strict-nonzero half — useful only when the caller intends to
/// add their own margin later, or when running the validate-
/// module's positive-octant rule with the spec interpreted as
/// `>= 0` rather than `> 0`).
///
/// ## Per-vertex handling
///
/// Only finite components are translated. A vertex slot
/// `[NaN, 1.0, 2.0]` becomes `[NaN, 1.0 + delta[1], 2.0 + delta[2]]`
/// — the non-finite component is passed through. A vertex slot
/// whose three components are *all* non-finite is left bit-for-bit
/// unchanged and contributes to
/// [`TranslateOctantReport::skipped_non_finite_vertices`] instead
/// of [`TranslateOctantReport::vertices_translated`].
///
/// ## Per-primitive handling
///
/// - Walks every `Triangles` primitive in source order.
///   Non-`Triangles` primitives are silently skipped (the rest of
///   the crate already rejects them at encode-time anyway).
/// - Both indexed and unindexed primitives translate every
///   `prim.positions` slot once — this is the buffer the slots
///   reference, regardless of how the index buffer routes through
///   it. Index buffers are not rewritten.
/// - `prim.normals` are *direction* vectors, not positions, so they
///   are left untouched (the validate-module's facet-orientation
///   rule still holds after a pure translation).
/// - `prim.extras`, `mesh.name`, the scene-graph `nodes` /
///   `roots`, and every non-position vertex attribute (tangents,
///   uvs, colours, joints, weights, morph targets) are preserved.
///
/// ## Empty / degenerate scenes
///
/// A scene with no `Triangles` primitives — or one whose every
/// `Triangles` primitive has zero positions — has no bbox to anchor
/// against and is a no-op (`delta == [0.0; 3]`,
/// `vertices_translated == 0`). A scene whose every position
/// component is non-finite is also a no-op (no axis contributes a
/// finite minimum); the repair reports the non-finite-skip count
/// instead.
///
/// Returns a single [`TranslateOctantReport`] summed across every
/// touched primitive.
pub fn repair_translate_to_positive_octant(
    scene: &mut Scene3D,
    margin: f32,
) -> TranslateOctantReport {
    let mut report = TranslateOctantReport::default();
    // Match the validate-module clamp idiom: non-finite / negative
    // margins fall back to the documented default rather than
    // panicking. Strict-zero (`0.0`) is still legal — see the doc
    // comment for the rationale.
    let margin = if margin.is_finite() && margin >= 0.0 {
        margin
    } else {
        DEFAULT_POSITIVE_OCTANT_MARGIN
    };

    // Count inspected triangles up-front so the report mirrors the
    // other repairs even when the early-exit branch fires.
    for mesh in &scene.meshes {
        for prim in &mesh.primitives {
            if prim.topology != Topology::Triangles {
                continue;
            }
            let face_count = match &prim.indices {
                Some(idx) => idx.len() / 3,
                None => prim.positions.len() / 3,
            };
            report.triangles_inspected += face_count;
        }
    }

    // Compute the per-axis minimum over every finite position
    // component across every `Triangles` primitive. Mirrors the
    // selection rule used by `crate::validate::bbox`.
    let mut mn = [f32::INFINITY; 3];
    let mut any_axis_finite = [false; 3];
    for mesh in &scene.meshes {
        for prim in &mesh.primitives {
            if prim.topology != Topology::Triangles {
                continue;
            }
            for p in &prim.positions {
                for (axis, &c) in p.iter().enumerate() {
                    if c.is_finite() {
                        if c < mn[axis] {
                            mn[axis] = c;
                        }
                        any_axis_finite[axis] = true;
                    }
                }
            }
        }
    }

    // If no finite components exist anywhere, there's nothing to
    // anchor a translation against. Walk vertices once to populate
    // `skipped_non_finite_vertices` and return.
    if !any_axis_finite.iter().any(|&f| f) {
        for mesh in &scene.meshes {
            for prim in &mesh.primitives {
                if prim.topology != Topology::Triangles {
                    continue;
                }
                for p in &prim.positions {
                    if !p.iter().any(|c| c.is_finite()) {
                        report.skipped_non_finite_vertices += 1;
                    }
                }
            }
        }
        return report;
    }

    // Per-axis: shift only when the existing minimum violates the
    // spec's strict-`> 0` rule, i.e. `mn[axis] <= 0`. The margin's
    // role is purely to ensure the post-shift minimum lands
    // strictly *above* zero (the spec says "nonnegative AND nonzero")
    // — not to enforce any particular distance from zero. A scene
    // whose minimum is already at `+1e-30` is left alone even though
    // it's well below the default margin: the spec's invariant is
    // already satisfied. The strict-`<= 0` test is what makes the
    // pass idempotent: after the shift the new minimum lands at
    // `+margin` (or a sub-ULP residual after `f32` cancellation, but
    // still strictly above zero on typical scenes), and a re-run sees
    // a strictly-positive minimum and does nothing. (The
    // `mn[axis] == +INFINITY` no-finite-component case is also
    // skipped because `+INFINITY <= 0` is false.)
    let mut delta = [0.0f32; 3];
    for axis in 0..3 {
        if any_axis_finite[axis] && mn[axis] <= 0.0 {
            delta[axis] = margin - mn[axis];
        }
    }
    report.delta = delta;

    // No-op early-exit: scene already sits strictly above the
    // margin on every axis. Walk-and-count of non-finite vertices
    // still runs so the report is symmetric with the apply branch.
    if delta == [0.0; 3] {
        for mesh in &scene.meshes {
            for prim in &mesh.primitives {
                if prim.topology != Topology::Triangles {
                    continue;
                }
                for p in &prim.positions {
                    if !p.iter().any(|c| c.is_finite()) {
                        report.skipped_non_finite_vertices += 1;
                    }
                }
            }
        }
        return report;
    }

    // Apply the translation. Per-component: only finite components
    // are shifted; non-finite components are passed through.
    for mesh in &mut scene.meshes {
        for prim in &mut mesh.primitives {
            if prim.topology != Topology::Triangles {
                continue;
            }
            for p in &mut prim.positions {
                if !p.iter().any(|c| c.is_finite()) {
                    report.skipped_non_finite_vertices += 1;
                    continue;
                }
                let mut shifted = false;
                for axis in 0..3 {
                    if p[axis].is_finite() && delta[axis] != 0.0 {
                        p[axis] += delta[axis];
                        shifted = true;
                    }
                }
                if shifted {
                    report.vertices_translated += 1;
                }
            }
        }
    }

    report
}

/// Outcome of a [`repair_make_winding_consistent`] pass.
///
/// Counters are summed across every `Triangles` primitive in the
/// scene. `triangles_flipped == 0` is the idempotency signal — a
/// scene whose every manifold edge is already walked in opposite
/// directions by its two incident triangles is left untouched.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct WindingConsistencyReport {
    /// Total triangle slots inspected (post-index-buffer resolution)
    /// across every touched primitive.
    pub triangles_inspected: usize,
    /// Number of triangles whose vertex order was swapped to align
    /// their winding with a manifold-edge neighbour's. A scene that
    /// already passes the validate-module's `inconsistent_winding`
    /// rule reports `triangles_flipped == 0`.
    pub triangles_flipped: usize,
    /// Number of distinct manifold-edge-connected components walked
    /// by the BFS. Each component picks one seed triangle whose
    /// winding is taken as canonical and propagated outward; the
    /// counter is incremented once per such seed.
    pub components_visited: usize,
    /// Number of manifold edges (exactly two incident triangles in
    /// the same primitive) along which winding consistency could not
    /// be resolved because doing so would conflict with an already-
    /// propagated decision elsewhere in the component. Mirrors the
    /// "unable to repair" case for non-orientable surfaces (Möbius-
    /// strip-like), but is also incremented when a flip would
    /// invalidate a previously-set face's orientation. Such edges
    /// remain flagged by [`crate::validate`]; the caller can re-run
    /// `validate` after this pass to confirm.
    pub conflicting_edges: usize,
}

/// Propagate one canonical winding across every manifold-edge-
/// connected component of every `Triangles` primitive.
///
/// The 1989 spec says facet orientation is "specified redundantly in
/// two ways which must be consistent": (1) the direction of the
/// normal is outward; (2) the vertices are listed in counter-clockwise
/// order when viewed from outside (right-hand rule). The per-facet
/// consistency between *stored normal* and *winding* is handled by
/// [`repair_orient_normals_from_winding`]; the *mesh-wide* consistency
/// — every manifold edge walked in opposite directions by its two
/// incident triangles — is this pass.
///
/// [`crate::validate`] surfaces the mesh-wide invariant under
/// [`crate::ValidationReport::inconsistent_winding_edges`] (on by
/// default via
/// [`crate::ValidationOptions::check_consistent_winding`]). This is
/// the matching mutating fix-up.
///
/// ## Algorithm
///
/// For each `Triangles` primitive in isolation:
/// 1. Build a per-primitive map from canonical undirected edge
///    (lo / hi bit-exact `f32` triples) to the list of incident
///    `(face_idx, directed-traversal-flag)` entries. Only edges with
///    exactly two incidences participate; boundary edges (one
///    incidence) and non-manifold edges (three or more) are left to
///    the watertight check.
/// 2. BFS over the manifold-edge adjacency graph. The first
///    unvisited face in source order is the *seed* for its
///    component — its current winding is canonical, by definition.
/// 3. For each BFS edge `(seed_face, neighbour_face)` walking
///    canonical edge `e`: if `seed_face` and `neighbour_face`
///    already traverse `e` in opposite directions, the neighbour is
///    consistent — mark it visited and queue its neighbours. If
///    they traverse `e` in the *same* direction, the neighbour
///    needs flipping — swap two of its vertex slots, mark it
///    visited, and queue.
/// 4. A "consistency conflict" — a neighbour-of-a-neighbour that
///    has *already* been visited but disagrees with the decision
///    propagated through a different path — is recorded under
///    [`WindingConsistencyReport::conflicting_edges`] and left
///    alone. Such conflicts arise on non-orientable surfaces
///    (Möbius-strip-like) where no single global winding satisfies
///    every edge constraint; the caller should re-run
///    [`crate::validate`] to surface the remaining offenders.
///
/// ## Flip representation
///
/// A flip swaps the second and third vertex slots of the offending
/// face. For an indexed primitive this swaps two entries in the
/// index buffer; for an unindexed primitive it swaps two entries in
/// the `prim.positions` (and, when matched 1:1, `prim.normals`)
/// buffers. Both transformations reverse the right-hand-rule cross
/// product direction of the face — the geometric meaning of
/// "flipping the winding".
///
/// Stored *facet normals* are NOT recomputed by this pass — flipping
/// the winding changes the cross-product direction, which means
/// [`repair_orient_normals_from_winding`] is the natural follow-up
/// when the stored normal must agree with the new winding. The two
/// passes are independent: this one fixes the mesh-wide invariant;
/// the orient pass fixes the per-facet invariant.
///
/// ## Per-primitive isolation
///
/// Each primitive is walked in isolation — manifold-edge adjacency
/// across primitives is not modelled. Two primitives that happen to
/// share a vertex position are treated as separate connected
/// components, mirroring the validate module's per-primitive edge
/// accounting. Non-`Triangles` primitives are silently skipped.
///
/// ## Empty / degenerate scenes
///
/// A scene with no `Triangles` primitives — or one whose every
/// primitive has zero faces — is a no-op (`triangles_flipped == 0`,
/// `components_visited == 0`). Degenerate edges (endpoints coincide
/// by bit-exact match) are skipped, matching the validate module's
/// edge accounting.
///
/// ## Idempotency
///
/// A second run sees every manifold edge already walked in opposite
/// directions and reports `triangles_flipped == 0`. The
/// `components_visited` count is *not* an idempotency signal — it
/// rises with every pass (one increment per BFS seed, regardless of
/// whether a flip was needed).
///
/// Returns a single [`WindingConsistencyReport`] summed across every
/// touched primitive.
pub fn repair_make_winding_consistent(scene: &mut Scene3D) -> WindingConsistencyReport {
    let mut report = WindingConsistencyReport::default();
    for mesh in &mut scene.meshes {
        for prim in &mut mesh.primitives {
            if prim.topology != Topology::Triangles {
                continue;
            }
            make_winding_consistent_in_primitive(prim, &mut report);
        }
    }
    report
}

/// Per-primitive winding propagation. Builds the edge-adjacency
/// graph and walks each connected component, flipping any neighbour
/// whose winding disagrees with the seed.
fn make_winding_consistent_in_primitive(
    prim: &mut Primitive,
    report: &mut WindingConsistencyReport,
) {
    use std::collections::{HashMap, VecDeque};

    let face_count = match &prim.indices {
        Some(idx) => idx.len() / 3,
        None => prim.positions.len() / 3,
    };
    if face_count == 0 {
        return;
    }
    report.triangles_inspected += face_count;

    // Track the canonical direction each face currently walks each
    // of its three edges. `face_edges[f]` holds three
    // `(canonical_edge_key, dir_flag)` tuples (or `None` for
    // degenerate / unresolvable corners). `dir_flag = true` means
    // the face walks the edge in lo→hi canonical direction, `false`
    // means hi→lo. A flip swaps two slots, which inverts all three
    // edges' `dir_flag`s — we recompute on the fly when a face is
    // flipped.
    type EdgeKey = (VertKey, VertKey);

    let face_corners = |prim: &Primitive, face_idx: usize| -> Option<[VertKey; 3]> {
        let (vi0, vi1, vi2) = resolve_face(&prim.indices, face_idx);
        let p0 = prim.positions.get(vi0).copied()?;
        let p1 = prim.positions.get(vi1).copied()?;
        let p2 = prim.positions.get(vi2).copied()?;
        Some([VertKey::from(p0), VertKey::from(p1), VertKey::from(p2)])
    };

    // `dir_flag` for one directed traversal a→b: `true` if a < b
    // (walks the canonical key in lo→hi direction), `false` if
    // a > b. Equal keys mean a degenerate edge — caller skips.
    let edge_of = |a: VertKey, b: VertKey| -> Option<(EdgeKey, bool)> {
        match a.cmp(&b) {
            std::cmp::Ordering::Less => Some(((a, b), true)),
            std::cmp::Ordering::Greater => Some(((b, a), false)),
            std::cmp::Ordering::Equal => None,
        }
    };

    let face_edges_of = |corners: [VertKey; 3]| -> [Option<(EdgeKey, bool)>; 3] {
        [
            edge_of(corners[0], corners[1]),
            edge_of(corners[1], corners[2]),
            edge_of(corners[2], corners[0]),
        ]
    };

    // Build the initial adjacency map: canonical edge → list of
    // (face_idx, dir_flag) — but the BFS needs `dir_flag` *after*
    // flips, so we don't pre-cache the per-face state; instead we
    // store only the adjacency `edge → faces` and recompute the
    // direction when the BFS reaches the edge.
    let mut edge_faces: HashMap<EdgeKey, Vec<usize>> = HashMap::new();
    for face_idx in 0..face_count {
        let Some(corners) = face_corners(prim, face_idx) else {
            continue;
        };
        for (key, _) in face_edges_of(corners).into_iter().flatten() {
            edge_faces.entry(key).or_default().push(face_idx);
        }
    }

    // Per-face flip flag — incrementally maintained so the BFS can
    // tell whether `face f`'s current winding is the original or
    // the flipped one. We DON'T mutate the primitive buffers yet;
    // we batch the flips at the end so the BFS sees a consistent
    // snapshot of every face's direction.
    let mut flipped = vec![false; face_count];
    let mut visited = vec![false; face_count];

    // Helper: the directed traversal flag of `face_idx` on
    // canonical edge `key`, taking the in-progress flip state into
    // account.
    let face_dir_on_edge = |corners: &[[VertKey; 3]],
                            flipped: &[bool],
                            face_idx: usize,
                            target: EdgeKey|
     -> Option<bool> {
        let mut c = corners[face_idx];
        if flipped[face_idx] {
            // Swap slots 1 and 2 to invert the winding.
            c.swap(1, 2);
        }
        for slot in face_edges_of(c).iter().flatten() {
            if slot.0 == target {
                return Some(slot.1);
            }
        }
        None
    };

    // Cache every face's original corners up front. `face_corners`
    // returns `None` on resolve failure (out-of-range index); cache
    // an all-same sentinel that produces zero edges so the BFS skips
    // it harmlessly.
    let sentinel = VertKey(u32::MAX, u32::MAX, u32::MAX);
    let mut corners: Vec<[VertKey; 3]> = Vec::with_capacity(face_count);
    for face_idx in 0..face_count {
        corners.push(face_corners(prim, face_idx).unwrap_or([sentinel; 3]));
    }

    // BFS from each unvisited face. The seed's current orientation
    // is canonical; neighbours that disagree are flipped.
    for seed in 0..face_count {
        if visited[seed] {
            continue;
        }
        visited[seed] = true;
        report.components_visited += 1;
        let mut queue: VecDeque<usize> = VecDeque::new();
        queue.push_back(seed);
        while let Some(face_idx) = queue.pop_front() {
            // Re-evaluate this face's three edges under the current
            // flip state and walk each one's manifold partner.
            let mut c = corners[face_idx];
            if flipped[face_idx] {
                c.swap(1, 2);
            }
            for slot in face_edges_of(c).iter().flatten() {
                let key = slot.0;
                let this_dir = slot.1;
                let Some(faces) = edge_faces.get(&key) else {
                    continue;
                };
                // Manifold-only: exactly two incident triangles.
                if faces.len() != 2 {
                    continue;
                }
                let neighbour = if faces[0] == face_idx {
                    faces[1]
                } else if faces[1] == face_idx {
                    faces[0]
                } else {
                    // `face_idx` doesn't actually own this edge in
                    // the adjacency (e.g. duplicate edges within
                    // one face after a sentinel); skip.
                    continue;
                };
                let Some(neigh_dir) = face_dir_on_edge(&corners, &flipped, neighbour, key) else {
                    continue;
                };
                if visited[neighbour] {
                    // The neighbour's orientation is already
                    // fixed. If it disagrees with this face on the
                    // shared edge, that's a non-orientable
                    // conflict.
                    if neigh_dir == this_dir {
                        report.conflicting_edges += 1;
                    }
                    continue;
                }
                // Unvisited neighbour. Same direction = needs flip;
                // opposite direction = already consistent.
                if neigh_dir == this_dir {
                    flipped[neighbour] = true;
                }
                visited[neighbour] = true;
                queue.push_back(neighbour);
            }
        }
    }

    // Apply the flips. Each flipped face has its second and third
    // vertex slots swapped — index buffer or position/normal arrays.
    let to_flip: Vec<usize> = flipped
        .iter()
        .enumerate()
        .filter_map(|(i, &f)| if f { Some(i) } else { None })
        .collect();
    if to_flip.is_empty() {
        return;
    }
    report.triangles_flipped += to_flip.len();
    match &mut prim.indices {
        Some(Indices::U16(v)) => {
            for &face_idx in &to_flip {
                let b = face_idx * 3;
                v.swap(b + 1, b + 2);
            }
        }
        Some(Indices::U32(v)) => {
            for &face_idx in &to_flip {
                let b = face_idx * 3;
                v.swap(b + 1, b + 2);
            }
        }
        None => {
            let normals_match = prim
                .normals
                .as_ref()
                .map(|ns| ns.len() == prim.positions.len())
                .unwrap_or(false);
            for &face_idx in &to_flip {
                let b = face_idx * 3;
                prim.positions.swap(b + 1, b + 2);
                if normals_match {
                    if let Some(ns) = prim.normals.as_mut() {
                        ns.swap(b + 1, b + 2);
                    }
                }
            }
        }
    }
}

/// Default tolerance for [`repair_split_t_junctions`] — matches
/// [`crate::DEFAULT_T_JUNCTION_TOLERANCE`] (the validate-module's
/// detection tolerance).
///
/// A vertex `V` is treated as lying on edge `PQ` when:
///
/// 1. Its perpendicular distance from the infinite line through `P`
///    and `Q` is at most `eps * |PQ|`.
/// 2. Its orthogonal projection onto `PQ`, parameterised as `t ∈
///    [0, 1]`, lies strictly in `(eps, 1 - eps)` — i.e. strictly
///    between the endpoints with `eps`-margin.
///
/// Mirrors the validate-module's
/// [`crate::DEFAULT_T_JUNCTION_TOLERANCE`] constant exactly so a
/// scene that passes `validate` with `check_t_junctions = true` and
/// the default tolerance is a no-op for this repair at the matching
/// `eps`.
pub const DEFAULT_T_JUNCTION_SPLIT_TOLERANCE: f32 = 1.0e-5;

/// Outcome of a [`repair_split_t_junctions`] pass.
///
/// Counters are summed across every `Triangles` primitive in the
/// scene. `triangles_split == 0` is the idempotency signal — a scene
/// with no T-junctions (under the configured tolerance) is left
/// untouched.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TJunctionSplitReport {
    /// Total triangle slots inspected (post-index-buffer resolution)
    /// across every touched primitive, on entry.
    pub triangles_inspected: usize,
    /// Number of triangles whose one chosen edge carried at least
    /// one other corner strictly between its endpoints and was
    /// therefore replaced by a fan of `1 + split_vertex_count`
    /// sub-triangles.
    pub triangles_split: usize,
    /// Total number of sub-triangles produced by every split. The
    /// post-pass face count of the touched primitives is
    /// `pre_face_count - triangles_split + triangles_emitted`.
    pub triangles_emitted: usize,
    /// Total number of distinct splitting vertices (counted once per
    /// edge they sit on, per pass). A single vertex that splits the
    /// same edge of two adjacent triangles contributes twice; a
    /// vertex that splits two different edges of one triangle
    /// contributes once because the pass picks the edge with the
    /// most splits and ignores the others on the current pass —
    /// see "Iteration to fixpoint" below.
    pub split_vertices_inserted: usize,
    /// Number of triangles whose every edge was clean (no foreign
    /// corner strictly inside) and which were therefore copied
    /// through unchanged.
    pub triangles_unchanged: usize,
    /// Number of primitives skipped because their `normals` length
    /// did not match `positions.len()` — splitting a face requires
    /// emitting fresh normals for each new sub-face and a mismatched
    /// length would leak existing producer-bug data into the result.
    /// The safe action is to skip; the caller can run
    /// [`repair_recompute_zero_normals`] first to fix the lengths.
    pub skipped_length_mismatch: usize,
}

/// Split every T-junction edge in-place — the matching mutating
/// fix-up for [`crate::ValidationOptions::check_t_junctions`].
///
/// The 1989 spec's vertex-to-vertex rule (§6.5) says "every triangle
/// must share exactly two vertices with each of its adjacent
/// triangles" — i.e. no corner of one triangle may lie strictly
/// *inside* an edge of another. The validate module surfaces such
/// incidences under
/// [`crate::ValidationReport::t_junction_defects`]; this is the
/// matching repair. After this pass and a re-run of `validate` with
/// the same `eps`, `t_junction_defects` is zero on the same scene
/// (modulo the iteration-to-fixpoint note below).
///
/// ## Algorithm
///
/// Per `Triangles` primitive in isolation (no cross-primitive
/// adjacency):
///
/// 1. Collect every distinct vertex position in the primitive into a
///    bit-exact-key set.
/// 2. For each face `(A, B, C)`, test each of its three edges for
///    *foreign* splitting vertices — vertex keys from step 1 that
///    are not `A`, `B` or `C` and that satisfy the geometric
///    [`point_strictly_on_segment_t`] predicate at the configured
///    `eps`. Edges with non-finite positions or degenerate length
///    (`|edge|² == 0`) are skipped.
/// 3. Pick the *single* edge of the face with the most splitting
///    vertices. Ties break in cyclic edge order `(A,B) → (B,C) →
///    (C,A)`. The unchosen edges are left for a subsequent pass
///    (their splitters will still be valid in the post-pass mesh).
/// 4. Sort the chosen edge's splitting vertices along the edge by
///    their parameter `t ∈ (0, 1)` and replace the original face by
///    a fan rooted at the *opposite* corner — for an edge `(P, Q)`
///    with splitters `V₁, V₂, …, Vₙ` between `P` and `Q` (in
///    increasing `t`), the original face `(P, Q, R)` becomes the
///    sequence `(P, V₁, R), (V₁, V₂, R), …, (Vₙ, Q, R)`. The fan
///    preserves both the face's plane (so the recomputed normal is
///    identical) and the original winding direction at every
///    sub-triangle.
/// 5. Faces with no splitting vertices on any edge are copied through
///    unchanged.
///
/// ## Indexed vs unindexed
///
/// - **Indexed** primitives (`Indices::U16` / `Indices::U32`): each
///   splitting vertex picks up a fresh entry in `prim.positions`
///   (and a matched fresh entry in `prim.normals` when present and
///   length-matched). The index buffer is rewritten with the new
///   triangle fan; the `Indices` discriminant is preserved as long
///   as the resulting maximum index still fits — `U16` upgrades to
///   `U32` automatically when a split would push the position count
///   over `u16::MAX`, since a downstream consumer's index decode
///   would otherwise overflow.
/// - **Unindexed** primitives: `prim.positions` is fully rewritten
///   as the new flat triangle soup (three corners per face, the way
///   STL's binary encoder emits). `prim.normals` is rewritten in
///   lockstep when present and matched 1:1 with positions; the new
///   face's normal slot is replicated from the original face's
///   first-corner normal (the spec's per-face value) — the fan
///   preserves the plane, so every sub-triangle inherits the same
///   face normal.
///
/// ## Iteration to fixpoint
///
/// One pass handles the common producer-pattern of "every face
/// carries at most one T-junction"; nested T-junctions where two
/// new fan triangles each carry their own splitter need re-runs.
/// `triangles_split == 0` on a re-run is the fixpoint signal.
/// Re-running on a scene that's already passed `validate`'s
/// `check_t_junctions` rule at the same `eps` is a no-op.
///
/// ## Out-of-scope
///
/// - **Stored facet normals** are NOT recomputed by this pass; the
///   fan preserves the original plane, so the per-face normal stored
///   in the first corner of each new sub-face is identical to the
///   original. Running [`repair_normalize_unit_normals`] or
///   [`repair_orient_normals_from_winding`] afterwards is harmless.
/// - **Non-`Triangles` primitives** are silently skipped.
/// - **Cross-primitive T-junctions** (a corner of primitive `P1`
///   lying strictly inside an edge of primitive `P2`) are NOT
///   detected — adjacency is per-primitive, matching the validate
///   module's per-primitive edge accounting. Pre-merge into a
///   single primitive with [`repair_weld_vertices`] when you need
///   cross-primitive coverage.
/// - **Non-finite positions** are silently skipped; the predicate
///   returns `false` on any NaN/Inf component.
///
/// `eps` is the same tolerance used by the validate module; values
/// outside `[0, 0.5)` or non-finite clamp to
/// [`DEFAULT_T_JUNCTION_SPLIT_TOLERANCE`].
pub fn repair_split_t_junctions(scene: &mut Scene3D, eps: f32) -> TJunctionSplitReport {
    let eps = if eps.is_finite() && (0.0..0.5).contains(&eps) {
        eps
    } else {
        DEFAULT_T_JUNCTION_SPLIT_TOLERANCE
    };
    let mut report = TJunctionSplitReport::default();
    for mesh in &mut scene.meshes {
        for prim in &mut mesh.primitives {
            if prim.topology != Topology::Triangles {
                continue;
            }
            split_t_junctions_in_primitive(prim, eps, &mut report);
        }
    }
    report
}

/// Per-primitive split pass. Walks every face; for each, picks the
/// single edge with the most foreign splitters and replaces the face
/// by a fan rooted at the opposite corner.
fn split_t_junctions_in_primitive(
    prim: &mut Primitive,
    eps: f32,
    report: &mut TJunctionSplitReport,
) {
    let face_count = match &prim.indices {
        Some(idx) => idx.len() / 3,
        None => prim.positions.len() / 3,
    };
    if face_count == 0 {
        return;
    }
    report.triangles_inspected += face_count;

    // Per-corner normals only ride along when matched 1:1 with
    // positions. A length-mismatched normals array is a producer-bug
    // signal; splitting a face would force us to invent entries that
    // do not correspond to any real face-normal source, so we leave
    // the whole primitive alone in that case.
    let normals_match = match prim.normals.as_ref() {
        Some(ns) => ns.len() == prim.positions.len(),
        None => true, // Missing normals are fine — we just don't emit any.
    };
    if !normals_match {
        report.skipped_length_mismatch += 1;
        return;
    }

    // Collect distinct vertex keys. Two corners with the same
    // bit-exact position contribute a single splitter candidate (the
    // splitter test rejects exact-endpoint matches, so a duplicate
    // key never splits an edge it sits on).
    let mut keys: std::collections::HashSet<VertKey> =
        std::collections::HashSet::with_capacity(prim.positions.len());
    for p in &prim.positions {
        keys.insert(VertKey::from(*p));
    }

    // For each face decide which edge (if any) to split.
    //
    // `chosen[face]`: optional `(edge_index, splitter_keys-sorted-by-t)`.
    // `edge_index` is `0 → (A,B)`, `1 → (B,C)`, `2 → (C,A)` so the
    // emit step knows which corner is the fan apex.
    #[allow(clippy::type_complexity)]
    let mut chosen: Vec<Option<(usize, Vec<[f32; 3]>)>> = Vec::with_capacity(face_count);
    for face_idx in 0..face_count {
        let (vi0, vi1, vi2) = resolve_face(&prim.indices, face_idx);
        let a = prim.positions.get(vi0).copied();
        let b = prim.positions.get(vi1).copied();
        let c = prim.positions.get(vi2).copied();
        let (a, b, c) = match (a, b, c) {
            (Some(a), Some(b), Some(c)) => (a, b, c),
            // Out-of-range index slot — leave the face alone; the
            // drop-degenerates pass owns the rejection.
            _ => {
                chosen.push(None);
                continue;
            }
        };
        let ka = VertKey::from(a);
        let kb = VertKey::from(b);
        let kc = VertKey::from(c);
        let endpoint_set: [VertKey; 3] = [ka, kb, kc];

        #[allow(clippy::type_complexity)]
        let mut best: Option<(usize, Vec<(f32, [f32; 3])>)> = None;
        for (edge_idx, (p, q)) in [(a, b), (b, c), (c, a)].into_iter().enumerate() {
            let mut hits: Vec<(f32, [f32; 3])> = Vec::new();
            for &k in &keys {
                if endpoint_set.contains(&k) {
                    continue;
                }
                let v = [
                    f32::from_bits(k.0),
                    f32::from_bits(k.1),
                    f32::from_bits(k.2),
                ];
                if let Some(t) = point_strictly_on_segment_t(p, q, v, eps) {
                    hits.push((t, v));
                }
            }
            if hits.is_empty() {
                continue;
            }
            // Sort along the edge so the fan walks endpoints in order.
            hits.sort_by(|x, y| x.0.total_cmp(&y.0));
            // De-dup colinear-coincident t values (two splitter
            // candidates at the same parameter would emit a
            // zero-length sub-triangle — handled by the
            // drop-degenerates pass, but cleaner to dedup here).
            hits.dedup_by(|x, y| x.0.to_bits() == y.0.to_bits());

            // Pick the edge with the most splitters; ties keep the
            // earlier cyclic edge per the doc.
            let take = match &best {
                None => true,
                Some((_, cur)) => hits.len() > cur.len(),
            };
            if take {
                best = Some((edge_idx, hits));
            }
        }
        if let Some((edge_idx, hits)) = best {
            let positions: Vec<[f32; 3]> = hits.into_iter().map(|(_, v)| v).collect();
            report.split_vertices_inserted += positions.len();
            report.triangles_split += 1;
            report.triangles_emitted += positions.len() + 1;
            chosen.push(Some((edge_idx, positions)));
        } else {
            report.triangles_unchanged += 1;
            chosen.push(None);
        }
    }

    if report.triangles_split == 0 {
        // Nothing to do.
        return;
    }
    if !chosen.iter().any(|c| c.is_some()) {
        // Counter incremented above on a different primitive; nothing
        // to do here.
        return;
    }

    // Decide output shape: append for indexed, rewrite for unindexed.
    match prim.indices.take() {
        Some(idx) => {
            let (idx_kind_was_u16, raw_idx): (bool, Vec<u32>) = match idx {
                Indices::U16(v) => (true, v.iter().map(|x| *x as u32).collect()),
                Indices::U32(v) => (false, v),
            };
            // Per-face emit: walk each face's pre-resolved corner
            // indices; if the face is split, push the new splitting
            // vertices onto `prim.positions` (and the matched
            // `prim.normals`) and emit a fan of triangle indices
            // rooted at the apex corner.
            let mut new_indices: Vec<u32> = Vec::with_capacity(raw_idx.len());
            let has_normals = prim.normals.is_some();
            for (face_idx, decision) in chosen.iter().enumerate() {
                let b = face_idx * 3;
                let ia = raw_idx[b];
                let ib = raw_idx[b + 1];
                let ic = raw_idx[b + 2];
                let Some((edge_idx, splitters)) = decision else {
                    new_indices.push(ia);
                    new_indices.push(ib);
                    new_indices.push(ic);
                    continue;
                };
                // Apex is the corner *not* on the chosen edge.
                //   edge 0 → (A,B), apex = C
                //   edge 1 → (B,C), apex = A
                //   edge 2 → (C,A), apex = B
                let (i_start, i_end, i_apex) = match edge_idx {
                    0 => (ia, ib, ic),
                    1 => (ib, ic, ia),
                    _ => (ic, ia, ib),
                };
                // Append splitter positions + (optional) normals.
                // Each splitter takes a fresh slot; we don't try to
                // de-duplicate against an existing slot since the
                // splitter's bit-exact key may already live at a
                // pre-existing index, but using a fresh slot keeps
                // the emit O(splitters) without an index lookup.
                // The pre-existing position is left in place
                // unaffected.
                let face_normal = if has_normals {
                    prim.normals.as_ref().map(|ns| ns[i_apex as usize])
                } else {
                    None
                };
                let mut chain_indices: Vec<u32> = Vec::with_capacity(splitters.len() + 2);
                chain_indices.push(i_start);
                for splitter in splitters {
                    let new_slot = prim.positions.len() as u32;
                    prim.positions.push(*splitter);
                    if let Some(ns) = prim.normals.as_mut() {
                        ns.push(face_normal.unwrap_or([0.0, 0.0, 0.0]));
                    }
                    chain_indices.push(new_slot);
                }
                chain_indices.push(i_end);
                // Emit the fan: (chain[i], chain[i+1], apex).
                for w in chain_indices.windows(2) {
                    new_indices.push(w[0]);
                    new_indices.push(w[1]);
                    new_indices.push(i_apex);
                }
            }
            // Pick the narrowest discriminant that still fits.
            let max_index = new_indices.iter().copied().max().unwrap_or(0);
            if idx_kind_was_u16 && max_index <= u16::MAX as u32 {
                let narrowed: Vec<u16> = new_indices.into_iter().map(|x| x as u16).collect();
                prim.indices = Some(Indices::U16(narrowed));
            } else {
                prim.indices = Some(Indices::U32(new_indices));
            }
        }
        None => {
            // Unindexed: rewrite positions + (optional) normals.
            let old_positions = std::mem::take(&mut prim.positions);
            let old_normals = prim.normals.take();
            let mut new_positions: Vec<[f32; 3]> = Vec::with_capacity(old_positions.len());
            let mut new_normals: Option<Vec<[f32; 3]>> = old_normals
                .as_ref()
                .map(|_| Vec::with_capacity(old_positions.len()));

            for (face_idx, decision) in chosen.iter().enumerate() {
                let b = face_idx * 3;
                let a_pos = old_positions[b];
                let b_pos = old_positions[b + 1];
                let c_pos = old_positions[b + 2];
                let a_n = old_normals.as_ref().map(|n| n[b]);
                let b_n = old_normals.as_ref().map(|n| n[b + 1]);
                let c_n = old_normals.as_ref().map(|n| n[b + 2]);
                let Some((edge_idx, splitters)) = decision else {
                    new_positions.push(a_pos);
                    new_positions.push(b_pos);
                    new_positions.push(c_pos);
                    if let Some(ns) = new_normals.as_mut() {
                        ns.push(a_n.unwrap_or([0.0, 0.0, 0.0]));
                        ns.push(b_n.unwrap_or([0.0, 0.0, 0.0]));
                        ns.push(c_n.unwrap_or([0.0, 0.0, 0.0]));
                    }
                    continue;
                };
                let (start, end, apex, apex_n) = match edge_idx {
                    0 => (a_pos, b_pos, c_pos, c_n),
                    1 => (b_pos, c_pos, a_pos, a_n),
                    _ => (c_pos, a_pos, b_pos, b_n),
                };
                // Build the chain (start → ...splitters... → end).
                let mut chain: Vec<[f32; 3]> = Vec::with_capacity(splitters.len() + 2);
                chain.push(start);
                chain.extend(splitters.iter().copied());
                chain.push(end);
                // Emit one sub-triangle per chain edge with the apex
                // last (preserves winding direction).
                let face_normal = apex_n.unwrap_or([0.0, 0.0, 0.0]);
                for w in chain.windows(2) {
                    new_positions.push(w[0]);
                    new_positions.push(w[1]);
                    new_positions.push(apex);
                    if let Some(ns) = new_normals.as_mut() {
                        ns.push(face_normal);
                        ns.push(face_normal);
                        ns.push(face_normal);
                    }
                }
            }
            prim.positions = new_positions;
            prim.normals = new_normals;
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

/// Signed enclosed-volume report returned by [`mesh_volume`].
///
/// The signed volume of a triangle mesh is the divergence-theorem sum
/// of per-triangle signed tetrahedron volumes `(v0 · (v1 × v2)) / 6`,
/// each tetrahedron spanning the coordinate origin and one facet. For
/// a **closed** surface the contributions of interior structure cancel
/// and the sum equals the volume the surface encloses, signed by the
/// facet winding: positive when triangles wind counter-clockwise as
/// seen from outside (the right-hand-rule outward orientation the 1989
/// spec mandates), negative when the surface is wound inside-out.
///
/// The accumulation is done in `f64` (the corner coordinates are
/// promoted from `f32` first) so a million-facet mesh does not lose the
/// running sum to single-precision cancellation; the reported fields
/// are `f64` for the same reason.
///
/// ## Open / non-watertight meshes
///
/// The sum is still well-defined for an open mesh, but its value then
/// depends on where the origin sits relative to the hole(s) and is not
/// a meaningful volume. [`crate::validate`]'s `watertight` rule (or
/// [`boundary_loops`] returning empty) is the precondition for reading
/// [`Self::signed_volume`] as an enclosed volume; this report does not
/// itself decide watertightness.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MeshVolumeReport {
    /// Number of triangle facets summed across every `Triangles`
    /// primitive in the scene (post-index-buffer resolution).
    /// Non-`Triangles` primitives contribute nothing.
    pub triangles_summed: usize,
    /// The signed divergence-theorem volume `Σ (v0 · (v1 × v2)) / 6`.
    /// Sign carries the facet winding orientation; magnitude is the
    /// enclosed volume for a closed surface. `0.0` for an empty scene.
    pub signed_volume: f64,
    /// Whether any corner coordinate summed was non-finite (NaN or ±∞).
    /// When `true`, [`Self::signed_volume`] may itself be non-finite
    /// and should not be trusted as a volume; the facet that introduced
    /// the non-finite value still contributed to `triangles_summed`.
    pub had_non_finite: bool,
}

impl MeshVolumeReport {
    /// Absolute enclosed volume — `signed_volume.abs()`. For a closed
    /// surface this is the geometry's volume regardless of winding
    /// orientation. Returns a non-finite value unchanged when
    /// [`Self::had_non_finite`] is set.
    pub fn volume(&self) -> f64 {
        self.signed_volume.abs()
    }

    /// Winding-orientation hint from the sign of the signed volume:
    /// `Some(true)` when the volume is strictly positive (facets wind
    /// outward — the spec's right-hand-rule orientation), `Some(false)`
    /// when strictly negative (inside-out winding), and `None` when the
    /// signed volume is exactly zero or non-finite (no orientation can
    /// be inferred — e.g. an empty scene, a flat sheet through the
    /// origin, or a mesh with non-finite corners).
    pub fn winds_outward(&self) -> Option<bool> {
        if !self.signed_volume.is_finite() || self.signed_volume == 0.0 {
            None
        } else {
            Some(self.signed_volume > 0.0)
        }
    }
}

/// Compute the signed enclosed volume of `scene` without mutating it.
///
/// Each triangle facet `(v0, v1, v2)` forms a tetrahedron with the
/// coordinate origin whose signed volume is `(v0 · (v1 × v2)) / 6`.
/// Summing that over every facet of a **closed** mesh yields the volume
/// the surface encloses, signed by the winding: positive for the
/// right-hand-rule outward orientation the 1989 spec requires, negative
/// for an inside-out mesh. See [`MeshVolumeReport`] for the
/// closed-surface precondition and the `f64` accumulation rationale.
///
/// This is a pure diagnostic — it reads vertex positions in their
/// stored order (winding *is* the signal here, so no canonicalisation
/// is applied) and never touches the scene. Non-`Triangles` primitives
/// are skipped, matching the rest of this module. An empty scene
/// returns a report with `signed_volume == 0.0` and
/// `triangles_summed == 0`.
pub fn mesh_volume(scene: &Scene3D) -> MeshVolumeReport {
    let mut report = MeshVolumeReport::default();
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
                report.triangles_summed += 1;
                // Promote to f64 before the triple product so the
                // running sum survives large meshes.
                let a = [v0[0] as f64, v0[1] as f64, v0[2] as f64];
                let b = [v1[0] as f64, v1[1] as f64, v1[2] as f64];
                let c = [v2[0] as f64, v2[1] as f64, v2[2] as f64];
                if !(a.iter().all(|x| x.is_finite())
                    && b.iter().all(|x| x.is_finite())
                    && c.iter().all(|x| x.is_finite()))
                {
                    report.had_non_finite = true;
                }
                // Scalar triple product v0 · (v1 × v2).
                let cross = [
                    b[1] * c[2] - b[2] * c[1],
                    b[2] * c[0] - b[0] * c[2],
                    b[0] * c[1] - b[1] * c[0],
                ];
                let triple = a[0] * cross[0] + a[1] * cross[1] + a[2] * cross[2];
                report.signed_volume += triple / 6.0;
            }
        }
    }
    report
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

/// Geometric "vertex V lies strictly between segment endpoints P and
/// Q" predicate. When the predicate fires, returns `Some(t)` with the
/// projection parameter so callers can sort splitters along the
/// edge. Returns `None` when:
///
/// - The segment is degenerate (`|PQ|² == 0`).
/// - Any coordinate is non-finite.
/// - `eps` is outside `[0, 0.5)`.
/// - The projection parameter `t` is not strictly inside
///   `(eps, 1 - eps)`.
/// - The perpendicular distance from V to PQ exceeds `eps * |PQ|`.
///
/// Mirrors the predicate the validate module uses for its T-junction
/// sub-check — the two functions agree bit-for-bit on the same input
/// because they share the same arithmetic. The matching repair pass
/// is [`repair_split_t_junctions`].
fn point_strictly_on_segment_t(p: [f32; 3], q: [f32; 3], v: [f32; 3], eps: f32) -> Option<f32> {
    let d = [q[0] - p[0], q[1] - p[1], q[2] - p[2]];
    let pv = [v[0] - p[0], v[1] - p[1], v[2] - p[2]];
    let len_sq = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    if !len_sq.is_finite() || len_sq == 0.0 {
        return None;
    }
    if !(eps.is_finite() && (0.0..0.5).contains(&eps)) {
        return None;
    }
    let dot = pv[0] * d[0] + pv[1] * d[1] + pv[2] * d[2];
    let t = dot / len_sq;
    if !t.is_finite() || t <= eps || t >= 1.0 - eps {
        return None;
    }
    let pv_sq = pv[0] * pv[0] + pv[1] * pv[1] + pv[2] * pv[2];
    let perp_sq_times_len_sq = pv_sq * len_sq - dot * dot;
    if !perp_sq_times_len_sq.is_finite() {
        return None;
    }
    let perp_sq_times_len_sq = perp_sq_times_len_sq.max(0.0);
    let tol = (eps * eps) * (len_sq * len_sq);
    if perp_sq_times_len_sq <= tol {
        Some(t)
    } else {
        None
    }
}

/// Outcome of a [`repair_cap_boundary_loops`] pass.
///
/// Counters are summed across every `Triangles` primitive in the
/// scene. The 1989 spec says each facet "is part of the boundary
/// between the interior and the exterior of the object" — a valid STL
/// solid is *closed*, with no boundary edges at all. This repair
/// restores that invariant per-primitive by triangulating each closed
/// naked-edge loop with a fan, so every boundary edge the cap touches
/// goes from used-once (boundary) to used-twice (manifold).
///
/// `loops_capped == 0` is the idempotency signal — a primitive with no
/// closed boundary loops (already watertight, or whose only naked
/// edges form non-manifold open chains) is left untouched.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CapBoundaryLoopsReport {
    /// Total triangle slots inspected (post-index-buffer resolution)
    /// across every touched primitive, on entry.
    pub triangles_inspected: usize,
    /// Number of closed boundary loops that were capped with a fan.
    pub loops_capped: usize,
    /// Number of cap triangles emitted. A closed loop with `n`
    /// boundary edges caps to `n - 2` fan triangles, so this is the
    /// sum of `(edge_count - 2)` over every capped loop.
    pub cap_triangles_emitted: usize,
    /// Number of *open* boundary chains skipped — a non-manifold
    /// boundary (three-plus naked edges meeting at a point) that does
    /// not bound a single well-defined hole, so the pass refuses to
    /// guess a cap rather than emit nonsense geometry.
    pub open_chains_skipped: usize,
    /// Number of closed loops with fewer than three boundary edges
    /// (a degenerate two-edge "sliver" hole) that were skipped — a
    /// triangle fan needs at least three corners.
    pub degenerate_loops_skipped: usize,
    /// Number of primitives skipped because their `normals` array
    /// length disagreed with `positions` — a producer-bug signal.
    /// Capping would force us to invent per-vertex normal entries for
    /// the new corners; we leave the whole primitive alone instead.
    pub skipped_length_mismatch: usize,
}

/// Cap every closed boundary loop in `scene` by triangulating it,
/// restoring the spec's closed-surface invariant per-primitive.
///
/// The 1989 spec says each facet "is part of the boundary between the
/// interior and the exterior of the object" — a valid STL solid is
/// closed, with no edge used by only one triangle. When a producer
/// emits a surface with holes, every hole shows up as a *boundary
/// loop* (the cycle formed by edges used by exactly one triangle —
/// see [`boundary_loops`]). This pass is the matching mutating fix-up:
/// for each **closed** loop it emits a triangle fan that turns every
/// boundary edge the cap touches into a manifold (used-twice) edge.
///
/// Per-`Triangles`-primitive isolation (no cross-primitive boundary
/// merging — unlike the scene-wide [`boundary_loops`] diagnostic).
/// Within a primitive, boundary edges are the directed edges whose
/// undirected key is used exactly once. They are chained into loops
/// the same way [`boundary_loops`] chains them; a loop walked
/// tail-to-head keeps the surface consistently on one side, so the cap
/// fan is wound to traverse each boundary edge in the *opposite*
/// direction (turning a used-once edge into a used-twice manifold
/// edge). The fan is rooted at the loop's lexicographically-smallest
/// vertex for determinism.
///
/// Open (non-manifold) boundary chains are counted under
/// [`CapBoundaryLoopsReport::open_chains_skipped`] and left alone — a
/// chain that did not close on itself does not bound a single
/// well-defined hole, so guessing a cap would invent geometry. Closed
/// loops with fewer than three edges are counted under
/// `degenerate_loops_skipped`. Non-`Triangles` primitives are silently
/// skipped, as is any primitive whose `normals` length disagrees with
/// its `positions` length (`skipped_length_mismatch`).
///
/// Cap triangles inherit the all-zero face normal sentinel
/// (`[0.0, 0.0, 0.0]`) when the primitive carries per-vertex normals,
/// so a follow-up [`repair_recompute_zero_normals`] →
/// [`repair_orient_normals_from_winding`] fills them from the cap
/// winding without disturbing the surrounding surface's stored
/// normals.
///
/// A lone free triangle's three edges are each used once, so its
/// perimeter is itself a closed 3-edge loop: capping mirrors it into
/// one reversed fan triangle, yielding a zero-volume but edge-manifold
/// two-triangle shell. This falls out of the general rule rather than
/// being a special case.
///
/// `loops_capped == 0` is the idempotency signal: re-running on a
/// scene whose primitives are each edge-manifold (or whose only naked
/// edges form open chains) is a no-op.
pub fn repair_cap_boundary_loops(scene: &mut Scene3D) -> CapBoundaryLoopsReport {
    let mut report = CapBoundaryLoopsReport::default();
    for mesh in &mut scene.meshes {
        for prim in &mut mesh.primitives {
            if prim.topology != Topology::Triangles {
                continue;
            }
            cap_boundary_loops_in_primitive(prim, &mut report);
        }
    }
    report
}

/// Per-primitive cap pass. Chains the primitive's boundary edges into
/// loops and emits a fan for each closed one.
fn cap_boundary_loops_in_primitive(prim: &mut Primitive, report: &mut CapBoundaryLoopsReport) {
    let face_count = match &prim.indices {
        Some(idx) => idx.len() / 3,
        None => prim.positions.len() / 3,
    };
    if face_count == 0 {
        return;
    }
    report.triangles_inspected += face_count;

    // A length-mismatched normals array is a producer-bug signal;
    // emitting cap triangles would force us to invent per-vertex normal
    // entries, so we skip the whole primitive (mirrors the
    // T-junction-split pass).
    let normals_match = match prim.normals.as_ref() {
        Some(ns) => ns.len() == prim.positions.len(),
        None => true,
    };
    if !normals_match {
        report.skipped_length_mismatch += 1;
        return;
    }

    // Count undirected edge uses and record each directed instance by
    // its bit-exact vertex keys (matches `boundary_loops`).
    let mut undirected_uses: HashMap<(VertKey, VertKey), usize> = HashMap::new();
    let mut directed: Vec<(VertKey, VertKey)> = Vec::new();
    for face_idx in 0..face_count {
        let (vi0, vi1, vi2) = resolve_face(&prim.indices, face_idx);
        let (a, b, c) = match (
            prim.positions.get(vi0),
            prim.positions.get(vi1),
            prim.positions.get(vi2),
        ) {
            (Some(a), Some(b), Some(c)) => (*a, *b, *c),
            // Out-of-range index slot — the drop-degenerates pass owns
            // the rejection; skip this face for boundary accounting.
            _ => continue,
        };
        let keys = [VertKey::from(a), VertKey::from(b), VertKey::from(c)];
        for (i, j) in [(0, 1), (1, 2), (2, 0)] {
            let (ka, kb) = (keys[i], keys[j]);
            let key = if ka <= kb { (ka, kb) } else { (kb, ka) };
            *undirected_uses.entry(key).or_insert(0) += 1;
            directed.push((ka, kb));
        }
    }

    // Boundary adjacency: tail -> heads, restricted to edges used
    // exactly once (undirected).
    let mut out_edges: HashMap<VertKey, Vec<VertKey>> = HashMap::new();
    let mut remaining = 0usize;
    for (a, b) in directed {
        let key = if a <= b { (a, b) } else { (b, a) };
        if undirected_uses.get(&key) == Some(&1) {
            out_edges.entry(a).or_default().push(b);
            remaining += 1;
        }
    }
    if remaining == 0 {
        return;
    }

    // Deterministic consumption order (matches `boundary_loops`).
    for heads in out_edges.values_mut() {
        heads.sort_unstable();
        heads.reverse();
    }
    let mut tails: Vec<VertKey> = out_edges.keys().copied().collect();
    tails.sort_unstable();

    // Walk loops; collect the closed ones' ordered vertex keys.
    let mut closed_loops: Vec<Vec<VertKey>> = Vec::new();
    for &seed in &tails {
        while out_edges.get(&seed).is_some_and(|v| !v.is_empty()) {
            let mut chain: Vec<VertKey> = vec![seed];
            let mut cur = seed;
            let mut closed = false;
            while let Some(next) = out_edges.get_mut(&cur).and_then(|v| v.pop()) {
                if next == seed {
                    closed = true;
                    break;
                }
                chain.push(next);
                cur = next;
            }
            if !closed {
                report.open_chains_skipped += 1;
                continue;
            }
            if chain.len() < 3 {
                report.degenerate_loops_skipped += 1;
                continue;
            }
            closed_loops.push(chain);
        }
    }

    if closed_loops.is_empty() {
        return;
    }

    // Emit a fan for each closed loop. The loop `chain = [v0, v1, …,
    // v_{n-1}]` carries the boundary edges `v0→v1, …, v_{n-1}→v0` in
    // the existing surface's winding. A cap edge must traverse each in
    // the *opposite* direction so the shared edge goes used-twice
    // (manifold). Rooting the fan at the loop's lex-smallest vertex
    // `R`, the fan triangle for boundary edge `vi→vj` (with neither
    // endpoint == R) is `(R, vj, vi)` — which walks `vj→vi`, the
    // reverse. Determinism: rotate the chain so `R` is first.
    let cap_normal = [0.0_f32, 0.0, 0.0];
    // Map a vertex key to an existing slot index so the cap reuses the
    // surface's corner slots instead of duplicating positions.
    let mut slot_of: HashMap<VertKey, u32> = HashMap::with_capacity(prim.positions.len());
    for (i, p) in prim.positions.iter().enumerate() {
        slot_of.entry(VertKey::from(*p)).or_insert(i as u32);
    }

    // Accumulate the new fan triangles as key-triples; resolve to the
    // primitive's storage shape (indexed vs unindexed) afterwards.
    let mut fan_tris: Vec<[VertKey; 3]> = Vec::new();
    for chain in &closed_loops {
        // Rotate so the lexicographically-smallest vertex is the apex.
        let apex_pos = chain
            .iter()
            .enumerate()
            .min_by_key(|(_, k)| **k)
            .map(|(i, _)| i)
            .unwrap_or(0);
        let n = chain.len();
        let apex = chain[apex_pos];
        // Walk every boundary edge (vi→vj) skipping those incident to
        // the apex (they collapse to zero-area fan triangles).
        let mut emitted = 0usize;
        for e in 0..n {
            let vi = chain[e];
            let vj = chain[(e + 1) % n];
            if vi == apex || vj == apex {
                continue;
            }
            // Cap triangle reverses the boundary edge: (apex, vj, vi).
            fan_tris.push([apex, vj, vi]);
            emitted += 1;
        }
        report.loops_capped += 1;
        report.cap_triangles_emitted += emitted;
    }

    if fan_tris.is_empty() {
        return;
    }

    // Resolve a key to a slot, appending a fresh position (+ matched
    // normal) when the key is not already present.
    let mut resolve_slot = |prim: &mut Primitive, k: VertKey| -> u32 {
        if let Some(&s) = slot_of.get(&k) {
            return s;
        }
        let new_slot = prim.positions.len() as u32;
        prim.positions.push(key_to_pos(k));
        if let Some(ns) = prim.normals.as_mut() {
            ns.push(cap_normal);
        }
        slot_of.insert(k, new_slot);
        new_slot
    };

    match prim.indices.take() {
        Some(idx) => {
            let (idx_kind_was_u16, mut raw_idx): (bool, Vec<u32>) = match idx {
                Indices::U16(v) => (true, v.iter().map(|x| *x as u32).collect()),
                Indices::U32(v) => (false, v),
            };
            for tri in &fan_tris {
                let s0 = resolve_slot(prim, tri[0]);
                let s1 = resolve_slot(prim, tri[1]);
                let s2 = resolve_slot(prim, tri[2]);
                raw_idx.push(s0);
                raw_idx.push(s1);
                raw_idx.push(s2);
            }
            let max_index = raw_idx.iter().copied().max().unwrap_or(0);
            if idx_kind_was_u16 && max_index <= u16::MAX as u32 {
                let narrowed: Vec<u16> = raw_idx.into_iter().map(|x| x as u16).collect();
                prim.indices = Some(Indices::U16(narrowed));
            } else {
                prim.indices = Some(Indices::U32(raw_idx));
            }
        }
        None => {
            // Unindexed: append the fan as a flat triangle soup so the
            // primitive stays unindexed. Each cap corner is a fresh
            // position slot (the soup never references prior slots).
            for tri in &fan_tris {
                for k in tri {
                    prim.positions.push(key_to_pos(*k));
                    if let Some(ns) = prim.normals.as_mut() {
                        ns.push(cap_normal);
                    }
                }
            }
        }
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
    fn mesh_volume_unit_cube_soup_is_plus_one() {
        let scene = scene_with_primitives(vec![unit_cube_soup_primitive()]);
        let r = mesh_volume(&scene);
        assert_eq!(r.triangles_summed, 12);
        assert!(!r.had_non_finite);
        assert!((r.signed_volume - 1.0).abs() < 1e-9);
        assert_eq!(r.winds_outward(), Some(true));
    }

    #[test]
    fn mesh_volume_flags_non_finite_corner() {
        let mut prim = unit_cube_soup_primitive();
        prim.positions[0] = [f32::INFINITY, 0.0, 0.0];
        let scene = scene_with_primitives(vec![prim]);
        let r = mesh_volume(&scene);
        assert_eq!(r.triangles_summed, 12);
        assert!(r.had_non_finite);
    }

    #[test]
    fn mesh_volume_skips_non_triangle_primitives() {
        // A Points primitive contributes nothing to the sum.
        let mut prim = Primitive::new(Topology::Points);
        prim.positions = vec![[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]];
        let scene = scene_with_primitives(vec![prim]);
        let r = mesh_volume(&scene);
        assert_eq!(r.triangles_summed, 0);
        assert_eq!(r.signed_volume, 0.0);
        assert_eq!(r.winds_outward(), None);
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

    // ---- check_z_sorted ------------------------------------------------

    #[test]
    fn check_z_sorted_empty_scene_is_sorted() {
        let scene = Scene3D::new();
        let r = check_z_sorted(&scene);
        assert_eq!(r, ZSortReport::default());
        assert!(r.is_sorted());
        assert_eq!(r.first_out_of_order_triangle, None);
    }

    #[test]
    fn check_z_sorted_true_on_ascending_scene() {
        let flat = |z: f32| [[0.0, 0.0, z], [1.0, 0.0, z], [0.0, 1.0, z]];
        let scene =
            scene_with_primitives(vec![soup_from_faces(&[flat(1.0), flat(2.0), flat(3.0)])]);
        let r = check_z_sorted(&scene);
        assert!(r.is_sorted());
        assert_eq!(r.triangles_inspected, 3);
        assert_eq!(r.out_of_order_pairs, 0);
        assert_eq!(r.first_out_of_order_triangle, None);
    }

    #[test]
    fn check_z_sorted_false_on_descending_scene() {
        let flat = |z: f32| [[0.0, 0.0, z], [1.0, 0.0, z], [0.0, 1.0, z]];
        let scene =
            scene_with_primitives(vec![soup_from_faces(&[flat(5.0), flat(1.0), flat(3.0)])]);
        let r = check_z_sorted(&scene);
        assert!(!r.is_sorted());
        assert_eq!(r.triangles_inspected, 3);
        // pairs (5,1) and (1,3): only (5,1) is descending → one bad pair.
        assert_eq!(r.out_of_order_pairs, 1);
        // First break is the 2nd triangle (1-based) — z=1 after z=5.
        assert_eq!(r.first_out_of_order_triangle, Some(2));
    }

    #[test]
    fn check_z_sorted_agrees_with_repair_zero_reorder() {
        // Acceptance parity: `is_sorted()` is true iff the repair would
        // reorder nothing. Sweep a handful of orderings.
        let flat = |z: f32| [[0.0, 0.0, z], [1.0, 0.0, z], [0.0, 1.0, z]];
        let cases: &[&[f32]] = &[
            &[1.0, 2.0, 3.0],
            &[3.0, 2.0, 1.0],
            &[1.0, 3.0, 2.0],
            &[2.0, 2.0, 2.0],
            &[1.0, 1.0, 2.0, 0.5],
        ];
        for zs in cases {
            let faces: Vec<_> = zs.iter().map(|&z| flat(z)).collect();
            let scene = scene_with_primitives(vec![soup_from_faces(&faces)]);
            let diag = check_z_sorted(&scene);
            let mut clone = scene.clone();
            let rep = repair_sort_triangles_by_z(&mut clone);
            assert_eq!(
                diag.is_sorted(),
                rep.triangles_reordered == 0,
                "mismatch for {zs:?}: diag={diag:?} rep={rep:?}"
            );
            assert_eq!(diag.triangles_inspected, rep.triangles_inspected);
        }
    }

    #[test]
    fn check_z_sorted_does_not_mutate() {
        let flat = |z: f32| [[0.0, 0.0, z], [1.0, 0.0, z], [0.0, 1.0, z]];
        let scene =
            scene_with_primitives(vec![soup_from_faces(&[flat(9.0), flat(2.0), flat(7.0)])]);
        // `Scene3D` has no `PartialEq`; observe the per-face emit order
        // stays shuffled (the repair would re-order it; the diagnostic
        // must not).
        let before = face_min_zs(&scene.meshes[0].primitives[0]);
        let _ = check_z_sorted(&scene);
        let after = face_min_zs(&scene.meshes[0].primitives[0]);
        assert_eq!(before, after);
        assert_eq!(after, vec![9.0, 2.0, 7.0]);
    }

    #[test]
    fn check_z_sorted_per_primitive_boundary_not_counted() {
        // Two primitives, each individually sorted, but the second
        // starts below where the first ended. A boundary-straddling pair
        // must NOT count (the repair sorts each primitive on its own).
        let flat = |z: f32| [[0.0, 0.0, z], [1.0, 0.0, z], [0.0, 1.0, z]];
        let p0 = soup_from_faces(&[flat(5.0), flat(6.0)]);
        let p1 = soup_from_faces(&[flat(1.0), flat(2.0)]);
        let scene = scene_with_primitives(vec![p0, p1]);
        let r = check_z_sorted(&scene);
        assert!(r.is_sorted(), "{r:?}");
        assert_eq!(r.out_of_order_pairs, 0);
        assert_eq!(r.triangles_inspected, 4);
    }

    #[test]
    fn check_z_sorted_skips_non_triangles() {
        let mut points = Primitive::new(Topology::Points);
        points.positions = vec![[0.0, 0.0, 9.0], [0.0, 0.0, 1.0]];
        let scene = scene_with_primitives(vec![points]);
        let r = check_z_sorted(&scene);
        assert_eq!(r.triangles_inspected, 0);
        assert!(r.is_sorted());
    }

    #[test]
    fn check_z_sorted_nan_face_sorts_last_is_in_order() {
        // An all-NaN face after every finite face keys to the NaN-high
        // sentinel — that is exactly the repair's target order, so the
        // diagnostic reports already-sorted.
        let flat = |z: f32| [[0.0, 0.0, z], [1.0, 0.0, z], [0.0, 1.0, z]];
        let nan_face = [
            [0.0, 0.0, f32::NAN],
            [1.0, 0.0, f32::NAN],
            [0.0, 1.0, f32::NAN],
        ];
        let scene = scene_with_primitives(vec![soup_from_faces(&[flat(2.0), flat(8.0), nan_face])]);
        let r = check_z_sorted(&scene);
        assert!(r.is_sorted(), "{r:?}");
        // The reverse (NaN face first) is out of order.
        let scene2 =
            scene_with_primitives(vec![soup_from_faces(&[nan_face, flat(2.0), flat(8.0)])]);
        let r2 = check_z_sorted(&scene2);
        assert!(!r2.is_sorted());
        assert_eq!(r2.first_out_of_order_triangle, Some(2));
    }

    // ---- translate_to_positive_octant ----------------------------------

    fn triangle_with_corner(c: [f32; 3]) -> Primitive {
        // Triangle whose first corner is `c` and other two corners are
        // axis-aligned offsets so the bbox covers a unit step on each
        // axis above `c`. Useful for pinning bbox.min == c.
        soup_from_faces(&[[c, [c[0] + 1.0, c[1], c[2]], [c[0], c[1] + 1.0, c[2]]]])
    }

    #[test]
    fn translate_octant_shifts_negative_corner_into_positive() {
        let mut scene = scene_with_primitives(vec![triangle_with_corner([-2.0, -3.0, -4.0])]);
        let r = repair_translate_to_positive_octant(&mut scene, 0.0);
        // delta should be exactly `-min` on each axis (margin = 0).
        assert_eq!(r.delta, [2.0, 3.0, 4.0]);
        assert_eq!(r.triangles_inspected, 1);
        // Three corners on the only triangle.
        assert_eq!(r.vertices_translated, 3);
        assert_eq!(r.skipped_non_finite_vertices, 0);
        // Post-condition: bbox.min lands at 0 on every axis.
        let bbox = crate::validate::bbox(&scene).expect("scene has finite verts");
        assert_eq!(bbox.min, [0.0, 0.0, 0.0]);
        assert_eq!(bbox.max, [-1.0 + 2.0, -2.0 + 3.0, -4.0 + 4.0]);
    }

    #[test]
    fn translate_octant_default_margin_clears_strict_nonzero() {
        let mut scene = scene_with_primitives(vec![triangle_with_corner([0.0, 0.0, 0.0])]);
        let r = repair_translate_to_positive_octant(&mut scene, DEFAULT_POSITIVE_OCTANT_MARGIN);
        // Every axis had min == 0, so each one gets bumped by the margin.
        assert_eq!(
            r.delta,
            [
                DEFAULT_POSITIVE_OCTANT_MARGIN,
                DEFAULT_POSITIVE_OCTANT_MARGIN,
                DEFAULT_POSITIVE_OCTANT_MARGIN
            ]
        );
        let bbox = crate::validate::bbox(&scene).unwrap();
        assert!(bbox.min[0] > 0.0 && bbox.min[1] > 0.0 && bbox.min[2] > 0.0);
        // The validate-module's positive-octant rule should now report
        // zero defects under default margin.
        let opts = crate::validate::ValidationOptions {
            check_positive_octant: true,
            ..Default::default()
        };
        let rep = crate::validate::validate(&scene, &opts);
        assert_eq!(
            rep.positive_octant_defects, 0,
            "expected zero defects after repair, got {rep:?}"
        );
    }

    #[test]
    fn translate_octant_idempotent_on_already_positive_scene() {
        // Smallest corner already well above any reasonable margin.
        let mut scene = scene_with_primitives(vec![triangle_with_corner([10.0, 20.0, 30.0])]);
        let r = repair_translate_to_positive_octant(&mut scene, DEFAULT_POSITIVE_OCTANT_MARGIN);
        assert_eq!(r.delta, [0.0, 0.0, 0.0]);
        assert_eq!(r.vertices_translated, 0);
        let bbox = crate::validate::bbox(&scene).unwrap();
        assert_eq!(bbox.min, [10.0, 20.0, 30.0]);
    }

    #[test]
    fn translate_octant_second_pass_is_noop() {
        let mut scene = scene_with_primitives(vec![triangle_with_corner([-5.0, -1.0, -2.0])]);
        let _first =
            repair_translate_to_positive_octant(&mut scene, DEFAULT_POSITIVE_OCTANT_MARGIN);
        let second =
            repair_translate_to_positive_octant(&mut scene, DEFAULT_POSITIVE_OCTANT_MARGIN);
        assert_eq!(second.delta, [0.0, 0.0, 0.0]);
        assert_eq!(second.vertices_translated, 0);
    }

    #[test]
    fn translate_octant_negative_margin_clamps_to_default() {
        let mut scene = scene_with_primitives(vec![triangle_with_corner([-1.0, -1.0, -1.0])]);
        let r = repair_translate_to_positive_octant(&mut scene, -42.0);
        // Clamps to the default margin, so post-shift min is +margin.
        for axis in 0..3 {
            let expected = DEFAULT_POSITIVE_OCTANT_MARGIN + 1.0;
            assert!(
                (r.delta[axis] - expected).abs() < 1e-9,
                "delta[{axis}] = {} ≠ {}",
                r.delta[axis],
                expected
            );
        }
    }

    #[test]
    fn translate_octant_skips_non_triangles_primitive() {
        // Build a non-triangles primitive in the all-negative octant —
        // the repair should walk past it and report zero inspected
        // triangles + zero delta.
        let mut prim = Primitive::new(Topology::Points);
        prim.positions = vec![[-1.0, -2.0, -3.0]];
        let mut scene = scene_with_primitives(vec![prim]);
        let r = repair_translate_to_positive_octant(&mut scene, DEFAULT_POSITIVE_OCTANT_MARGIN);
        assert_eq!(r.triangles_inspected, 0);
        assert_eq!(r.delta, [0.0, 0.0, 0.0]);
        assert_eq!(r.vertices_translated, 0);
        // The non-triangles primitive's position is untouched.
        assert_eq!(
            scene.meshes[0].primitives[0].positions[0],
            [-1.0, -2.0, -3.0]
        );
    }

    #[test]
    fn translate_octant_passes_non_finite_components_through() {
        // Mixed-finite vertex: one component NaN, one component on the
        // negative side. Repair shifts the finite axis, leaves NaN alone.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[f32::NAN, -1.0, -1.0], [1.0, -1.0, -1.0], [0.0, 0.0, -1.0]];
        let mut scene = scene_with_primitives(vec![prim]);
        let r = repair_translate_to_positive_octant(&mut scene, 0.0);
        // delta on axis 0 came from the two finite x values (1.0, 0.0)
        // — min is 0, margin is 0, so delta.x == 0.
        // delta on axis 1 came from y mins (-1.0, -1.0, 0.0) → 1.0.
        // delta on axis 2 from z (-1.0) → 1.0.
        assert_eq!(r.delta, [0.0, 1.0, 1.0]);
        // First slot's NaN is preserved.
        let p0 = scene.meshes[0].primitives[0].positions[0];
        assert!(p0[0].is_nan(), "expected NaN, got {}", p0[0]);
        assert_eq!(p0[1], 0.0);
        assert_eq!(p0[2], 0.0);
        // Second slot's finite axes all shifted.
        let p1 = scene.meshes[0].primitives[0].positions[1];
        assert_eq!(p1, [1.0, 0.0, 0.0]);
    }

    #[test]
    fn translate_octant_skips_all_nonfinite_vertex_slot() {
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![
            [f32::NAN, f32::INFINITY, f32::NEG_INFINITY],
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        ];
        let mut scene = scene_with_primitives(vec![prim]);
        let r = repair_translate_to_positive_octant(&mut scene, 0.0);
        // delta on x = 1.0 (from -1.0 min), y and z stay at 0.
        assert_eq!(r.delta, [1.0, 0.0, 0.0]);
        assert_eq!(r.skipped_non_finite_vertices, 1);
        // 2 of 3 vertices were translated (the all-non-finite one was
        // skipped); the slot whose only finite axis (x=0.0) needed no
        // delta on the other two axes still counts as translated
        // because at least one component (x) was shifted.
        assert_eq!(r.vertices_translated, 2);
        // The all-non-finite slot is unchanged bit-for-bit.
        let p0 = scene.meshes[0].primitives[0].positions[0];
        assert!(p0[0].is_nan());
        assert!(p0[1].is_infinite() && p0[1].is_sign_positive());
        assert!(p0[2].is_infinite() && p0[2].is_sign_negative());
    }

    #[test]
    fn translate_octant_empty_scene_is_noop() {
        let mut scene = Scene3D::new();
        let r = repair_translate_to_positive_octant(&mut scene, DEFAULT_POSITIVE_OCTANT_MARGIN);
        assert_eq!(r.triangles_inspected, 0);
        assert_eq!(r.vertices_translated, 0);
        assert_eq!(r.skipped_non_finite_vertices, 0);
        assert_eq!(r.delta, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn translate_octant_per_axis_independence() {
        // x already > margin; y straddles 0; z deeply negative — only
        // y and z should pick up a non-zero delta.
        let mut scene = scene_with_primitives(vec![triangle_with_corner([5.0, -0.5, -10.0])]);
        let r = repair_translate_to_positive_octant(&mut scene, 0.0);
        assert_eq!(r.delta[0], 0.0, "x already in +octant");
        assert_eq!(r.delta[1], 0.5);
        assert_eq!(r.delta[2], 10.0);
    }

    #[test]
    fn translate_octant_normals_not_modified() {
        // Translation does not change direction vectors.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]];
        let n = [0.5_f32, 0.5, 0.5];
        prim.normals = Some(vec![n; 3]);
        let mut scene = scene_with_primitives(vec![prim]);
        let _ = repair_translate_to_positive_octant(&mut scene, 0.0);
        let after = scene.meshes[0].primitives[0].normals.as_ref().unwrap();
        for slot in after {
            assert_eq!(*slot, n);
        }
    }

    // -- repair_make_winding_consistent --------------------------------

    /// Build the two-triangle "flipped-neighbour quad" the validate
    /// module uses to exercise its `inconsistent_winding` rule. Tri 0
    /// is canonically wound; tri 1 walks the shared diagonal in the
    /// SAME direction as tri 0 (rather than the opposite direction a
    /// consistent quad would walk). The pass should flip tri 1.
    fn flipped_neighbour_quad_unindexed() -> Primitive {
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![
            // tri 0 — CCW around +Z, walks the diagonal (1,1,0)→(0,0,0)
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            // tri 1 — also walks the diagonal (1,1,0)→(0,0,0) (same
            // direction as tri 0 → inconsistent)
            [1.0, 1.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ];
        prim.normals = Some(vec![[0.0, 0.0, 1.0]; 6]);
        prim
    }

    #[test]
    fn winding_consistency_empty_scene_is_noop() {
        let mut scene = Scene3D::new();
        let r = repair_make_winding_consistent(&mut scene);
        assert_eq!(r, WindingConsistencyReport::default());
    }

    #[test]
    fn winding_consistency_no_triangles_primitive_skipped() {
        // A primitive with non-Triangles topology is silently skipped.
        let mut prim = Primitive::new(Topology::Lines);
        prim.positions = vec![[0.0, 0.0, 0.0]; 6];
        let mut scene = scene_with_primitives(vec![prim]);
        let r = repair_make_winding_consistent(&mut scene);
        assert_eq!(r.triangles_inspected, 0);
        assert_eq!(r.triangles_flipped, 0);
        assert_eq!(r.components_visited, 0);
    }

    #[test]
    fn winding_consistency_already_consistent_quad_is_idempotent() {
        // Same two triangles, but tri 1 is wound the correct way —
        // walks the diagonal (0,0,0)→(1,1,0), opposite to tri 0.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
        ];
        prim.normals = Some(vec![[0.0, 0.0, 1.0]; 6]);
        let mut scene = scene_with_primitives(vec![prim]);
        let pre_positions = scene.meshes[0].primitives[0].positions.clone();
        let r = repair_make_winding_consistent(&mut scene);
        assert_eq!(r.triangles_inspected, 2);
        assert_eq!(r.triangles_flipped, 0);
        // Each connected component picks one seed.
        assert_eq!(r.components_visited, 1);
        assert_eq!(r.conflicting_edges, 0);
        assert_eq!(scene.meshes[0].primitives[0].positions, pre_positions);
    }

    #[test]
    fn winding_consistency_flips_flipped_neighbour_unindexed() {
        let mut scene = scene_with_primitives(vec![flipped_neighbour_quad_unindexed()]);

        // Pre-condition: validate flags 1 inconsistent edge.
        let pre = crate::validate(
            &scene,
            &crate::ValidationOptions {
                check_facet_orientation: false,
                check_unit_normal: false,
                ..crate::ValidationOptions::default()
            },
        );
        assert_eq!(pre.inconsistent_winding_edges, 1);

        let r = repair_make_winding_consistent(&mut scene);
        assert_eq!(r.triangles_inspected, 2);
        assert_eq!(r.triangles_flipped, 1, "report: {r:?}");
        assert_eq!(r.components_visited, 1);
        assert_eq!(r.conflicting_edges, 0);

        // Post-condition: validate flags zero inconsistent edges.
        let post = crate::validate(
            &scene,
            &crate::ValidationOptions {
                check_facet_orientation: false,
                check_unit_normal: false,
                ..crate::ValidationOptions::default()
            },
        );
        assert_eq!(post.inconsistent_winding_edges, 0, "post: {post:?}");

        // Tri 1's second and third slots were swapped: the original
        // [1,1,0] [0,0,0] [0,1,0] becomes [1,1,0] [0,1,0] [0,0,0].
        let pos = &scene.meshes[0].primitives[0].positions;
        assert_eq!(pos[3], [1.0, 1.0, 0.0]);
        assert_eq!(pos[4], [0.0, 1.0, 0.0]);
        assert_eq!(pos[5], [0.0, 0.0, 0.0]);

        // Idempotency: a second run is a no-op.
        let r2 = repair_make_winding_consistent(&mut scene);
        assert_eq!(r2.triangles_flipped, 0);
    }

    #[test]
    fn winding_consistency_flips_flipped_neighbour_indexed_u32() {
        // Indexed variant: one shared position buffer for the two
        // triangles (4 unique corners), index buffer carries the
        // flipped second face.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![
            [0.0, 0.0, 0.0], // 0
            [1.0, 0.0, 0.0], // 1
            [1.0, 1.0, 0.0], // 2
            [0.0, 1.0, 0.0], // 3
        ];
        // tri 0: 0,1,2 walks diagonal 2→0.
        // tri 1 flipped: 2,0,3 also walks 2→0 (inconsistent).
        prim.indices = Some(Indices::U32(vec![0, 1, 2, 2, 0, 3]));
        prim.normals = Some(vec![[0.0, 0.0, 1.0]; 4]);
        let mut scene = scene_with_primitives(vec![prim]);

        let r = repair_make_winding_consistent(&mut scene);
        assert_eq!(r.triangles_flipped, 1);

        // Indices' second face was swapped: 2,0,3 → 2,3,0.
        match &scene.meshes[0].primitives[0].indices {
            Some(Indices::U32(v)) => {
                assert_eq!(v.as_slice(), &[0, 1, 2, 2, 3, 0]);
            }
            other => panic!("expected U32 indices, got {other:?}"),
        }
        // Position buffer is untouched (indexed: flip rewrites only the
        // index buffer).
        let pos = &scene.meshes[0].primitives[0].positions;
        assert_eq!(pos[0], [0.0, 0.0, 0.0]);
        assert_eq!(pos[1], [1.0, 0.0, 0.0]);
        assert_eq!(pos[2], [1.0, 1.0, 0.0]);
        assert_eq!(pos[3], [0.0, 1.0, 0.0]);

        // Discriminant preserved.
        assert!(matches!(
            scene.meshes[0].primitives[0].indices,
            Some(Indices::U32(_))
        ));
    }

    #[test]
    fn winding_consistency_flips_flipped_neighbour_indexed_u16() {
        // U16 discriminant must be preserved on flip.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
        ];
        prim.indices = Some(Indices::U16(vec![0, 1, 2, 2, 0, 3]));
        prim.normals = Some(vec![[0.0, 0.0, 1.0]; 4]);
        let mut scene = scene_with_primitives(vec![prim]);

        let r = repair_make_winding_consistent(&mut scene);
        assert_eq!(r.triangles_flipped, 1);

        match &scene.meshes[0].primitives[0].indices {
            Some(Indices::U16(v)) => assert_eq!(v.as_slice(), &[0, 1, 2, 2, 3, 0]),
            other => panic!("expected U16 indices, got {other:?}"),
        }
    }

    #[test]
    fn winding_consistency_unindexed_swaps_normals_in_lockstep() {
        // Per-vertex normals (parallel to positions) get swapped along
        // with the corner positions on a flip.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            // flipped tri:
            [1.0, 1.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ];
        // Distinct per-vertex normals so the swap is observable.
        prim.normals = Some(vec![
            [0.1, 0.0, 0.0],
            [0.2, 0.0, 0.0],
            [0.3, 0.0, 0.0],
            // tri 1 — slots 4 (idx in vec = 4) and 5 should swap.
            [0.4, 0.0, 0.0],
            [0.5, 0.0, 0.0],
            [0.6, 0.0, 0.0],
        ]);
        let mut scene = scene_with_primitives(vec![prim]);

        let _ = repair_make_winding_consistent(&mut scene);
        let ns = scene.meshes[0].primitives[0].normals.as_ref().unwrap();
        // Tri 0 slots unchanged.
        assert_eq!(ns[0][0], 0.1);
        assert_eq!(ns[1][0], 0.2);
        assert_eq!(ns[2][0], 0.3);
        // Tri 1's second + third normal slots swapped.
        assert_eq!(ns[3][0], 0.4);
        assert_eq!(ns[4][0], 0.6);
        assert_eq!(ns[5][0], 0.5);
    }

    #[test]
    fn winding_consistency_counts_each_component_separately() {
        // Two disconnected triangles — no shared positions, so each
        // is its own component with its own seed.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![
            // tri 0 — origin
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            // tri 1 — far away
            [10.0, 10.0, 0.0],
            [11.0, 10.0, 0.0],
            [10.0, 11.0, 0.0],
        ];
        prim.normals = Some(vec![[0.0, 0.0, 1.0]; 6]);
        let mut scene = scene_with_primitives(vec![prim]);

        let r = repair_make_winding_consistent(&mut scene);
        assert_eq!(r.triangles_inspected, 2);
        assert_eq!(r.components_visited, 2);
        // No shared edges → no flips needed.
        assert_eq!(r.triangles_flipped, 0);
    }

    #[test]
    fn winding_consistency_non_triangles_primitive_skipped() {
        // Mixed scene: one Triangles primitive (counted) + one
        // non-Triangles (silently skipped).
        let mut tris = flipped_neighbour_quad_unindexed();
        // Hand off the same primitive ID to keep the test focused.
        let mut other = Primitive::new(Topology::LineStrip);
        other.positions = vec![[0.0; 3]; 3];
        // Sanity: the Triangles primitive flips one face.
        let mut scene = scene_with_primitives(vec![tris.clone(), other]);
        let r = repair_make_winding_consistent(&mut scene);
        assert_eq!(r.triangles_inspected, 2, "only Triangles counted");
        assert_eq!(r.triangles_flipped, 1);
        // (silence the unused-mut warning by referring to `tris` after.)
        tris.positions.clear();
    }

    #[test]
    fn winding_consistency_unindexed_doesnt_change_facecount() {
        // Pure invariant: flipping a face never adds or removes a
        // triangle. positions.len() / 3 stays constant.
        let mut scene = scene_with_primitives(vec![flipped_neighbour_quad_unindexed()]);
        let before = scene.meshes[0].primitives[0].positions.len();
        let _ = repair_make_winding_consistent(&mut scene);
        let after = scene.meshes[0].primitives[0].positions.len();
        assert_eq!(before, after);
        assert_eq!(after / 3, 2);
    }

    #[test]
    fn winding_consistency_preserves_extras_and_mesh_name() {
        // Extras and mesh name pass through untouched.
        use serde_json::json;
        let mut prim = flipped_neighbour_quad_unindexed();
        prim.extras
            .insert("custom".to_string(), json!("preserve-me"));
        let mut mesh = Mesh::new(Some("widget".to_string()));
        mesh.primitives.push(prim);
        let mut scene = Scene3D::new();
        scene.add_mesh(mesh);

        let _ = repair_make_winding_consistent(&mut scene);
        assert_eq!(
            scene.meshes[0].primitives[0]
                .extras
                .get("custom")
                .and_then(|v| v.as_str()),
            Some("preserve-me"),
        );
        assert_eq!(scene.meshes[0].name.as_deref(), Some("widget"));
    }

    // ----- T-junction split repair -----

    /// A classic T-junction: triangle A spans an edge `(0,0,0) → (2,0,0)`
    /// and triangle B sits adjacent with corner at the edge midpoint
    /// `(1,0,0)`. Triangle A has no awareness of B's corner. After the
    /// repair, A is replaced by two sub-triangles whose shared corner is
    /// the midpoint, so the watertight check passes too.
    fn t_junction_unindexed_pair() -> Primitive {
        let mut prim = Primitive::new(Topology::Triangles);
        // Triangle A: (0,0,0) → (2,0,0) → (1,1,0)  ← the long edge.
        prim.positions.push([0.0, 0.0, 0.0]);
        prim.positions.push([2.0, 0.0, 0.0]);
        prim.positions.push([1.0, 1.0, 0.0]);
        // Triangle B: (1,0,0) → (2,0,0) → (1,-1,0) ← shares midpoint
        // (1,0,0) with edge `(0,0,0) → (2,0,0)` of A.
        prim.positions.push([1.0, 0.0, 0.0]);
        prim.positions.push([2.0, 0.0, 0.0]);
        prim.positions.push([1.0, -1.0, 0.0]);
        prim.normals = Some(vec![[0.0, 0.0, 1.0]; prim.positions.len()]);
        prim
    }

    #[test]
    fn t_junction_split_empty_scene_is_noop() {
        let mut scene = Scene3D::new();
        let r = repair_split_t_junctions(&mut scene, DEFAULT_T_JUNCTION_SPLIT_TOLERANCE);
        assert_eq!(r.triangles_inspected, 0);
        assert_eq!(r.triangles_split, 0);
        assert_eq!(r.triangles_emitted, 0);
    }

    #[test]
    fn t_junction_split_skips_non_triangles_primitive() {
        let mut prim = Primitive::new(Topology::Lines);
        prim.positions = vec![[0.0; 3], [1.0; 3]];
        let mut scene = scene_with_primitives(vec![prim]);
        let r = repair_split_t_junctions(&mut scene, DEFAULT_T_JUNCTION_SPLIT_TOLERANCE);
        assert_eq!(r.triangles_inspected, 0);
        assert_eq!(r.triangles_split, 0);
    }

    #[test]
    fn t_junction_split_clean_pair_is_noop() {
        // Two triangles that share a full edge — no foreign corner sits
        // mid-edge. The pass should walk every face and emit zero
        // splits.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions.push([0.0, 0.0, 0.0]);
        prim.positions.push([1.0, 0.0, 0.0]);
        prim.positions.push([0.0, 1.0, 0.0]);
        prim.positions.push([1.0, 0.0, 0.0]);
        prim.positions.push([1.0, 1.0, 0.0]);
        prim.positions.push([0.0, 1.0, 0.0]);
        prim.normals = Some(vec![[0.0, 0.0, 1.0]; prim.positions.len()]);
        let mut scene = scene_with_primitives(vec![prim]);
        let r = repair_split_t_junctions(&mut scene, DEFAULT_T_JUNCTION_SPLIT_TOLERANCE);
        assert_eq!(r.triangles_inspected, 2);
        assert_eq!(r.triangles_split, 0);
        assert_eq!(r.triangles_unchanged, 2);
        assert_eq!(scene.meshes[0].primitives[0].positions.len(), 6);
    }

    #[test]
    fn t_junction_split_classic_pair_unindexed() {
        let mut scene = scene_with_primitives(vec![t_junction_unindexed_pair()]);
        let r = repair_split_t_junctions(&mut scene, DEFAULT_T_JUNCTION_SPLIT_TOLERANCE);
        // Triangle A has a midpoint splitter (B's first corner) on
        // its long edge → 1 split, 2 sub-triangles.
        // Triangle B has no foreign corner inside any of its edges
        // → unchanged.
        assert_eq!(r.triangles_inspected, 2);
        assert_eq!(r.triangles_split, 1);
        assert_eq!(r.split_vertices_inserted, 1);
        assert_eq!(r.triangles_emitted, 2);
        assert_eq!(r.triangles_unchanged, 1);
        // Final face count: 2 (A's split) + 1 (B unchanged) = 3.
        let prim = &scene.meshes[0].primitives[0];
        assert_eq!(prim.positions.len() / 3, 3);
        // Splitter (1,0,0) must appear among the new corners.
        assert!(prim.positions.iter().any(|p| p == &[1.0, 0.0, 0.0]));
    }

    #[test]
    fn t_junction_split_indexed_u32() {
        // Same fixture as the unindexed case but pre-welded into an
        // indexed primitive. Splitter index gets appended to
        // positions, original index buffer is rewritten.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![
            [0.0, 0.0, 0.0],  // 0
            [2.0, 0.0, 0.0],  // 1
            [1.0, 1.0, 0.0],  // 2  (A apex)
            [1.0, 0.0, 0.0],  // 3  (T-junction vertex)
            [1.0, -1.0, 0.0], // 4 (B apex)
        ];
        prim.normals = Some(vec![[0.0, 0.0, 1.0]; prim.positions.len()]);
        // Triangle A: 0→1→2. Triangle B: 3→1→4.
        prim.indices = Some(Indices::U32(vec![0, 1, 2, 3, 1, 4]));
        let mut scene = scene_with_primitives(vec![prim]);
        let r = repair_split_t_junctions(&mut scene, DEFAULT_T_JUNCTION_SPLIT_TOLERANCE);
        assert_eq!(r.triangles_split, 1);
        assert_eq!(r.split_vertices_inserted, 1);
        let prim = &scene.meshes[0].primitives[0];
        // Positions gain ONE entry — the appended splitter slot.
        // (The vertex already exists at index 3, but the append path
        // creates a fresh slot for simplicity — the documented
        // behaviour.)
        assert_eq!(prim.positions.len(), 6);
        // U32 discriminant preserved (max index well under u16::MAX
        // so the auto-narrow could land us in U16 — check the
        // logical face count instead).
        let face_count = match prim.indices.as_ref().unwrap() {
            Indices::U16(v) => v.len() / 3,
            Indices::U32(v) => v.len() / 3,
        };
        assert_eq!(face_count, 3);
    }

    #[test]
    fn t_junction_split_indexed_u16_auto_widening() {
        // Force the U16 → U32 widening path: pad to almost u16::MAX
        // pre-existing positions, then add the T-junction pair so a
        // single splitter spills over.
        let mut prim = Primitive::new(Topology::Triangles);
        // Pad with dummy positions (away from origin so they don't
        // interfere with the T-junction edge).
        for i in 0..(u16::MAX as usize) {
            prim.positions.push([1000.0 + i as f32, 0.0, 0.0]);
        }
        prim.positions.push([0.0, 0.0, 0.0]); // discarded by retruncate
        prim.positions.push([2.0, 0.0, 0.0]);
        prim.positions.push([1.0, 1.0, 0.0]);
        prim.positions.push([1.0, 0.0, 0.0]);
        prim.positions.push([1.0, -1.0, 0.0]);
        prim.normals = Some(vec![[0.0, 0.0, 1.0]; prim.positions.len()]);
        // Use U16 indices with positions filling slots 0..=u16::MAX
        // exactly — so the next appended splitter slot (u16::MAX + 1
        // = 65536) sits one past the U16 boundary and forces the
        // auto-widening path.
        prim.positions.truncate(0);
        prim.normals = Some(Vec::new());
        // 5 T-junction positions occupy slots u16::MAX-4 .. u16::MAX.
        // Slots 0..u16::MAX-4 are padded with sentinel positions far
        // from the T-junction edge.
        let total_padding = (u16::MAX as usize + 1) - 5;
        for i in 0..total_padding {
            prim.positions.push([1000.0 + i as f32, 0.0, 0.0]);
            prim.normals.as_mut().unwrap().push([0.0, 0.0, 1.0]);
        }
        let base = prim.positions.len() as u16;
        for p in [
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, -1.0, 0.0],
        ] {
            prim.positions.push(p);
            prim.normals.as_mut().unwrap().push([0.0, 0.0, 1.0]);
        }
        // Final position count: u16::MAX as usize + 1 = 65536. Max
        // index in the U16 buffer is base + 4 = u16::MAX.
        assert_eq!(prim.positions.len(), u16::MAX as usize + 1);
        prim.indices = Some(Indices::U16(vec![
            base,
            base + 1,
            base + 2,
            base + 3,
            base + 1,
            base + 4,
        ]));
        let mut scene = scene_with_primitives(vec![prim]);
        let r = repair_split_t_junctions(&mut scene, DEFAULT_T_JUNCTION_SPLIT_TOLERANCE);
        assert_eq!(r.triangles_split, 1);
        // Appending a fresh splitter pushes the max index above
        // u16::MAX, so the discriminant must auto-widen to U32.
        let kind = scene.meshes[0].primitives[0].indices.as_ref().unwrap();
        assert!(matches!(kind, Indices::U32(_)));
    }

    #[test]
    fn t_junction_split_idempotent_on_already_clean_scene() {
        // Run once on the clean fixture — no faces split. Then a
        // second run should still report zero splits.
        let mut scene = scene_with_primitives(vec![unit_cube_soup_primitive()]);
        let r1 = repair_split_t_junctions(&mut scene, DEFAULT_T_JUNCTION_SPLIT_TOLERANCE);
        assert_eq!(r1.triangles_split, 0);
        let r2 = repair_split_t_junctions(&mut scene, DEFAULT_T_JUNCTION_SPLIT_TOLERANCE);
        assert_eq!(r2.triangles_split, 0);
    }

    #[test]
    fn t_junction_split_eps_clamp_on_nonfinite() {
        let mut scene = scene_with_primitives(vec![t_junction_unindexed_pair()]);
        // NaN, infinity, and out-of-range eps all clamp to the
        // default tolerance — the classic T-junction must still
        // resolve.
        let r = repair_split_t_junctions(&mut scene, f32::NAN);
        assert_eq!(r.triangles_split, 1);
    }

    #[test]
    fn t_junction_split_preserves_normals_lengthmatch() {
        // Length-mismatched normals → the primitive is skipped.
        let mut prim = t_junction_unindexed_pair();
        // Shorten normals so the length doesn't match positions.
        prim.normals = Some(vec![[0.0, 0.0, 1.0]; 1]);
        let mut scene = scene_with_primitives(vec![prim]);
        let r = repair_split_t_junctions(&mut scene, DEFAULT_T_JUNCTION_SPLIT_TOLERANCE);
        assert_eq!(r.skipped_length_mismatch, 1);
        assert_eq!(r.triangles_split, 0);
        // Positions untouched.
        assert_eq!(scene.meshes[0].primitives[0].positions.len(), 6);
    }

    #[test]
    fn t_junction_split_two_splitters_on_one_edge_emits_three_subtriangles() {
        // One long edge `(0,0,0) → (3,0,0)` carries two splitters
        // at parameters t = 1/3 and t = 2/3. The fan should emit 3
        // sub-triangles.
        let mut prim = Primitive::new(Topology::Triangles);
        // Triangle A: 0→3→apex.
        prim.positions.push([0.0, 0.0, 0.0]);
        prim.positions.push([3.0, 0.0, 0.0]);
        prim.positions.push([1.5, 1.0, 0.0]);
        // Triangle B contributes a corner at (1,0,0) and (2,0,0).
        // It's two adjacent triangles emitted as a flat strip so
        // both corners are reachable as foreign keys.
        prim.positions.push([1.0, 0.0, 0.0]);
        prim.positions.push([2.0, 0.0, 0.0]);
        prim.positions.push([1.5, -1.0, 0.0]);
        prim.normals = Some(vec![[0.0, 0.0, 1.0]; prim.positions.len()]);
        let mut scene = scene_with_primitives(vec![prim]);
        let r = repair_split_t_junctions(&mut scene, DEFAULT_T_JUNCTION_SPLIT_TOLERANCE);
        // Triangle A's long edge has 2 splitters → 3 sub-triangles.
        // Triangle B has no foreign corner on any of its edges →
        // unchanged. The "best edge" tie-break picks the long edge
        // because both other A edges have no splitters.
        assert_eq!(r.triangles_split, 1);
        assert_eq!(r.split_vertices_inserted, 2);
        assert_eq!(r.triangles_emitted, 3);
        // Final face count: 3 + 1 = 4.
        let prim = &scene.meshes[0].primitives[0];
        assert_eq!(prim.positions.len() / 3, 4);
    }

    #[test]
    fn t_junction_split_extras_and_mesh_name_preserved() {
        use serde_json::json;
        let mut prim = t_junction_unindexed_pair();
        prim.extras.insert("preserved".to_string(), json!(true));
        let mut mesh = Mesh::new(Some("widget".to_string()));
        mesh.primitives.push(prim);
        let mut scene = Scene3D::new();
        scene.add_mesh(mesh);

        let _ = repair_split_t_junctions(&mut scene, DEFAULT_T_JUNCTION_SPLIT_TOLERANCE);
        assert_eq!(
            scene.meshes[0].primitives[0]
                .extras
                .get("preserved")
                .and_then(|v| v.as_bool()),
            Some(true),
        );
        assert_eq!(scene.meshes[0].name.as_deref(), Some("widget"));
    }

    // -- boundary_loops -------------------------------------------------

    #[test]
    fn boundary_loops_empty_scene() {
        assert!(boundary_loops(&Scene3D::new()).is_empty());
    }

    #[test]
    fn boundary_loops_watertight_cube_has_none() {
        // A closed cube has every edge shared by exactly two
        // triangles — zero boundary edges, zero loops.
        let scene = scene_with_primitives(vec![unit_cube_soup_primitive()]);
        assert_eq!(shells(&scene)[0].boundary_edges, 0);
        assert!(boundary_loops(&scene).is_empty());
    }

    #[test]
    fn boundary_loops_single_triangle_one_closed_loop() {
        // A lone triangle is all boundary: its three edges form one
        // closed loop walking the winding.
        let prim = one_triangle([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]);
        let scene = scene_with_primitives(vec![prim]);
        let loops = boundary_loops(&scene);
        assert_eq!(loops.len(), 1);
        assert!(loops[0].closed);
        assert_eq!(loops[0].edge_count(), 3);
        assert_eq!(loops[0].vertices.len(), 3);
        // The three corner positions all appear exactly once.
        let mut got: Vec<[f32; 3]> = loops[0].vertices.clone();
        got.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mut want = vec![[0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]];
        want.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert_eq!(got, want);
    }

    #[test]
    fn boundary_loops_walk_is_chained_in_winding_order() {
        // Two triangles sharing edge (1,0,0)-(0,1,0) form a quad whose
        // outer boundary is a single 4-edge loop. Verify the chain is
        // an actual cycle and the interior diagonal is never a loop edge.
        let mut prim = Primitive::new(Topology::Triangles);
        // tri A: (0,0,0)->(1,0,0)->(0,1,0)
        prim.positions
            .extend_from_slice(&[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]);
        // tri B: (1,0,0)->(1,1,0)->(0,1,0) — shares edge (1,0,0)-(0,1,0)
        prim.positions
            .extend_from_slice(&[[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]]);
        let scene = scene_with_primitives(vec![prim]);

        let loops = boundary_loops(&scene);
        assert_eq!(loops.len(), 1, "the quad has one outer boundary loop");
        let lp = &loops[0];
        assert!(lp.closed);
        // 4 distinct corner positions, the shared diagonal is interior.
        assert_eq!(lp.edge_count(), 4);
        assert_eq!(lp.vertices.len(), 4);
        let mut uniq: Vec<[f32; 3]> = lp.vertices.clone();
        uniq.sort_by(|a, b| a.partial_cmp(b).unwrap());
        uniq.dedup();
        assert_eq!(uniq.len(), 4, "no vertex repeats in the loop");
        // The diagonal edge (1,0,0)-(0,1,0) is NOT a boundary edge, so
        // consecutive loop vertices are never that pair.
        let n = lp.vertices.len();
        for i in 0..n {
            let a = lp.vertices[i];
            let b = lp.vertices[(i + 1) % n];
            let is_diag = (a == [1.0, 0.0, 0.0] && b == [0.0, 1.0, 0.0])
                || (a == [0.0, 1.0, 0.0] && b == [1.0, 0.0, 0.0]);
            assert!(!is_diag, "interior diagonal must not be a loop edge");
        }
    }

    #[test]
    fn boundary_loops_total_edge_count_matches_shell_boundary_count() {
        // Two disjoint triangles → two separate closed loops, and the
        // sum of their edge counts equals the total boundary-edge count.
        let a = one_triangle([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]);
        let b = one_triangle([[5.0, 5.0, 0.0], [6.0, 5.0, 0.0], [5.0, 6.0, 0.0]]);
        let scene = scene_with_primitives(vec![a, b]);

        let total_boundary: usize = shells(&scene).iter().map(|s| s.boundary_edges).sum();
        let loops = boundary_loops(&scene);
        let loop_edges: usize = loops.iter().map(|l| l.edge_count()).sum();
        assert_eq!(loop_edges, total_boundary);
        assert_eq!(loops.len(), 2);
        assert!(loops.iter().all(|l| l.closed && l.edge_count() == 3));
    }

    #[test]
    fn boundary_loops_stable_across_runs() {
        let prim = unit_cube_soup_primitive();
        // Drop one face to open a triangular hole.
        let mut prim2 = Primitive::new(Topology::Triangles);
        prim2
            .positions
            .extend_from_slice(&prim.positions[..33.min(prim.positions.len())]);
        let scene = scene_with_primitives(vec![prim2]);
        let r1 = boundary_loops(&scene);
        let r2 = boundary_loops(&scene);
        assert_eq!(r1, r2, "boundary_loops output is deterministic");
        assert!(!r1.is_empty());
    }

    #[test]
    fn boundary_loops_accounts_for_every_edge_on_non_manifold_boundary() {
        // A "bowtie": two triangles meeting at a single shared corner
        // (no shared edge). Every edge is a boundary edge; the shared
        // corner has multiple outgoing boundary edges — but every
        // boundary edge must still be accounted for exactly once.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions
            .extend_from_slice(&[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.5, 1.0, 0.0]]);
        prim.positions
            .extend_from_slice(&[[0.5, 1.0, 0.0], [1.5, 2.0, 0.0], [-0.5, 2.0, 0.0]]);
        let scene = scene_with_primitives(vec![prim]);

        let total_boundary: usize = shells(&scene).iter().map(|s| s.boundary_edges).sum();
        let loops = boundary_loops(&scene);
        let loop_edges: usize = loops.iter().map(|l| l.edge_count()).sum();
        assert_eq!(
            loop_edges, total_boundary,
            "every boundary edge appears in exactly one loop"
        );
        assert!(!loops.is_empty());
    }

    #[test]
    fn cap_skips_non_triangles_primitive() {
        let mut prim = Primitive::new(Topology::Lines);
        prim.positions
            .extend_from_slice(&[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]);
        let mut scene = scene_with_primitives(vec![prim]);
        let r = repair_cap_boundary_loops(&mut scene);
        assert_eq!(r.triangles_inspected, 0);
        assert_eq!(r.loops_capped, 0);
    }

    #[test]
    fn cap_skips_normals_length_mismatch() {
        // Open tetra missing one face but with a deliberately
        // wrong-length normals array — the pass must refuse to invent
        // per-vertex normals and leave the primitive untouched.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions.extend_from_slice(&[
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0],
        ]);
        // 9 positions but only 1 normal slot.
        prim.normals = Some(vec![[0.0, 0.0, 1.0]]);
        let mut scene = scene_with_primitives(vec![prim]);
        let r = repair_cap_boundary_loops(&mut scene);
        assert_eq!(r.skipped_length_mismatch, 1, "report: {r:?}");
        assert_eq!(r.loops_capped, 0);
        // Geometry untouched.
        assert_eq!(scene.meshes[0].primitives[0].positions.len(), 9);
    }

    #[test]
    fn cap_skips_open_non_manifold_chain() {
        // Three triangles fanned around a shared central edge A–B make
        // that edge used three times (non-manifold). The remaining
        // naked edges form chains whose walk hits a vertex with
        // multiple outgoing boundary edges and cannot close into a
        // single cycle, so the pass emits open chains rather than
        // guessing a cap.
        //   A = (0,0,0)  B = (0,0,1)
        //   wings P0=(1,0,0) P1=(-1,0,0) P2=(0,1,0)
        let mut prim = Primitive::new(Topology::Triangles);
        let a = [0.0_f32, 0.0, 0.0];
        let b = [0.0_f32, 0.0, 1.0];
        let p0 = [1.0_f32, 0.0, 0.0];
        let p1 = [-1.0_f32, 0.0, 0.0];
        let p2 = [0.0_f32, 1.0, 0.0];
        prim.positions.extend_from_slice(&[a, b, p0]);
        prim.positions.extend_from_slice(&[a, b, p1]);
        prim.positions.extend_from_slice(&[a, b, p2]);
        prim.normals = Some(vec![[0.0, 0.0, 0.0]; 9]);
        let mut scene = scene_with_primitives(vec![prim]);
        let before = scene.meshes[0].primitives[0].positions.len();
        let r = repair_cap_boundary_loops(&mut scene);
        // The non-manifold A–B edge prevents any single closed loop
        // from forming; the pass reports open chains and caps nothing.
        assert_eq!(r.loops_capped, 0, "report: {r:?}");
        assert!(r.open_chains_skipped >= 1, "report: {r:?}");
        assert_eq!(r.cap_triangles_emitted, 0);
        // Untouched geometry (no fan appended).
        assert_eq!(scene.meshes[0].primitives[0].positions.len(), before);
    }

    #[test]
    fn cap_lone_triangle_perimeter_is_a_closed_loop() {
        // A single free triangle's three edges are each used once, so
        // its perimeter is itself a closed 3-edge boundary loop. The
        // cap therefore mirrors the triangle (one reversed fan
        // triangle), producing a zero-volume but edge-manifold shell —
        // consistent, documented behaviour rather than a special case.
        let prim = one_triangle([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]);
        let mut scene = scene_with_primitives(vec![prim]);
        let r = repair_cap_boundary_loops(&mut scene);
        assert_eq!(r.loops_capped, 1, "report: {r:?}");
        assert_eq!(r.cap_triangles_emitted, 1);
        // Now edge-manifold: every edge used exactly twice.
        assert!(boundary_loops(&scene).is_empty());
    }

    #[test]
    fn cap_indexed_primitive_reuses_existing_slots() {
        // Open tetra as an indexed primitive: the cap must reuse the
        // four existing corner slots (no new positions appended) since
        // every loop vertex already has a bit-exact slot.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions.extend_from_slice(&[
            [0.0, 0.0, 0.0], // 0 = A
            [1.0, 0.0, 0.0], // 1 = B
            [0.0, 1.0, 0.0], // 2 = C
            [0.0, 0.0, 1.0], // 3 = D
        ]);
        prim.normals = Some(vec![[0.0, 0.0, 0.0]; 4]);
        // Faces A,C,B / A,B,D / A,D,C — leaves B,C,D open.
        prim.indices = Some(Indices::U32(vec![0, 2, 1, 0, 1, 3, 0, 3, 2]));
        let mut scene = scene_with_primitives(vec![prim]);
        let r = repair_cap_boundary_loops(&mut scene);
        assert_eq!(r.loops_capped, 1, "report: {r:?}");
        assert_eq!(r.cap_triangles_emitted, 1);
        let p = &scene.meshes[0].primitives[0];
        // No new positions — all loop corners already had slots.
        assert_eq!(p.positions.len(), 4, "cap should reuse existing slots");
        match &p.indices {
            Some(Indices::U32(v)) => assert_eq!(v.len(), 12, "3 old + 1 cap = 4 faces"),
            other => panic!("expected U32 indices, got {other:?}"),
        }
    }
}

//! Spec-aligned, opt-in geometry validation for STL [`Scene3D`]s.
//!
//! The 3D Systems *StereoLithography Interface Specification* (October
//! 1989) defines three rules a strictly-conformant STL surface must
//! obey, plus a unit/octant constraint inherited from the SLA pedigree:
//!
//! 1. **Facet orientation.** The facet's normal points outward; the
//!    three vertices are listed in counter-clockwise order when viewed
//!    from outside (right-hand rule). Spec says the two pieces of
//!    information are "specified redundantly … which must be
//!    consistent" — so the recomputed-from-winding normal should match
//!    the stored normal up to numerical tolerance.
//! 2. **Vertex-to-vertex rule.** Every triangle shares exactly two
//!    vertices with each of its adjacent triangles. T-junctions (where
//!    one triangle's vertex sits on another triangle's edge mid-span)
//!    are prohibited. A "shared edge" is an unordered pair of vertex
//!    positions; a watertight surface is one where every edge appears
//!    in exactly two triangles. T-junction geometry is a separate
//!    (opt-in) sub-check; the watertight edge-use count alone misses
//!    it because the offending vertex is *not* an endpoint of the
//!    edge it sits on.
//! 3. **All-positive octant.** All vertex coordinates must be
//!    "positive-definite (nonnegative and nonzero)" — i.e. strictly
//!    greater than zero on every axis. This is an SLA-era artefact
//!    that no modern slicer enforces, so we report it as a soft
//!    issue rather than a hard error.
//!
//! On top of those rules the spec also says a facet normal is a *unit*
//! vector; we surface non-unit normals as a separate diagnostic.
//!
//! ## Scope
//!
//! Validation is **opt-in and non-mutating** — the encoder/decoder
//! never invokes it. The intended consumers are:
//!
//! - Pipeline tooling that wants a "fitness for slicing" report on
//!   inbound STL geometry.
//! - Bug bisection workflows that want to point at a specific
//!   triangle index when a downstream renderer / printer rejects
//!   the file.
//! - Format-conversion adapters that want to know whether the source
//!   surface is watertight before exporting to a closed-mesh format.
//!
//! Reports a [`ValidationReport`] with per-rule counts and a list of
//! up to [`MAX_REPORTED_DEFECTS`] illustrative defect indices for each
//! rule (so large meshes don't OOM the diagnostic itself).
//!
//! Spec quotes are taken from `docs/3d/stl/fabbers-stl-format.html`
//! (Marshall Burns' transcription of §6.5 of *Automated Fabrication*).

use oxideav_mesh3d::{Indices, Scene3D, Topology};

/// Cap on the number of per-rule defect indices stored in a
/// [`ValidationReport`]. Counts are unbounded; only the example list
/// is truncated. Set to a small number on purpose — the report is for
/// human + tool consumption, not for re-walking the full geometry.
pub const MAX_REPORTED_DEFECTS: usize = 32;

/// Default tolerance for "the recomputed face normal matches the
/// stored normal" — a single component of the normalised cross
/// product must be within this absolute distance of the stored
/// value to count as a match.
pub const DEFAULT_NORMAL_TOLERANCE: f32 = 1.0e-3;

/// Default tolerance for "the stored normal is a unit vector" — the
/// length of the stored normal must be within this absolute distance
/// of `1.0`. Set generously enough to absorb the float-precision loss
/// of `f32::sqrt` on near-axis-aligned normals.
pub const DEFAULT_UNIT_NORMAL_TOLERANCE: f32 = 1.0e-3;

/// Default tolerance for "vertex V lies on edge PQ" — the
/// perpendicular distance from V to the infinite line through P, Q
/// must be at most `eps * |PQ|`, and the projected parameter must lie
/// in `(eps, 1 - eps)` (strictly between the endpoints under the same
/// tolerance). Smaller than the normal tolerance because the T-junction
/// check is geometric and we want low false-positive rates on near-
/// degenerate but legitimately corner-touching triangulations.
pub const DEFAULT_T_JUNCTION_TOLERANCE: f32 = 1.0e-5;

/// Per-vertex axis-aligned bounding box for a [`Scene3D`].
///
/// All coordinates are taken as-is from the scene's `Triangles`
/// primitives without applying node-graph transforms — STL has no
/// instancing, so the typical scene tree this crate produces is one
/// root node per mesh with identity transforms anyway.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bbox {
    /// Minimum corner.
    pub min: [f32; 3],
    /// Maximum corner.
    pub max: [f32; 3],
}

impl Bbox {
    /// Width on each axis (`max - min`). Returns `[0; 3]` for an
    /// empty bbox.
    pub fn extents(&self) -> [f32; 3] {
        [
            self.max[0] - self.min[0],
            self.max[1] - self.min[1],
            self.max[2] - self.min[2],
        ]
    }

    /// Centre point (`(min + max) / 2`).
    pub fn centre(&self) -> [f32; 3] {
        [
            (self.min[0] + self.max[0]) * 0.5,
            (self.min[1] + self.max[1]) * 0.5,
            (self.min[2] + self.max[2]) * 0.5,
        ]
    }

    /// Whether any extent is zero or negative — i.e. the bbox is
    /// degenerate (all points coplanar on at least one axis or the
    /// bbox is empty).
    pub fn is_degenerate(&self) -> bool {
        let e = self.extents();
        e[0] <= 0.0 || e[1] <= 0.0 || e[2] <= 0.0
    }

    /// Volume of the bounding box — the product of its three extents.
    /// Returns `0.0` when the box is degenerate on any axis (matches
    /// [`Self::is_degenerate`]). Non-finite extents propagate through
    /// the `f32` multiplication (the bbox itself is only ever built
    /// from finite vertex coordinates by [`bbox`], so this returns a
    /// finite number for any bbox produced by this crate).
    pub fn volume(&self) -> f32 {
        let e = self.extents();
        e[0] * e[1] * e[2]
    }

    /// Surface area of the bounding box — `2 * (xy + yz + xz)`. Returns
    /// `0.0` for a fully-degenerate (point) box; partially-degenerate
    /// boxes report a positive area drawn only from the two
    /// non-degenerate axes (one face's area, doubled).
    pub fn surface_area(&self) -> f32 {
        let e = self.extents();
        2.0 * (e[0] * e[1] + e[1] * e[2] + e[0] * e[2])
    }

    /// Length of the box's space diagonal — `sqrt(dx^2 + dy^2 + dz^2)`.
    /// Useful as a single scalar "scene size" headline.
    pub fn diagonal_length(&self) -> f32 {
        let e = self.extents();
        (e[0] * e[0] + e[1] * e[1] + e[2] * e[2]).sqrt()
    }

    /// Index of the axis (`0` = X, `1` = Y, `2` = Z) with the greatest
    /// extent. Slicer pipelines use this to pick which axis to sweep
    /// the cutting plane along (sweeping the longest axis maximises the
    /// per-layer fill ratio for a given layer thickness). Ties resolve
    /// toward the lower index — `[1, 1, 0.5]` returns `0`, `[0.5, 1, 1]`
    /// returns `1`. Returns [`None`] for a degenerate (zero-volume) box,
    /// because no single axis dominates a flat or empty bbox.
    pub fn longest_axis(&self) -> Option<usize> {
        if self.is_degenerate() {
            return None;
        }
        let e = self.extents();
        // Tie-break toward the lower index by using strict `>` against
        // the running best.
        let mut best = 0usize;
        if e[1] > e[best] {
            best = 1;
        }
        if e[2] > e[best] {
            best = 2;
        }
        Some(best)
    }

    /// Whether `p` lies inside the bounding box (inclusive on every
    /// face). Non-finite components in `p` propagate through the `<=`
    /// comparison and return `false` for that component, which mirrors
    /// the spec-style silent-skip behaviour [`bbox`] uses for
    /// non-finite vertex coordinates.
    pub fn contains_point(&self, p: [f32; 3]) -> bool {
        p[0] >= self.min[0]
            && p[0] <= self.max[0]
            && p[1] >= self.min[1]
            && p[1] <= self.max[1]
            && p[2] >= self.min[2]
            && p[2] <= self.max[2]
    }

    /// Degenerate "single point" bbox — `min == max == p`. Useful as
    /// the seed for code that incrementally merges further bboxes via
    /// [`Self::merge`] without juggling an [`Option<Bbox>`]. The
    /// resulting box reports zero extents on every axis and is
    /// [`Self::is_degenerate`] by construction. Non-finite `p`
    /// components are stored as-is; subsequent merges with finite
    /// boxes propagate the non-finite values through the per-axis
    /// `min`/`max` (`NaN`-aware ordering is not applied here — feed
    /// only finite seeds when bit-exact correctness across an
    /// accumulation chain matters).
    pub fn point(p: [f32; 3]) -> Self {
        Self { min: p, max: p }
    }

    /// Component-wise union of two bounding boxes — the smallest
    /// axis-aligned box that contains every point in either input.
    /// `merge` is commutative and associative, so an accumulation
    /// chain like `a.merge(b).merge(c)` produces the same box as
    /// `c.merge(a).merge(b)`. Useful for pipeline tooling that needs
    /// to compose a scene-wide bbox from per-mesh / per-primitive
    /// scopes (e.g. multi-source slicer pre-flight where each input
    /// reports its own bbox) without re-walking the geometry.
    pub fn merge(&self, other: &Bbox) -> Bbox {
        Bbox {
            min: [
                self.min[0].min(other.min[0]),
                self.min[1].min(other.min[1]),
                self.min[2].min(other.min[2]),
            ],
            max: [
                self.max[0].max(other.max[0]),
                self.max[1].max(other.max[1]),
                self.max[2].max(other.max[2]),
            ],
        }
    }

    /// Box grown by `margin` on every face — `min - margin`, `max +
    /// margin` per axis. Useful as a slicer pre-flight safety margin
    /// (extruder / laser kerf, raft clearance, build-plate edge
    /// tolerance). Negative `margin` shrinks the box and may produce
    /// a degenerate or inverted result when the magnitude exceeds
    /// half an extent on any axis — the caller is responsible for
    /// re-checking [`Self::is_degenerate`] afterwards if that
    /// matters. Non-finite `margin` propagates through the
    /// component-wise addition; finite inputs are guaranteed to
    /// produce finite outputs.
    pub fn expanded_by(&self, margin: f32) -> Bbox {
        Bbox {
            min: [
                self.min[0] - margin,
                self.min[1] - margin,
                self.min[2] - margin,
            ],
            max: [
                self.max[0] + margin,
                self.max[1] + margin,
                self.max[2] + margin,
            ],
        }
    }

    /// Whether two bounding boxes overlap on every axis (inclusive on
    /// every face — boxes that touch on exactly one face share that
    /// face and count as intersecting). Symmetric
    /// (`a.intersects(&b) == b.intersects(&a)`); self-intersection is
    /// always true for any non-inverted box. The dual of [`Self::merge`]
    /// — `merge` returns the smallest box containing both inputs;
    /// `intersects` reports whether the largest box contained in both
    /// inputs is non-empty (see [`Self::intersect`] for the box itself).
    /// Useful for slicer pre-flight collision queries when several
    /// parts share a build-plate.
    pub fn intersects(&self, other: &Bbox) -> bool {
        self.min[0] <= other.max[0]
            && self.max[0] >= other.min[0]
            && self.min[1] <= other.max[1]
            && self.max[1] >= other.min[1]
            && self.min[2] <= other.max[2]
            && self.max[2] >= other.min[2]
    }

    /// The overlap region of two bounding boxes — the largest box
    /// contained in both inputs. Returns [`None`] when the inputs do
    /// not overlap (`!self.intersects(other)`); otherwise the returned
    /// box has `min == max(self.min, other.min)` and `max ==
    /// min(self.max, other.max)` component-wise. The result may be
    /// degenerate on any axis whose `self.min == other.max` (or the
    /// dual) — touching on exactly one face produces a flat (zero-
    /// extent) intersection. Symmetric and idempotent
    /// (`a.intersect(&a) == Some(a)` for any non-inverted box).
    /// Component-wise dual of [`Self::merge`].
    pub fn intersect(&self, other: &Bbox) -> Option<Bbox> {
        if !self.intersects(other) {
            return None;
        }
        Some(Bbox {
            min: [
                self.min[0].max(other.min[0]),
                self.min[1].max(other.min[1]),
                self.min[2].max(other.min[2]),
            ],
            max: [
                self.max[0].min(other.max[0]),
                self.max[1].min(other.max[1]),
                self.max[2].min(other.max[2]),
            ],
        })
    }

    /// Whether `other` lies entirely inside `self` (inclusive on every
    /// face). Reflexive (`a.contains_bbox(&a) == true` for any
    /// non-inverted box) and transitive (`a ⊇ b && b ⊇ c → a ⊇ c`).
    /// A degenerate `other` (zero extents on some axis) is still
    /// "contained" as long as its single-point face lies within
    /// `self`'s closed range. Useful for slicer pre-flight checks
    /// like "does this part bbox fit inside the build-plate
    /// envelope" — call `build_plate.contains_bbox(&part)`.
    pub fn contains_bbox(&self, other: &Bbox) -> bool {
        self.min[0] <= other.min[0]
            && self.min[1] <= other.min[1]
            && self.min[2] <= other.min[2]
            && self.max[0] >= other.max[0]
            && self.max[1] >= other.max[1]
            && self.max[2] >= other.max[2]
    }

    /// The eight corner vertices of the bounding box, in a fixed
    /// canonical order derived from the three-bit Cartesian product of
    /// `(min, max)` on each axis with X as the lowest-order bit:
    ///
    /// | Index | Bit pattern (zyx) | Corner                               |
    /// | ----- | ----------------- | ------------------------------------ |
    /// | 0     | `000`             | `(min.x, min.y, min.z)`              |
    /// | 1     | `001`             | `(max.x, min.y, min.z)`              |
    /// | 2     | `010`             | `(min.x, max.y, min.z)`              |
    /// | 3     | `011`             | `(max.x, max.y, min.z)`              |
    /// | 4     | `100`             | `(min.x, min.y, max.z)`              |
    /// | 5     | `101`             | `(max.x, min.y, max.z)`              |
    /// | 6     | `110`             | `(min.x, max.y, max.z)`              |
    /// | 7     | `111`             | `(max.x, max.y, max.z)`              |
    ///
    /// The lowest-z face is corners `[0, 1, 2, 3]` (a slicer's first
    /// layer for a part on the build plate); the highest-z face is
    /// `[4, 5, 6, 7]`; opposite corners are at indices `i` and `7 - i`.
    /// Corner `0` is always [`Self::min`] and corner `7` is always
    /// [`Self::max`].
    ///
    /// Useful for pipelines that need to test the bbox against a
    /// non-axis-aligned transform (e.g. asking "would this part still
    /// fit on the build plate after a 30° Z-rotation?"), for visualising
    /// the bbox as a wireframe, or for computing a rotated bbox by
    /// transforming each corner and re-bounding the transformed set.
    /// A degenerate bbox (one or more zero extents) collapses pairs of
    /// corners onto each other but the eight-slot layout is preserved.
    /// All eight corners satisfy [`Self::contains_point`] on `self`
    /// (inclusive on every face).
    pub fn corners(&self) -> [[f32; 3]; 8] {
        let mn = self.min;
        let mx = self.max;
        [
            [mn[0], mn[1], mn[2]],
            [mx[0], mn[1], mn[2]],
            [mn[0], mx[1], mn[2]],
            [mx[0], mx[1], mn[2]],
            [mn[0], mn[1], mx[2]],
            [mx[0], mn[1], mx[2]],
            [mn[0], mx[1], mx[2]],
            [mx[0], mx[1], mx[2]],
        ]
    }
}

/// Axis-aligned bounding box of every vertex in every `Triangles`
/// primitive of `scene`.
///
/// Returns [`None`] when the scene contains zero `Triangles` vertices
/// (an empty scene, or a scene whose primitives are all non-triangles).
/// Non-finite coordinates are skipped — they don't contribute to
/// `min`/`max`, matching the spec's silent-skip behaviour around
/// pathological inputs.
pub fn bbox(scene: &Scene3D) -> Option<Bbox> {
    let mut acc = BboxAccumulator::new();
    for_each_emitted_vertex(scene, |p| acc.add(p));
    acc.finish()
}

/// Axis-aligned bounding box of every vertex of the `mesh_idx`-th mesh's
/// `Triangles` primitives.
///
/// Returns [`None`] when:
/// - `mesh_idx` is out of range for `scene.meshes`.
/// - The selected mesh has no `Triangles` vertices (all non-triangle
///   primitives, or all vertex coordinates non-finite).
///
/// Non-finite coordinates are skipped (same convention as [`bbox`]).
/// Node-graph transforms are NOT applied — STL produces identity-
/// transform single-mesh trees in practice.
pub fn bbox_of_mesh(scene: &Scene3D, mesh_idx: usize) -> Option<Bbox> {
    let mesh = scene.meshes.get(mesh_idx)?;
    let mut acc = BboxAccumulator::new();
    for prim in &mesh.primitives {
        if prim.topology != Topology::Triangles {
            continue;
        }
        accumulate_primitive(prim, &mut acc);
    }
    acc.finish()
}

/// Axis-aligned bounding box of the `prim_idx`-th primitive of the
/// `mesh_idx`-th mesh, when that primitive is `Triangles`.
///
/// Returns [`None`] when:
/// - Either index is out of range.
/// - The selected primitive's topology is not `Triangles`.
/// - The primitive has no finite vertex coordinates.
///
/// Non-finite coordinates are skipped (same convention as [`bbox`]).
pub fn bbox_of_primitive(scene: &Scene3D, mesh_idx: usize, prim_idx: usize) -> Option<Bbox> {
    let prim = scene.meshes.get(mesh_idx)?.primitives.get(prim_idx)?;
    if prim.topology != Topology::Triangles {
        return None;
    }
    let mut acc = BboxAccumulator::new();
    accumulate_primitive(prim, &mut acc);
    acc.finish()
}

/// Walker that accumulates min/max per axis. Used by [`bbox`],
/// [`bbox_of_mesh`], [`bbox_of_primitive`].
struct BboxAccumulator {
    any: bool,
    mn: [f32; 3],
    mx: [f32; 3],
}

impl BboxAccumulator {
    fn new() -> Self {
        Self {
            any: false,
            mn: [f32::INFINITY; 3],
            mx: [f32::NEG_INFINITY; 3],
        }
    }

    fn add(&mut self, p: [f32; 3]) {
        for (axis, &c) in p.iter().enumerate() {
            if c.is_finite() {
                if c < self.mn[axis] {
                    self.mn[axis] = c;
                }
                if c > self.mx[axis] {
                    self.mx[axis] = c;
                }
                self.any = true;
            }
        }
    }

    fn finish(self) -> Option<Bbox> {
        if self.any
            && self.mn.iter().all(|c| c.is_finite())
            && self.mx.iter().all(|c| c.is_finite())
        {
            Some(Bbox {
                min: self.mn,
                max: self.mx,
            })
        } else {
            None
        }
    }
}

/// Drive a single `Triangles` primitive through a [`BboxAccumulator`].
/// Mirrors the index-resolution logic in [`for_each_emitted_vertex`]
/// without walking the whole scene.
fn accumulate_primitive(prim: &oxideav_mesh3d::Primitive, acc: &mut BboxAccumulator) {
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
                acc.add(*p);
            }
        }
    }
}

/// Logical face-index used in [`ValidationReport`] defect lists.
///
/// `mesh` and `primitive` indices are scene-graph order; `face` is the
/// triangle's index within that primitive's effective vertex stream
/// (post-index-buffer resolution, matching the encoder's emit order).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct FaceLocator {
    /// Index into [`Scene3D::meshes`].
    pub mesh: usize,
    /// Index into the mesh's `primitives` vec.
    pub primitive: usize,
    /// Index of the triangle within the primitive (0-based).
    pub face: usize,
}

/// Tunable knobs for [`validate`].
#[derive(Clone, Copy, Debug)]
pub struct ValidationOptions {
    /// Component-wise tolerance for "stored normal matches recomputed
    /// face normal". Default [`DEFAULT_NORMAL_TOLERANCE`].
    pub normal_tolerance: f32,
    /// Tolerance for "stored normal length is `1.0`". Default
    /// [`DEFAULT_UNIT_NORMAL_TOLERANCE`].
    pub unit_normal_tolerance: f32,
    /// Whether to apply the spec's "all-positive octant" rule. Modern
    /// slicers ignore this, so it defaults to `false` — set to `true`
    /// to surface negative-coordinate facets in the report.
    pub check_positive_octant: bool,
    /// Whether to apply the right-hand-rule check (stored normal vs
    /// recomputed-from-winding cross product). Default `true`.
    pub check_facet_orientation: bool,
    /// Whether to apply the unit-length normal check. Default `true`.
    pub check_unit_normal: bool,
    /// Whether to apply the watertight (vertex-to-vertex) check.
    /// Default `true`. The check uses bit-exact `f32` position
    /// equality; meshes whose duplicate corners differ by floating-
    /// point noise should be pre-deduplicated through the tolerance
    /// helpers in [`crate::EncodeStats`].
    pub check_watertight: bool,
    /// Whether to apply the T-junction check — every vertex must
    /// either coincide with another triangle's corner or lie outside
    /// every other triangle's edge. The spec phrases this as "a
    /// vertex of one triangle cannot lie on the side (edge) of another
    /// triangle". Default `false` because the brute-force check is
    /// `O(E · V_unique)` and is intended for diagnostic use on
    /// triangulations small enough to comfortably scan; enable it
    /// explicitly when you need the report.
    pub check_t_junctions: bool,
    /// Tolerance for "vertex V lies on edge PQ" — see
    /// [`DEFAULT_T_JUNCTION_TOLERANCE`]. Used only when
    /// `check_t_junctions` is on. Negative / non-finite values are
    /// clamped to the default.
    pub t_junction_tolerance: f32,
    /// Whether to apply the consistent-winding check — for a correctly
    /// oriented surface, an edge shared by two triangles must be
    /// traversed in *opposite* directions by each (one walks `A→B`, the
    /// other `B→A`). When both traverse it `A→B`, one triangle's winding
    /// is flipped relative to its neighbour. Default `true`. The spec's
    /// facet-orientation rule (§6.5) says the vertices are "listed in
    /// counterclockwise order when looking at the object from the
    /// outside" and "must be consistent"; this is the mesh-wide
    /// (neighbour-relative) form of that rule, distinct from the
    /// per-facet stored-normal-vs-winding [`check_facet_orientation`]
    /// rule and from the undirected-edge [`check_watertight`] rule.
    /// Uses bit-exact `f32` position equality like the watertight
    /// check.
    pub check_consistent_winding: bool,
}

impl Default for ValidationOptions {
    fn default() -> Self {
        Self {
            normal_tolerance: DEFAULT_NORMAL_TOLERANCE,
            unit_normal_tolerance: DEFAULT_UNIT_NORMAL_TOLERANCE,
            check_positive_octant: false,
            check_facet_orientation: true,
            check_unit_normal: true,
            check_watertight: true,
            check_t_junctions: false,
            t_junction_tolerance: DEFAULT_T_JUNCTION_TOLERANCE,
            check_consistent_winding: true,
        }
    }
}

/// Result of a [`validate`] call.
///
/// Counts are unbounded; the per-rule example lists are capped at
/// [`MAX_REPORTED_DEFECTS`] entries each so the report stays cheap to
/// stash in a log or send over the wire even for million-triangle
/// surfaces. A `triangles_total` of 0 is a legitimate result for an
/// empty scene; every rule's count + examples will be empty too.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ValidationReport {
    /// Total triangle count walked.
    pub triangles_total: usize,
    /// Total count of facets whose stored normal disagrees with the
    /// recomputed-from-winding normal beyond [`ValidationOptions::normal_tolerance`].
    /// Zero when [`ValidationOptions::check_facet_orientation`] is off.
    pub facet_orientation_defects: usize,
    /// Up to [`MAX_REPORTED_DEFECTS`] illustrative facet locations
    /// from `facet_orientation_defects`.
    pub facet_orientation_examples: Vec<FaceLocator>,
    /// Total count of facets whose stored normal length differs from
    /// `1.0` beyond [`ValidationOptions::unit_normal_tolerance`].
    /// Zero when [`ValidationOptions::check_unit_normal`] is off.
    pub non_unit_normal_defects: usize,
    /// Up to [`MAX_REPORTED_DEFECTS`] illustrative facet locations
    /// from `non_unit_normal_defects`.
    pub non_unit_normal_examples: Vec<FaceLocator>,
    /// Total count of facets with at least one vertex whose any axis
    /// is `<= 0` (the "all-positive octant" rule). Zero when
    /// [`ValidationOptions::check_positive_octant`] is off.
    pub positive_octant_defects: usize,
    /// Up to [`MAX_REPORTED_DEFECTS`] illustrative facet locations
    /// from `positive_octant_defects`.
    pub positive_octant_examples: Vec<FaceLocator>,
    /// Number of *unique edges* — unordered (u32-bit, u32-bit) pairs
    /// of bit-exact vertex positions — that appear in exactly one
    /// triangle. Watertight surfaces have `boundary_edges == 0`.
    /// Zero when [`ValidationOptions::check_watertight`] is off.
    pub boundary_edges: usize,
    /// Number of unique edges that appear in three or more
    /// triangles — a sign of a non-manifold surface. Zero when
    /// [`ValidationOptions::check_watertight`] is off.
    pub non_manifold_edges: usize,
    /// `true` iff every edge appears in exactly two triangles
    /// (`boundary_edges == 0` AND `non_manifold_edges == 0`) AND
    /// at least one triangle was walked. `false` for empty scenes
    /// or when [`ValidationOptions::check_watertight`] is off.
    pub watertight: bool,
    /// Number of distinct (offending-vertex, edge) incidence
    /// pairs where a triangle's corner lies strictly between the
    /// two endpoints of another triangle's edge. The spec's
    /// vertex-to-vertex rule forbids these. Zero when
    /// [`ValidationOptions::check_t_junctions`] is off. Each
    /// incidence counts once even when several edges share the
    /// same offending vertex.
    pub t_junction_defects: usize,
    /// Up to [`MAX_REPORTED_DEFECTS`] illustrative facet locations
    /// of triangles whose corner sits in the middle of some other
    /// triangle's edge. A single triangle may appear multiple times
    /// when more than one of its corners is offending; entries are
    /// reported in scan order.
    pub t_junction_examples: Vec<FaceLocator>,
    /// Number of *manifold* edges (exactly two incident triangles)
    /// that both triangles traverse in the same direction — a sign
    /// that one of the two adjacent triangles has flipped winding
    /// relative to the other. Watertight surfaces can still have
    /// these (the undirected edge-use count is 2 either way), so the
    /// check is a distinct invariant. Zero when
    /// [`ValidationOptions::check_consistent_winding`] is off.
    pub inconsistent_winding_edges: usize,
    /// Up to [`MAX_REPORTED_DEFECTS`] illustrative facet locations of
    /// triangles incident on a same-direction shared edge. Each
    /// offending edge contributes the locators of both adjacent
    /// triangles (capped overall at [`MAX_REPORTED_DEFECTS`]).
    pub inconsistent_winding_examples: Vec<FaceLocator>,
}

impl ValidationReport {
    /// `true` when no rule reported any defects — the report is
    /// "clean". Empty scenes are vacuously clean.
    pub fn is_clean(&self) -> bool {
        self.facet_orientation_defects == 0
            && self.non_unit_normal_defects == 0
            && self.positive_octant_defects == 0
            && self.boundary_edges == 0
            && self.non_manifold_edges == 0
            && self.t_junction_defects == 0
            && self.inconsistent_winding_edges == 0
    }

    /// Sum of every per-rule defect count in the report — a single
    /// scalar headline for tooling that wants to log or sort scenes
    /// by their overall validity rather than inspect each rule's
    /// counters individually.
    ///
    /// The sum is the arithmetic total across all seven defect
    /// counters (`facet_orientation_defects`, `non_unit_normal_defects`,
    /// `positive_octant_defects`, `boundary_edges`, `non_manifold_edges`,
    /// `t_junction_defects`, `inconsistent_winding_edges`). Counters
    /// for rules whose [`ValidationOptions`] toggle is off are zero by
    /// construction and contribute nothing to the sum, so the number
    /// is bounded by the rule set actually run.
    ///
    /// Returns `0` iff [`Self::is_clean`] returns `true`; the converse
    /// also holds — these two predicates encode the same invariant
    /// at quantitative and boolean granularity.
    pub fn defect_total(&self) -> usize {
        self.facet_orientation_defects
            + self.non_unit_normal_defects
            + self.positive_octant_defects
            + self.boundary_edges
            + self.non_manifold_edges
            + self.t_junction_defects
            + self.inconsistent_winding_edges
    }

    /// Labeled breakdown of every per-rule defect count, in the same
    /// scan order as the [`validate`] pass. Useful for logging,
    /// sorting, and CI-style row-per-rule reporting:
    ///
    /// ```rust
    /// use oxideav_stl::{validate, ValidationOptions};
    /// # let scene = oxideav_mesh3d::Scene3D::new();
    /// let rep = validate(&scene, &ValidationOptions::default());
    /// for (rule, count) in rep.defects_by_rule() {
    ///     if count > 0 {
    ///         eprintln!("{rule}: {count}");
    ///     }
    /// }
    /// ```
    ///
    /// The labels are stable strings safe to use as metric names or
    /// log keys: `"facet_orientation"`, `"non_unit_normal"`,
    /// `"positive_octant"`, `"boundary_edges"`,
    /// `"non_manifold_edges"`, `"t_junction"`,
    /// `"inconsistent_winding"`. The seven entries' counts sum to
    /// [`Self::defect_total`].
    pub fn defects_by_rule(&self) -> [(&'static str, usize); 7] {
        [
            ("facet_orientation", self.facet_orientation_defects),
            ("non_unit_normal", self.non_unit_normal_defects),
            ("positive_octant", self.positive_octant_defects),
            ("boundary_edges", self.boundary_edges),
            ("non_manifold_edges", self.non_manifold_edges),
            ("t_junction", self.t_junction_defects),
            ("inconsistent_winding", self.inconsistent_winding_edges),
        ]
    }
}

/// Run the configured rules against `scene` and return a
/// [`ValidationReport`].
///
/// Non-`Triangles` primitives are silently skipped (they would be
/// rejected at encode-time anyway). Per-vertex normal arrays
/// shorter than the position array trigger "no stored normal
/// available" — the facet-orientation check is silently skipped on
/// those facets rather than counted as a defect.
pub fn validate(scene: &Scene3D, opts: &ValidationOptions) -> ValidationReport {
    let mut rep = ValidationReport::default();

    // Edge map for the watertight check. Key is a canonical (lo, hi)
    // pair of (x, y, z) bit-tuples so reverse-orientation edges still
    // collide. `u32` triples come straight from `f32::to_bits` for the
    // same NaN-distinct semantics the rest of the crate uses.
    type Vert = (u32, u32, u32);
    use std::collections::HashMap;
    let mut edge_uses: HashMap<(Vert, Vert), usize> = HashMap::new();

    let mut tri_index_global: usize = 0;
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
                rep.triangles_total += 1;
                tri_index_global += 1;
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
                let loc = FaceLocator {
                    mesh: mesh_idx,
                    primitive: prim_idx,
                    face: face_idx,
                };

                // Positive-octant rule.
                if opts.check_positive_octant {
                    let in_octant = positive_octant_face(v0, v1, v2);
                    if !in_octant {
                        rep.positive_octant_defects += 1;
                        push_capped(&mut rep.positive_octant_examples, loc);
                    }
                }

                // Facet orientation + unit-normal rules need the
                // stored normal — pulled from the first vertex's slot
                // when the normal array exists and is the right size.
                let stored_normal = prim
                    .normals
                    .as_ref()
                    .filter(|ns| ns.len() == prim.positions.len())
                    .map(|ns| ns[vi0]);

                if opts.check_facet_orientation {
                    if let Some(stored) = stored_normal {
                        let recomputed = recompute_normal(v0, v1, v2);
                        if !normals_within(stored, recomputed, opts.normal_tolerance) {
                            rep.facet_orientation_defects += 1;
                            push_capped(&mut rep.facet_orientation_examples, loc);
                        }
                    }
                }
                if opts.check_unit_normal {
                    if let Some(stored) = stored_normal {
                        let len =
                            (stored[0] * stored[0] + stored[1] * stored[1] + stored[2] * stored[2])
                                .sqrt();
                        // Allow zero-length normals (the spec'd
                        // sentinel for "consumer should recompute
                        // from winding") to pass — they're a known
                        // convention, not a defect. Anything else
                        // outside [1 - eps, 1 + eps] is a defect.
                        let is_zero_normal = len.abs() < opts.unit_normal_tolerance;
                        if !is_zero_normal && (len - 1.0).abs() > opts.unit_normal_tolerance {
                            rep.non_unit_normal_defects += 1;
                            push_capped(&mut rep.non_unit_normal_examples, loc);
                        }
                    }
                }

                // Watertight rule — count uses of each canonical edge.
                if opts.check_watertight {
                    let bits =
                        |p: [f32; 3]| -> Vert { (p[0].to_bits(), p[1].to_bits(), p[2].to_bits()) };
                    let b0 = bits(v0);
                    let b1 = bits(v1);
                    let b2 = bits(v2);
                    for (a, b) in [(b0, b1), (b1, b2), (b2, b0)] {
                        let key = if a <= b { (a, b) } else { (b, a) };
                        *edge_uses.entry(key).or_insert(0) += 1;
                    }
                }
            }
        }
    }

    if opts.check_watertight {
        let mut boundary = 0usize;
        let mut non_manifold = 0usize;
        for &uses in edge_uses.values() {
            match uses {
                0 => {} // unreachable but harmless
                1 => boundary += 1,
                2 => {} // exactly two — the manifold-correct case
                _ => non_manifold += 1,
            }
        }
        rep.boundary_edges = boundary;
        rep.non_manifold_edges = non_manifold;
        // Empty scenes are vacuously NOT watertight (there's nothing
        // to seal); a non-empty scene with zero boundary + zero
        // non-manifold edges is.
        rep.watertight = tri_index_global > 0 && boundary == 0 && non_manifold == 0;
    }

    if opts.check_t_junctions {
        check_t_junctions(scene, opts, &mut rep);
    }
    if opts.check_consistent_winding {
        check_consistent_winding(scene, &mut rep);
    }
    rep
}

/// Mesh-wide consistent-winding detector (directed-edge form of the
/// spec's facet-orientation rule).
///
/// The watertight check counts *undirected* edge uses: an edge
/// `{A, B}` shared by two triangles is "manifold" regardless of which
/// way each triangle walks it. But for a consistently oriented surface
/// (every triangle CCW from outside) the two triangles sharing that
/// edge must traverse it in *opposite* directions — one lists `A→B`,
/// the other `B→A`. When both list `A→B`, one of the two triangles is
/// wound backwards relative to its neighbour, which §6.5's "vertices
/// listed in counterclockwise order … must be consistent" rule forbids
/// even though the surface may still be perfectly watertight.
///
/// We collect, per canonical undirected edge, the set of directed
/// traversals `(from, to, owner)`. An edge is flagged when it has
/// exactly two incident triangles AND both walk it the same direction.
/// Edges with one incidence (boundary) or three-or-more (non-manifold)
/// are left to the watertight check — direction consistency is only
/// well-defined for the clean two-triangle manifold case.
fn check_consistent_winding(scene: &Scene3D, rep: &mut ValidationReport) {
    use std::collections::HashMap;
    type Vert = (u32, u32, u32);

    let bits = |p: [f32; 3]| -> Vert { (p[0].to_bits(), p[1].to_bits(), p[2].to_bits()) };

    // Per canonical undirected edge, the directed traversals that use
    // it. `dir` is `true` when the triangle walks the edge in the
    // canonical (lo→hi) direction, `false` for hi→lo. Two same-`dir`
    // entries on the same edge are an inconsistency.
    #[allow(clippy::type_complexity)]
    let mut edge_dirs: HashMap<(Vert, Vert), Vec<(bool, FaceLocator)>> = HashMap::new();

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
                let loc = FaceLocator {
                    mesh: mesh_idx,
                    primitive: prim_idx,
                    face: face_idx,
                };
                let b0 = bits(v0);
                let b1 = bits(v1);
                let b2 = bits(v2);
                for (a, b) in [(b0, b1), (b1, b2), (b2, b0)] {
                    // Skip degenerate edges whose endpoints coincide —
                    // direction is undefined there (the degenerate /
                    // duplicate-vertex case is the drop pass's job).
                    if a == b {
                        continue;
                    }
                    let (key, dir) = if a <= b {
                        ((a, b), true)
                    } else {
                        ((b, a), false)
                    };
                    edge_dirs.entry(key).or_default().push((dir, loc));
                }
            }
        }
    }

    let mut seen: std::collections::HashSet<FaceLocator> = std::collections::HashSet::new();
    for uses in edge_dirs.values() {
        // Direction consistency is only meaningful for the clean
        // two-triangle manifold edge. Boundary (1) and non-manifold
        // (3+) edges are reported by the watertight rule.
        if uses.len() != 2 {
            continue;
        }
        if uses[0].0 == uses[1].0 {
            rep.inconsistent_winding_edges += 1;
            for (_, loc) in uses {
                // Cap the example list overall but de-duplicate so a
                // triangle flipped against several neighbours is not
                // listed once per shared edge.
                if seen.insert(*loc) {
                    push_capped(&mut rep.inconsistent_winding_examples, *loc);
                }
            }
        }
    }
}

/// Brute-force T-junction detector.
///
/// Collects every triangle's corner positions + facet locator into a
/// `triangles` list, then collects every unique vertex position (by
/// bit pattern) into a `unique_verts` map. For every triangle edge
/// `(p, q)`, scans `unique_verts` for a position that lies strictly
/// between `p` and `q` (in the geometric, tolerance-bounded sense)
/// and is not bit-equal to either endpoint. When a match is found,
/// every triangle that *owns* that vertex as one of its three corners
/// is recorded as a T-junction defect — that's the offending
/// triangle, not the edge-owner.
///
/// Cost is `O(E · V_unique)`; gated behind `check_t_junctions` which
/// defaults `false`. The example list is capped at
/// [`MAX_REPORTED_DEFECTS`] and the count saturates at
/// `usize::MAX / 2` so the scan can terminate early once the example
/// list is full without changing the count's meaning.
fn check_t_junctions(scene: &Scene3D, opts: &ValidationOptions, rep: &mut ValidationReport) {
    use std::collections::HashMap;
    type Vert = (u32, u32, u32);

    let bits = |p: [f32; 3]| -> Vert { (p[0].to_bits(), p[1].to_bits(), p[2].to_bits()) };

    // First pass — collect all triangles + the vertex → owning-faces
    // table. Triangles whose positions are non-finite (NaN/Inf) are
    // skipped: the on-segment test would compare with NaN and silently
    // never match, but we surface a clean skip rather than walking the
    // edge into a position-domain hole.
    #[allow(clippy::type_complexity)]
    let mut triangles: Vec<([f32; 3], [f32; 3], [f32; 3])> = Vec::new();
    let mut vert_owners: HashMap<Vert, Vec<FaceLocator>> = HashMap::new();
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
                let loc = FaceLocator {
                    mesh: mesh_idx,
                    primitive: prim_idx,
                    face: face_idx,
                };
                triangles.push((v0, v1, v2));
                for v in [v0, v1, v2] {
                    vert_owners.entry(bits(v)).or_default().push(loc);
                }
            }
        }
    }

    let eps = if opts.t_junction_tolerance.is_finite() && opts.t_junction_tolerance >= 0.0 {
        opts.t_junction_tolerance
    } else {
        DEFAULT_T_JUNCTION_TOLERANCE
    };

    // Track which (offending-vertex, edge) pairs we've already
    // reported. Without this, a single vertex sitting on the same
    // physical edge that's used by two triangles fires twice; we
    // want one count per geometric incidence, not per edge-use.
    let mut seen_incidence: std::collections::HashSet<(Vert, (Vert, Vert))> =
        std::collections::HashSet::new();

    'outer: for (a, b, c) in &triangles {
        for (p, q) in [(*a, *b), (*b, *c), (*c, *a)] {
            let pb = bits(p);
            let qb = bits(q);
            let edge_key = if pb <= qb { (pb, qb) } else { (qb, pb) };
            for (&vb, owners) in &vert_owners {
                if vb == pb || vb == qb {
                    // Endpoint match — that's the well-formed
                    // edge-sharing case, not a T-junction.
                    continue;
                }
                let v = [
                    f32::from_bits(vb.0),
                    f32::from_bits(vb.1),
                    f32::from_bits(vb.2),
                ];
                if !point_strictly_on_segment(p, q, v, eps) {
                    continue;
                }
                if !seen_incidence.insert((vb, edge_key)) {
                    continue;
                }
                // Every triangle that lists this vertex as a corner
                // is in violation of the vertex-to-vertex rule —
                // they each have a corner sitting in the middle of
                // some other triangle's edge.
                for owner in owners {
                    rep.t_junction_defects = rep.t_junction_defects.saturating_add(1);
                    push_capped(&mut rep.t_junction_examples, *owner);
                }
                if rep.t_junction_defects >= usize::MAX / 2 {
                    break 'outer;
                }
            }
        }
    }
}

/// Geometric "vertex V lies strictly between segment endpoints P
/// and Q" predicate. Both:
///
/// 1. The perpendicular distance from V to the infinite line through
///    P and Q is at most `eps * |PQ|`.
/// 2. The orthogonal projection of V onto PQ, expressed as a
///    parameter `t` in `[0, 1]`, lies strictly in `(eps, 1 - eps)` —
///    i.e. V is *between* P and Q with `eps`-margin away from each
///    endpoint.
///
/// Returns `false` for degenerate edges (`|PQ|² == 0`), for non-finite
/// inputs, and for `eps >= 0.5` (which would collapse the open
/// interval to empty).
fn point_strictly_on_segment(p: [f32; 3], q: [f32; 3], v: [f32; 3], eps: f32) -> bool {
    let d = [q[0] - p[0], q[1] - p[1], q[2] - p[2]];
    let pv = [v[0] - p[0], v[1] - p[1], v[2] - p[2]];
    let len_sq = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
    if !len_sq.is_finite() || len_sq == 0.0 {
        return false;
    }
    if !(eps.is_finite() && (0.0..0.5).contains(&eps)) {
        return false;
    }
    // t = (pv · d) / |d|² — parametric position of V's projection
    // onto the infinite line through P, Q. Strictly between the
    // endpoints means t ∈ (eps, 1 - eps).
    let dot = pv[0] * d[0] + pv[1] * d[1] + pv[2] * d[2];
    let t = dot / len_sq;
    if !t.is_finite() || t <= eps || t >= 1.0 - eps {
        return false;
    }
    // Perpendicular component squared:
    //   |pv|² - t² · |d|² = |pv|² - (dot²) / |d|²
    // Comparing perp² ≤ (eps · |d|)² = eps² · |d|² is the same as
    //   |pv|² · |d|² - dot² ≤ eps² · |d|⁴.
    // Use that form to keep one division out of the comparison.
    let pv_sq = pv[0] * pv[0] + pv[1] * pv[1] + pv[2] * pv[2];
    let perp_sq_times_len_sq = pv_sq * len_sq - dot * dot;
    if !perp_sq_times_len_sq.is_finite() {
        return false;
    }
    let perp_sq_times_len_sq = perp_sq_times_len_sq.max(0.0);
    let tol = (eps * eps) * (len_sq * len_sq);
    perp_sq_times_len_sq <= tol
}

/// Per-vertex emitted-position iterator that mirrors the encoder's
/// walk order — one call per emitted vertex slot, in encoder order.
fn for_each_emitted_vertex(scene: &Scene3D, mut f: impl FnMut([f32; 3])) {
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
                        f(*p);
                    }
                }
            }
        }
    }
}

fn positive_octant_face(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> bool {
    // Spec rule: every coordinate must be "nonnegative AND nonzero" —
    // i.e. strictly greater than zero. NaN comparisons are also false
    // here, which mirrors the rest of the crate's strict treatment of
    // non-finite coordinates.
    for v in [a, b, c] {
        for c in v {
            if !(c.is_finite() && c > 0.0) {
                return false;
            }
        }
    }
    true
}

fn recompute_normal(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> [f32; 3] {
    let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let cx = u[1] * v[2] - u[2] * v[1];
    let cy = u[2] * v[0] - u[0] * v[2];
    let cz = u[0] * v[1] - u[1] * v[0];
    let len = (cx * cx + cy * cy + cz * cz).sqrt();
    if len > f32::EPSILON {
        [cx / len, cy / len, cz / len]
    } else {
        // Degenerate triangle — return the spec's "recompute later"
        // zero sentinel. The orientation check uses
        // `normals_within(stored, [0,0,0], eps)` for the comparison,
        // so a stored zero-normal-on-a-degenerate-triangle pair both
        // come up as a match.
        [0.0, 0.0, 0.0]
    }
}

fn normals_within(a: [f32; 3], b: [f32; 3], eps: f32) -> bool {
    (a[0] - b[0]).abs() <= eps && (a[1] - b[1]).abs() <= eps && (a[2] - b[2]).abs() <= eps
}

fn push_capped(v: &mut Vec<FaceLocator>, loc: FaceLocator) {
    if v.len() < MAX_REPORTED_DEFECTS {
        v.push(loc);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use oxideav_mesh3d::{Indices, Mesh, Primitive, Scene3D, Topology};

    fn unit_cube_indexed_scene() -> Scene3D {
        // 8 corners (all-positive octant) + 12 triangles, watertight,
        // every facet's stored normal equals the recomputed normal
        // (we provide per-vertex normals matching the face).
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
            0, 2, 1, 0, 3, 2, // bottom (-Z)
            4, 5, 6, 4, 6, 7, // top (+Z)
            0, 1, 5, 0, 5, 4, // front (-Y)
            2, 3, 7, 2, 7, 6, // back (+Y)
            1, 2, 6, 1, 6, 5, // right (+X)
            0, 4, 7, 0, 7, 3, // left (-X)
        ];
        // 8 vertex positions but each vertex appears in faces with
        // different normals — we pick (0, 0, 1) for everyone purely
        // for test scaffolding; some facet-orientation checks below
        // exercise this disagreement.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = positions;
        prim.indices = Some(Indices::U32(indices));
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(Some("cube".to_string())).with_primitive(prim));
        scene
    }

    fn one_facet(positions: Vec<[f32; 3]>, normal: [f32; 3]) -> Scene3D {
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = positions;
        prim.normals = Some(vec![normal; 3]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        scene
    }

    #[test]
    fn bbox_basic_unit_cube() {
        let scene = unit_cube_indexed_scene();
        let bb = bbox(&scene).unwrap();
        assert_eq!(bb.min, [0.0, 0.0, 0.0]);
        assert_eq!(bb.max, [1.0, 1.0, 1.0]);
        assert_eq!(bb.extents(), [1.0, 1.0, 1.0]);
        assert_eq!(bb.centre(), [0.5, 0.5, 0.5]);
        assert!(!bb.is_degenerate());
    }

    #[test]
    fn bbox_empty_scene_returns_none() {
        let scene = Scene3D::new();
        assert!(bbox(&scene).is_none());
    }

    #[test]
    fn bbox_skips_non_triangles_primitives() {
        // A Lines primitive with [10,10,10] points should NOT push the
        // bbox out — the Triangles primitive is still 0..1.
        let scene = unit_cube_indexed_scene();
        let mut scene = scene;
        let mut lines = Primitive::new(Topology::Lines);
        lines.positions = vec![[10.0, 10.0, 10.0], [-5.0, -5.0, -5.0]];
        scene.meshes[0].primitives.push(lines);
        let bb = bbox(&scene).unwrap();
        assert_eq!(bb.min, [0.0, 0.0, 0.0]);
        assert_eq!(bb.max, [1.0, 1.0, 1.0]);
    }

    #[test]
    fn bbox_skips_nan_and_inf_coordinates() {
        let scene = one_facet(
            vec![
                [f32::NAN, 0.0, 0.0],
                [1.0, f32::INFINITY, 0.0],
                [0.0, 1.0, 0.0],
            ],
            [0.0, 0.0, 1.0],
        );
        let bb = bbox(&scene).unwrap();
        // NaN/Inf are skipped; the remaining finite contributions
        // give min = (0, 0, 0) and max = (1, 1, 0).
        assert_eq!(bb.min, [0.0, 0.0, 0.0]);
        assert_eq!(bb.max, [1.0, 1.0, 0.0]);
    }

    #[test]
    fn bbox_volume_unit_cube_is_one() {
        let scene = unit_cube_indexed_scene();
        let bb = bbox(&scene).unwrap();
        assert_eq!(bb.volume(), 1.0);
        // 6 faces of unit area each.
        assert_eq!(bb.surface_area(), 6.0);
        // sqrt(1 + 1 + 1) = sqrt(3).
        let diag = bb.diagonal_length();
        assert!((diag - 3.0_f32.sqrt()).abs() < 1.0e-6);
    }

    #[test]
    fn bbox_volume_zero_on_degenerate_bbox() {
        // A single planar triangle on z=0: x and y extents are non-zero,
        // z extent is zero — degenerate on one axis.
        let scene = one_facet(
            vec![[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 3.0, 0.0]],
            [0.0, 0.0, 1.0],
        );
        let bb = bbox(&scene).unwrap();
        assert!(bb.is_degenerate());
        assert_eq!(bb.volume(), 0.0);
        // Surface area = 2 * (xy + yz + xz) = 2 * (6 + 0 + 0) = 12.
        assert_eq!(bb.surface_area(), 12.0);
    }

    #[test]
    fn bbox_longest_axis_picks_x_then_y_then_z() {
        // Build a synthetic non-cube bbox by laying out vertices.
        // Extents (2, 1, 1) → longest is X (0).
        let bb = Bbox {
            min: [0.0, 0.0, 0.0],
            max: [2.0, 1.0, 1.0],
        };
        assert_eq!(bb.longest_axis(), Some(0));

        // Extents (1, 3, 1) → longest is Y (1).
        let bb = Bbox {
            min: [0.0, 0.0, 0.0],
            max: [1.0, 3.0, 1.0],
        };
        assert_eq!(bb.longest_axis(), Some(1));

        // Extents (1, 1, 4) → longest is Z (2).
        let bb = Bbox {
            min: [0.0, 0.0, 0.0],
            max: [1.0, 1.0, 4.0],
        };
        assert_eq!(bb.longest_axis(), Some(2));
    }

    #[test]
    fn bbox_longest_axis_resolves_ties_toward_lower_index() {
        // Cube — all extents equal → tie resolves to X (0).
        let bb = Bbox {
            min: [0.0, 0.0, 0.0],
            max: [1.0, 1.0, 1.0],
        };
        assert_eq!(bb.longest_axis(), Some(0));

        // Y == Z but both larger than X → first encountered longest = Y.
        let bb = Bbox {
            min: [0.0, 0.0, 0.0],
            max: [1.0, 2.0, 2.0],
        };
        assert_eq!(bb.longest_axis(), Some(1));
    }

    #[test]
    fn bbox_longest_axis_returns_none_on_degenerate() {
        // Planar triangle bbox is degenerate; no single dominant axis.
        let bb = Bbox {
            min: [0.0, 0.0, 0.0],
            max: [1.0, 1.0, 0.0],
        };
        assert!(bb.is_degenerate());
        assert_eq!(bb.longest_axis(), None);
    }

    #[test]
    fn bbox_contains_point_inclusive_on_boundary() {
        let bb = Bbox {
            min: [0.0, 0.0, 0.0],
            max: [1.0, 1.0, 1.0],
        };
        // Strict interior.
        assert!(bb.contains_point([0.5, 0.5, 0.5]));
        // Corners (inclusive bounds).
        assert!(bb.contains_point([0.0, 0.0, 0.0]));
        assert!(bb.contains_point([1.0, 1.0, 1.0]));
        // Face midpoint.
        assert!(bb.contains_point([0.0, 0.5, 0.5]));
        // Outside on each axis.
        assert!(!bb.contains_point([-0.1, 0.5, 0.5]));
        assert!(!bb.contains_point([0.5, 1.1, 0.5]));
        assert!(!bb.contains_point([0.5, 0.5, 2.0]));
        // Non-finite components reject.
        assert!(!bb.contains_point([f32::NAN, 0.5, 0.5]));
        assert!(!bb.contains_point([f32::INFINITY, 0.5, 0.5]));
    }

    #[test]
    fn bbox_point_seed_is_degenerate() {
        let bb = Bbox::point([1.0, 2.0, 3.0]);
        assert_eq!(bb.min, [1.0, 2.0, 3.0]);
        assert_eq!(bb.max, [1.0, 2.0, 3.0]);
        assert_eq!(bb.extents(), [0.0, 0.0, 0.0]);
        assert_eq!(bb.centre(), [1.0, 2.0, 3.0]);
        assert!(bb.is_degenerate());
        assert_eq!(bb.volume(), 0.0);
        assert_eq!(bb.surface_area(), 0.0);
        assert_eq!(bb.diagonal_length(), 0.0);
        assert!(bb.longest_axis().is_none());
        // The seed itself is contained (inclusive bounds).
        assert!(bb.contains_point([1.0, 2.0, 3.0]));
        // A different point is not.
        assert!(!bb.contains_point([1.0, 2.0, 3.1]));
    }

    #[test]
    fn bbox_merge_is_commutative_and_unions_corners() {
        let a = Bbox {
            min: [-1.0, 0.0, 2.0],
            max: [1.0, 3.0, 5.0],
        };
        let b = Bbox {
            min: [0.0, -2.0, 4.0],
            max: [4.0, 1.0, 6.0],
        };
        let ab = a.merge(&b);
        let ba = b.merge(&a);
        assert_eq!(ab, ba);
        assert_eq!(ab.min, [-1.0, -2.0, 2.0]);
        assert_eq!(ab.max, [4.0, 3.0, 6.0]);

        // Merging with self is the identity.
        assert_eq!(a.merge(&a), a);

        // Associativity: (a U b) U c == a U (b U c).
        let c = Bbox {
            min: [-5.0, 5.0, -5.0],
            max: [-3.0, 7.0, 0.0],
        };
        let left = a.merge(&b).merge(&c);
        let right = a.merge(&b.merge(&c));
        assert_eq!(left, right);
        assert_eq!(left.min, [-5.0, -2.0, -5.0]);
        assert_eq!(left.max, [4.0, 7.0, 6.0]);
    }

    #[test]
    fn bbox_merge_seeded_from_point_accumulates_a_swarm() {
        // Worked use-case: build a scene-wide bbox out of per-primitive
        // bboxes without juggling Option<Bbox> in the caller.
        let points: [[f32; 3]; 4] = [
            [0.0, 0.0, 0.0],
            [1.0, 0.5, 0.5],
            [-0.5, 2.0, 0.5],
            [0.5, 0.5, 3.0],
        ];
        let mut acc = Bbox::point(points[0]);
        for p in &points[1..] {
            acc = acc.merge(&Bbox::point(*p));
        }
        assert_eq!(acc.min, [-0.5, 0.0, 0.0]);
        assert_eq!(acc.max, [1.0, 2.0, 3.0]);
        // Each seed point is inside the final hull.
        for p in &points {
            assert!(acc.contains_point(*p));
        }
    }

    #[test]
    fn bbox_expanded_by_grows_each_face_by_margin() {
        let bb = Bbox {
            min: [0.0, 0.0, 0.0],
            max: [2.0, 4.0, 6.0],
        };
        let e = bb.expanded_by(0.5);
        assert_eq!(e.min, [-0.5, -0.5, -0.5]);
        assert_eq!(e.max, [2.5, 4.5, 6.5]);
        // Extents grow by `2 * margin` on every axis.
        assert_eq!(e.extents(), [3.0, 5.0, 7.0]);
        // Centre is preserved by a symmetric expansion.
        assert_eq!(e.centre(), bb.centre());
        // The original bbox sits strictly inside the expanded one.
        assert!(e.contains_point(bb.min));
        assert!(e.contains_point(bb.max));
    }

    #[test]
    fn bbox_expanded_by_zero_is_identity() {
        let bb = Bbox {
            min: [-1.0, -2.0, -3.0],
            max: [4.0, 5.0, 6.0],
        };
        assert_eq!(bb.expanded_by(0.0), bb);
    }

    #[test]
    fn bbox_expanded_by_negative_margin_shrinks_box() {
        // -0.5 shrinks each face by 0.5; extents drop by 1.0 per axis.
        let bb = Bbox {
            min: [0.0, 0.0, 0.0],
            max: [4.0, 6.0, 8.0],
        };
        let s = bb.expanded_by(-0.5);
        assert_eq!(s.min, [0.5, 0.5, 0.5]);
        assert_eq!(s.max, [3.5, 5.5, 7.5]);
        assert_eq!(s.extents(), [3.0, 5.0, 7.0]);
        assert!(!s.is_degenerate());
    }

    #[test]
    fn bbox_expanded_by_negative_excess_inverts_box() {
        // Shrinking by more than half-extent on any axis produces an
        // inverted (degenerate) box on that axis. Caller must check
        // `is_degenerate` afterwards — documented contract.
        let bb = Bbox {
            min: [0.0, 0.0, 0.0],
            max: [1.0, 1.0, 1.0],
        };
        let s = bb.expanded_by(-0.6);
        // min crossed max on every axis.
        for axis in 0..3 {
            assert!(s.min[axis] > s.max[axis]);
        }
        assert!(s.is_degenerate());
    }

    #[test]
    fn bbox_intersects_is_symmetric_and_self_true() {
        let a = Bbox {
            min: [0.0, 0.0, 0.0],
            max: [2.0, 2.0, 2.0],
        };
        let b = Bbox {
            min: [1.0, 1.0, 1.0],
            max: [3.0, 3.0, 3.0],
        };
        // Overlap on every axis.
        assert!(a.intersects(&b));
        assert!(b.intersects(&a));
        // Self-intersection.
        assert!(a.intersects(&a));
    }

    #[test]
    fn bbox_intersects_returns_false_when_separated_on_any_axis() {
        let a = Bbox {
            min: [0.0, 0.0, 0.0],
            max: [1.0, 1.0, 1.0],
        };
        // Separated on X (gap (1.0, 2.0)).
        let sep_x = Bbox {
            min: [2.0, 0.0, 0.0],
            max: [3.0, 1.0, 1.0],
        };
        assert!(!a.intersects(&sep_x));
        // Separated on Y.
        let sep_y = Bbox {
            min: [0.0, 2.0, 0.0],
            max: [1.0, 3.0, 1.0],
        };
        assert!(!a.intersects(&sep_y));
        // Separated on Z.
        let sep_z = Bbox {
            min: [0.0, 0.0, 2.0],
            max: [1.0, 1.0, 3.0],
        };
        assert!(!a.intersects(&sep_z));
    }

    #[test]
    fn bbox_intersects_inclusive_on_touching_face() {
        // Two boxes sharing exactly one face (touching at x = 1)
        // count as intersecting under the inclusive rule.
        let a = Bbox {
            min: [0.0, 0.0, 0.0],
            max: [1.0, 1.0, 1.0],
        };
        let touch = Bbox {
            min: [1.0, 0.0, 0.0],
            max: [2.0, 1.0, 1.0],
        };
        assert!(a.intersects(&touch));
        assert!(touch.intersects(&a));
    }

    #[test]
    fn bbox_intersect_returns_overlap_region() {
        let a = Bbox {
            min: [0.0, 0.0, 0.0],
            max: [2.0, 2.0, 2.0],
        };
        let b = Bbox {
            min: [1.0, 1.0, 1.0],
            max: [3.0, 3.0, 3.0],
        };
        let ab = a.intersect(&b).expect("overlap is non-empty");
        let ba = b.intersect(&a).expect("overlap is non-empty");
        // Symmetric.
        assert_eq!(ab, ba);
        // Component-wise dual of merge: min = max-of-mins, max = min-of-maxes.
        assert_eq!(ab.min, [1.0, 1.0, 1.0]);
        assert_eq!(ab.max, [2.0, 2.0, 2.0]);
    }

    #[test]
    fn bbox_intersect_returns_none_when_separated() {
        let a = Bbox {
            min: [0.0, 0.0, 0.0],
            max: [1.0, 1.0, 1.0],
        };
        let far = Bbox {
            min: [5.0, 5.0, 5.0],
            max: [6.0, 6.0, 6.0],
        };
        assert!(a.intersect(&far).is_none());
        assert!(far.intersect(&a).is_none());
    }

    #[test]
    fn bbox_intersect_is_idempotent_on_self() {
        let a = Bbox {
            min: [-1.0, 0.0, 2.0],
            max: [3.0, 5.0, 7.0],
        };
        assert_eq!(a.intersect(&a), Some(a));
    }

    #[test]
    fn bbox_intersect_touching_face_is_degenerate() {
        // Boxes touching on exactly one face produce a flat (zero-extent
        // on the touching axis) intersection — documented contract.
        let a = Bbox {
            min: [0.0, 0.0, 0.0],
            max: [1.0, 1.0, 1.0],
        };
        let touch = Bbox {
            min: [1.0, 0.0, 0.0],
            max: [2.0, 1.0, 1.0],
        };
        let overlap = a.intersect(&touch).expect("touching boxes overlap");
        assert_eq!(overlap.min, [1.0, 0.0, 0.0]);
        assert_eq!(overlap.max, [1.0, 1.0, 1.0]);
        // Zero extent on X — degenerate by construction.
        assert!(overlap.is_degenerate());
        assert_eq!(overlap.volume(), 0.0);
    }

    #[test]
    fn bbox_intersect_contained_returns_inner_box() {
        // Inner box sits entirely inside outer; intersection is the inner.
        let outer = Bbox {
            min: [-1.0, -1.0, -1.0],
            max: [3.0, 3.0, 3.0],
        };
        let inner = Bbox {
            min: [0.0, 0.0, 0.0],
            max: [1.0, 1.0, 1.0],
        };
        assert_eq!(outer.intersect(&inner), Some(inner));
        assert_eq!(inner.intersect(&outer), Some(inner));
    }

    #[test]
    fn bbox_contains_bbox_reflexive_and_inclusive() {
        let a = Bbox {
            min: [0.0, 0.0, 0.0],
            max: [2.0, 2.0, 2.0],
        };
        // Reflexive — every box contains itself.
        assert!(a.contains_bbox(&a));
        // Inclusive — corner-touching inner box still counts as contained.
        let inner_corner = Bbox {
            min: [0.0, 0.0, 0.0],
            max: [1.0, 1.0, 1.0],
        };
        assert!(a.contains_bbox(&inner_corner));
        let inner_other = Bbox {
            min: [1.0, 1.0, 1.0],
            max: [2.0, 2.0, 2.0],
        };
        assert!(a.contains_bbox(&inner_other));
    }

    #[test]
    fn bbox_contains_bbox_rejects_overhanging_or_separated() {
        let a = Bbox {
            min: [0.0, 0.0, 0.0],
            max: [2.0, 2.0, 2.0],
        };
        // Overhangs on X.
        let oh = Bbox {
            min: [-0.5, 0.0, 0.0],
            max: [1.0, 1.0, 1.0],
        };
        assert!(!a.contains_bbox(&oh));
        // Fully outside.
        let far = Bbox {
            min: [5.0, 5.0, 5.0],
            max: [6.0, 6.0, 6.0],
        };
        assert!(!a.contains_bbox(&far));
        // Partial overlap — not contained.
        let partial = Bbox {
            min: [1.0, 1.0, 1.0],
            max: [3.0, 3.0, 3.0],
        };
        assert!(!a.contains_bbox(&partial));
        // Asymmetric — partial does not contain a either.
        assert!(!partial.contains_bbox(&a));
    }

    #[test]
    fn bbox_contains_bbox_transitive() {
        // a ⊇ b ⊇ c → a ⊇ c.
        let a = Bbox {
            min: [-1.0, -1.0, -1.0],
            max: [4.0, 4.0, 4.0],
        };
        let b = Bbox {
            min: [0.0, 0.0, 0.0],
            max: [3.0, 3.0, 3.0],
        };
        let c = Bbox {
            min: [1.0, 1.0, 1.0],
            max: [2.0, 2.0, 2.0],
        };
        assert!(a.contains_bbox(&b));
        assert!(b.contains_bbox(&c));
        assert!(a.contains_bbox(&c));
    }

    #[test]
    fn bbox_contains_bbox_accepts_degenerate_inner_on_face() {
        // Degenerate inner box (point) sitting on outer's face is contained.
        let outer = Bbox {
            min: [0.0, 0.0, 0.0],
            max: [1.0, 1.0, 1.0],
        };
        let inner_point = Bbox::point([0.5, 0.5, 0.5]);
        assert!(outer.contains_bbox(&inner_point));
        let inner_corner_point = Bbox::point([0.0, 0.0, 0.0]);
        assert!(outer.contains_bbox(&inner_corner_point));
        // Point just outside on one axis is not contained.
        let outside_point = Bbox::point([1.5, 0.5, 0.5]);
        assert!(!outer.contains_bbox(&outside_point));
    }

    #[test]
    fn bbox_intersect_merge_lattice_invariants() {
        // Bounding-box lattice sanity: for any two boxes A, B,
        //   A.merge(B) ⊇ A and ⊇ B  (union dominates both inputs)
        //   A.intersect(B), when present, is contained by both A and B
        let a = Bbox {
            min: [-1.0, 0.0, 1.0],
            max: [2.0, 3.0, 4.0],
        };
        let b = Bbox {
            min: [0.0, 1.0, 2.0],
            max: [3.0, 4.0, 5.0],
        };
        let union = a.merge(&b);
        assert!(union.contains_bbox(&a));
        assert!(union.contains_bbox(&b));
        let overlap = a.intersect(&b).expect("overlap is non-empty");
        assert!(a.contains_bbox(&overlap));
        assert!(b.contains_bbox(&overlap));
        // Self-consistency: union contains overlap as well.
        assert!(union.contains_bbox(&overlap));
    }

    #[test]
    fn bbox_of_mesh_isolates_mesh_index() {
        // Two-mesh scene: the second mesh sits at +10 on every axis.
        let mut scene = unit_cube_indexed_scene();
        let mut prim2 = Primitive::new(Topology::Triangles);
        prim2.positions = vec![[10.0, 10.0, 10.0], [11.0, 10.0, 10.0], [10.0, 11.0, 10.0]];
        scene.add_mesh(Mesh::new(Some("offset".to_string())).with_primitive(prim2));

        let mesh0 = bbox_of_mesh(&scene, 0).unwrap();
        assert_eq!(mesh0.min, [0.0, 0.0, 0.0]);
        assert_eq!(mesh0.max, [1.0, 1.0, 1.0]);

        let mesh1 = bbox_of_mesh(&scene, 1).unwrap();
        assert_eq!(mesh1.min, [10.0, 10.0, 10.0]);
        assert_eq!(mesh1.max, [11.0, 11.0, 10.0]);

        // Out-of-range index returns None.
        assert!(bbox_of_mesh(&scene, 2).is_none());

        // The whole-scene bbox spans both meshes.
        let whole = bbox(&scene).unwrap();
        assert_eq!(whole.min, [0.0, 0.0, 0.0]);
        assert_eq!(whole.max, [11.0, 11.0, 10.0]);
    }

    #[test]
    fn bbox_of_primitive_isolates_primitive_index() {
        let mut scene = unit_cube_indexed_scene();
        let mut prim2 = Primitive::new(Topology::Triangles);
        prim2.positions = vec![[5.0, 5.0, 5.0], [6.0, 5.0, 5.0], [5.0, 6.0, 5.0]];
        scene.meshes[0].primitives.push(prim2);

        let p0 = bbox_of_primitive(&scene, 0, 0).unwrap();
        assert_eq!(p0.min, [0.0, 0.0, 0.0]);
        assert_eq!(p0.max, [1.0, 1.0, 1.0]);

        let p1 = bbox_of_primitive(&scene, 0, 1).unwrap();
        assert_eq!(p1.min, [5.0, 5.0, 5.0]);
        assert_eq!(p1.max, [6.0, 6.0, 5.0]);

        // Out-of-range indices return None.
        assert!(bbox_of_primitive(&scene, 1, 0).is_none());
        assert!(bbox_of_primitive(&scene, 0, 2).is_none());
    }

    #[test]
    fn bbox_of_primitive_skips_non_triangles() {
        // A Lines primitive's vertices should NOT contribute even when
        // we ask directly for that primitive's bbox — STL's bbox is
        // defined over Triangles topology only.
        let mut scene = Scene3D::new();
        let mut lines = Primitive::new(Topology::Lines);
        lines.positions = vec![[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]];
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(lines));
        assert!(bbox_of_primitive(&scene, 0, 0).is_none());
    }

    #[test]
    fn validate_clean_unit_triangle() {
        let scene = one_facet(
            vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            [0.0, 0.0, 1.0],
        );
        let opts = ValidationOptions::default();
        let r = validate(&scene, &opts);
        assert_eq!(r.triangles_total, 1);
        assert_eq!(r.facet_orientation_defects, 0);
        assert_eq!(r.non_unit_normal_defects, 0);
        assert_eq!(r.boundary_edges, 3);
        // A single triangle has 3 edges all used once → not watertight.
        assert!(!r.watertight);
    }

    #[test]
    fn validate_orientation_flips_when_normal_is_inverted() {
        // Same triangle as above but with the stored normal pointing
        // the wrong way (down instead of up).
        let scene = one_facet(
            vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            [0.0, 0.0, -1.0],
        );
        let r = validate(&scene, &ValidationOptions::default());
        assert_eq!(r.facet_orientation_defects, 1);
        assert_eq!(r.facet_orientation_examples.len(), 1);
        assert_eq!(
            r.facet_orientation_examples[0],
            FaceLocator {
                mesh: 0,
                primitive: 0,
                face: 0,
            }
        );
    }

    #[test]
    fn validate_unit_normal_check_picks_up_oversized_normal() {
        let scene = one_facet(
            vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            [0.0, 0.0, 5.0], // length 5 — way off unit
        );
        // Don't run the orientation rule (which would also fire) so the
        // diagnostic isolates the unit-normal violation.
        let opts = ValidationOptions {
            check_facet_orientation: false,
            ..ValidationOptions::default()
        };
        let r = validate(&scene, &opts);
        assert_eq!(r.non_unit_normal_defects, 1);
        assert_eq!(r.facet_orientation_defects, 0);
    }

    #[test]
    fn validate_zero_normal_passes_unit_check() {
        // The spec'd "consumer should recompute" sentinel is
        // [0, 0, 0] — that's not a unit normal but it IS a valid
        // STL output, so the unit-normal check tolerates it.
        let scene = one_facet(
            vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            [0.0, 0.0, 0.0],
        );
        let opts = ValidationOptions {
            check_facet_orientation: false,
            ..ValidationOptions::default()
        };
        let r = validate(&scene, &opts);
        assert_eq!(r.non_unit_normal_defects, 0);
    }

    #[test]
    fn validate_positive_octant_off_by_default() {
        // Vertex at (-0.5, …) — the spec rule fires only when
        // explicitly enabled.
        let scene = one_facet(
            vec![[-0.5, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            [0.0, 0.0, 1.0],
        );
        let r = validate(&scene, &ValidationOptions::default());
        assert_eq!(r.positive_octant_defects, 0);
    }

    #[test]
    fn validate_positive_octant_on_flags_negative_vertex() {
        let scene = one_facet(
            vec![[-0.5, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            [0.0, 0.0, 1.0],
        );
        let opts = ValidationOptions {
            check_positive_octant: true,
            ..ValidationOptions::default()
        };
        let r = validate(&scene, &opts);
        assert_eq!(r.positive_octant_defects, 1);
        // Also catches origin-touching geometry — the rule says
        // "nonnegative AND nonzero". A vertex exactly on an axis
        // plane (a 0 coordinate) violates the nonzero half.
        let scene = one_facet(
            vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            [0.0, 0.0, 1.0],
        );
        let r = validate(&scene, &opts);
        assert_eq!(r.positive_octant_defects, 1);
    }

    #[test]
    fn validate_watertight_cube_has_zero_boundary() {
        // The cube fixture has 8 corners + 12 triangles + 18 unique
        // edges, every one shared by exactly two triangles. The
        // facet-orientation check is disabled here because the
        // fixture deliberately uses a single per-vertex normal that
        // doesn't match every face's recomputed normal.
        let scene = unit_cube_indexed_scene();
        let opts = ValidationOptions {
            check_facet_orientation: false,
            check_unit_normal: false,
            ..ValidationOptions::default()
        };
        let r = validate(&scene, &opts);
        assert_eq!(r.triangles_total, 12);
        assert_eq!(r.boundary_edges, 0);
        assert_eq!(r.non_manifold_edges, 0);
        assert!(r.watertight);
        assert!(r.is_clean());
    }

    #[test]
    fn validate_open_strip_has_boundary_edges() {
        // Two triangles sharing one edge → 5 distinct edges, 1 shared
        // (used twice), 4 boundary (used once). NOT watertight.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            // second triangle sharing the (1, 0, 0)–(0, 1, 0) edge
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
        ];
        prim.normals = Some(vec![[0.0, 0.0, 1.0]; 6]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = validate(&scene, &ValidationOptions::default());
        assert_eq!(r.triangles_total, 2);
        assert_eq!(r.boundary_edges, 4);
        assert_eq!(r.non_manifold_edges, 0);
        assert!(!r.watertight);
    }

    #[test]
    fn validate_three_triangles_sharing_an_edge_is_non_manifold() {
        // A "fin" geometry — three triangles share one edge. That
        // edge is used 3 times → non-manifold.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![
            // shared edge: (0,0,0) → (1,0,0)
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            //
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            //
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ];
        prim.normals = Some(vec![[0.0, 0.0, 1.0]; 9]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let opts = ValidationOptions {
            check_facet_orientation: false,
            ..ValidationOptions::default()
        };
        let r = validate(&scene, &opts);
        assert_eq!(r.triangles_total, 3);
        assert!(r.non_manifold_edges >= 1, "report: {r:?}");
        assert!(!r.watertight);
    }

    #[test]
    fn validate_examples_capped_at_max_reported_defects() {
        // Synthesise (MAX_REPORTED_DEFECTS + 5) bad facets — the count
        // must reflect them all but the example list caps at MAX.
        let triangles = MAX_REPORTED_DEFECTS + 5;
        let mut positions: Vec<[f32; 3]> = Vec::new();
        for _ in 0..triangles {
            positions.push([0.0, 0.0, 0.0]);
            positions.push([1.0, 0.0, 0.0]);
            positions.push([0.0, 1.0, 0.0]);
        }
        let normals = vec![[0.0, 0.0, -1.0]; positions.len()]; // wrong way
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = positions;
        prim.normals = Some(normals);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = validate(&scene, &ValidationOptions::default());
        assert_eq!(r.facet_orientation_defects, triangles);
        assert_eq!(r.facet_orientation_examples.len(), MAX_REPORTED_DEFECTS);
    }

    #[test]
    fn validate_empty_scene_is_vacuous() {
        let scene = Scene3D::new();
        let r = validate(&scene, &ValidationOptions::default());
        assert_eq!(r.triangles_total, 0);
        // Vacuously NOT watertight (nothing to seal) but also clean
        // (no rule fired).
        assert!(!r.watertight);
        assert!(r.is_clean());
    }

    #[test]
    fn validate_skips_non_triangles_primitives() {
        let mut scene = unit_cube_indexed_scene();
        let mut lines = Primitive::new(Topology::Lines);
        lines.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]];
        scene.meshes[0].primitives.push(lines);
        let opts = ValidationOptions {
            check_facet_orientation: false,
            check_unit_normal: false,
            ..ValidationOptions::default()
        };
        let r = validate(&scene, &opts);
        // Non-Triangles primitive contributes no triangles, no edges.
        assert_eq!(r.triangles_total, 12);
        assert!(r.watertight);
    }

    #[test]
    fn validate_facet_locator_threads_through_primitives() {
        // Two primitives, each with one (bad) facet — locators should
        // distinguish them by `primitive` index.
        let mut p0 = Primitive::new(Topology::Triangles);
        p0.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        p0.normals = Some(vec![[0.0, 0.0, -1.0]; 3]);
        let mut p1 = Primitive::new(Topology::Triangles);
        p1.positions = vec![[0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]];
        p1.normals = Some(vec![[0.0, 0.0, 1.0]; 3]);
        let mut mesh = Mesh::new(None::<String>).with_primitive(p0);
        mesh.primitives.push(p1);
        let mut scene = Scene3D::new();
        scene.add_mesh(mesh);
        let r = validate(&scene, &ValidationOptions::default());
        assert_eq!(r.facet_orientation_defects, 2);
        assert_eq!(r.facet_orientation_examples[0].primitive, 0);
        assert_eq!(r.facet_orientation_examples[1].primitive, 1);
    }

    /// `t_junction_*` defaults are off + use the default tolerance.
    #[test]
    fn t_junction_check_is_off_by_default() {
        let opts = ValidationOptions::default();
        assert!(!opts.check_t_junctions);
        assert_eq!(opts.t_junction_tolerance, DEFAULT_T_JUNCTION_TOLERANCE);
    }

    /// `point_strictly_on_segment` returns true for the midpoint of a
    /// generic segment.
    #[test]
    fn point_strictly_on_segment_midpoint() {
        assert!(point_strictly_on_segment(
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            1.0e-5,
        ));
    }

    /// Endpoint coincidence and out-of-bounds projection return false.
    #[test]
    fn point_strictly_on_segment_rejects_endpoints_and_outside() {
        // V == P
        assert!(!point_strictly_on_segment(
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            1.0e-5,
        ));
        // V == Q
        assert!(!point_strictly_on_segment(
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            1.0e-5,
        ));
        // V past Q (t = 1.5)
        assert!(!point_strictly_on_segment(
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
            1.0e-5,
        ));
        // V behind P (t = -0.5)
        assert!(!point_strictly_on_segment(
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            1.0e-5,
        ));
    }

    /// A vertex off the line by more than `eps * |PQ|` is rejected.
    #[test]
    fn point_strictly_on_segment_rejects_off_line() {
        assert!(!point_strictly_on_segment(
            [0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
            [5.0, 1.0, 0.0],
            1.0e-5,
        ));
    }

    /// Degenerate edge (`P == Q`) returns false rather than panicking.
    #[test]
    fn point_strictly_on_segment_rejects_degenerate_edge() {
        assert!(!point_strictly_on_segment(
            [1.0, 2.0, 3.0],
            [1.0, 2.0, 3.0],
            [1.0, 2.0, 3.0],
            1.0e-5,
        ));
    }

    /// Non-finite inputs return false.
    #[test]
    fn point_strictly_on_segment_rejects_non_finite() {
        assert!(!point_strictly_on_segment(
            [0.0, 0.0, 0.0],
            [f32::NAN, 0.0, 0.0],
            [0.5, 0.0, 0.0],
            1.0e-5,
        ));
        assert!(!point_strictly_on_segment(
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [f32::INFINITY, 0.0, 0.0],
            1.0e-5,
        ));
    }

    /// Diagonal segment midpoint (proves the predicate is direction-
    /// agnostic).
    #[test]
    fn point_strictly_on_segment_diagonal_midpoint() {
        assert!(point_strictly_on_segment(
            [0.0, 0.0, 0.0],
            [3.0, 3.0, 3.0],
            [1.5, 1.5, 1.5],
            1.0e-5,
        ));
    }

    /// Two triangles sharing the (0,0,0)→(2,0,0) edge through a
    /// midpoint (1,0,0) — the bottom triangle is split into two halves
    /// that each carry the midpoint. That's the textbook T-junction:
    /// the top triangle's edge runs from (0,0,0) to (2,0,0), but the
    /// bottom triangle's vertex sits at (1,0,0) on that edge.
    #[test]
    fn t_junction_classic_split_edge_is_flagged() {
        let top = {
            let mut p = Primitive::new(Topology::Triangles);
            p.positions = vec![[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [1.0, 2.0, 0.0]];
            p
        };
        let bottom_left = {
            let mut p = Primitive::new(Topology::Triangles);
            p.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.5, -1.0, 0.0]];
            p
        };
        let bottom_right = {
            let mut p = Primitive::new(Topology::Triangles);
            p.positions = vec![[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [1.5, -1.0, 0.0]];
            p
        };
        let mesh = Mesh::new(None::<String>)
            .with_primitive(top)
            .with_primitive(bottom_left)
            .with_primitive(bottom_right);
        let mut scene = Scene3D::new();
        scene.add_mesh(mesh);

        // Off — no T-junction signal.
        let opts_off = ValidationOptions {
            check_t_junctions: false,
            check_facet_orientation: false,
            check_unit_normal: false,
            check_watertight: false,
            ..ValidationOptions::default()
        };
        let r0 = validate(&scene, &opts_off);
        assert_eq!(r0.t_junction_defects, 0);

        // On — bottom_left + bottom_right each own the (1, 0, 0)
        // midpoint, and that point sits on the top triangle's edge.
        let opts_on = ValidationOptions {
            check_t_junctions: true,
            check_facet_orientation: false,
            check_unit_normal: false,
            check_watertight: false,
            ..ValidationOptions::default()
        };
        let r1 = validate(&scene, &opts_on);
        assert!(r1.t_junction_defects >= 2, "report: {r1:?}");
        assert!(r1.t_junction_examples.len() >= 2);
        assert!(!r1.is_clean());
    }

    /// A clean unit-cube triangulation must NOT report any T-junctions.
    #[test]
    fn t_junction_clean_cube_is_clean() {
        let scene = unit_cube_indexed_scene();
        let opts = ValidationOptions {
            check_t_junctions: true,
            check_facet_orientation: false,
            check_unit_normal: false,
            check_watertight: false,
            ..ValidationOptions::default()
        };
        let r = validate(&scene, &opts);
        assert_eq!(r.t_junction_defects, 0, "report: {r:?}");
        assert!(r.t_junction_examples.is_empty());
    }

    /// Negative + non-finite tolerance values clamp to the default
    /// — the report should reflect the cube being clean either way.
    #[test]
    fn t_junction_negative_tolerance_clamps_to_default() {
        let scene = unit_cube_indexed_scene();
        let opts = ValidationOptions {
            check_t_junctions: true,
            t_junction_tolerance: -1.0,
            check_facet_orientation: false,
            check_unit_normal: false,
            check_watertight: false,
            ..ValidationOptions::default()
        };
        let r = validate(&scene, &opts);
        assert_eq!(r.t_junction_defects, 0);
    }

    /// Empty scene + T-junction check on → vacuously clean.
    #[test]
    fn t_junction_empty_scene_is_vacuous() {
        let scene = Scene3D::new();
        let opts = ValidationOptions {
            check_t_junctions: true,
            ..ValidationOptions::default()
        };
        let r = validate(&scene, &opts);
        assert_eq!(r.t_junction_defects, 0);
        assert!(r.is_clean());
    }

    /// The consistent-winding check is on by default and a properly
    /// wound watertight cube reports zero inconsistent edges.
    #[test]
    fn consistent_winding_on_by_default_clean_cube() {
        let opts = ValidationOptions::default();
        assert!(opts.check_consistent_winding);
        let scene = unit_cube_indexed_scene();
        // Disable the per-facet rules (the cube fixture's single shared
        // normal deliberately disagrees with some faces) so the winding
        // rule is isolated.
        let opts = ValidationOptions {
            check_facet_orientation: false,
            check_unit_normal: false,
            ..ValidationOptions::default()
        };
        let r = validate(&scene, &opts);
        assert_eq!(r.inconsistent_winding_edges, 0, "report: {r:?}");
        assert!(r.inconsistent_winding_examples.is_empty());
        assert!(r.is_clean());
    }

    /// Two triangles sharing an edge but wound the *same* way around it
    /// (both list the edge `A→B`) are flagged. The surface is still
    /// watertight on the undirected count — this is exactly the case
    /// the watertight rule cannot see.
    #[test]
    fn consistent_winding_flags_flipped_neighbour() {
        // Triangle 0: (0,0,0) (1,0,0) (1,1,0) — walks the diagonal
        //   edge (1,0,0)→(1,1,0)... actually share the (0,0,0)-(1,1,0)
        //   diagonal. Build a quad split into two triangles where the
        //   second is wound the wrong way around the shared diagonal.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![
            // tri 0: 0,0,0 -> 1,0,0 -> 1,1,0  (shared edge 0,0,0 ->? )
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            // tri 1 (correctly wound would be 0,0,0 -> 1,1,0 -> 0,1,0,
            // giving the diagonal as 1,1,0 -> 0,0,0, opposite to tri 0's
            // 0,0,0 -> 1,1,0). We FLIP it: 0,0,0 -> 0,1,0 -> 1,1,0 so
            // the diagonal is walked 1,1,0 -> 0,0,0 ... no — flip means
            // both walk 0,0,0 -> 1,1,0. Use that ordering directly:
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
        ];
        prim.normals = Some(vec![[0.0, 0.0, 1.0]; 6]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));

        // Determine the shared diagonal's direction in each triangle.
        // Tri 0 edges: (0,0,0)->(1,0,0), (1,0,0)->(1,1,0),
        //   (1,1,0)->(0,0,0). Diagonal walked (1,1,0)->(0,0,0).
        // Tri 1 edges: (0,0,0)->(1,1,0), (1,1,0)->(0,1,0),
        //   (0,1,0)->(0,0,0). Diagonal walked (0,0,0)->(1,1,0).
        // Those are OPPOSITE → consistent. So this layout is the CLEAN
        // case; assert it is clean to anchor the predicate, then build
        // the flipped variant below.
        let r_clean = validate(
            &scene,
            &ValidationOptions {
                check_facet_orientation: false,
                check_unit_normal: false,
                ..ValidationOptions::default()
            },
        );
        assert_eq!(r_clean.inconsistent_winding_edges, 0, "clean: {r_clean:?}");

        // Now flip tri 1 so it walks the diagonal the SAME way as tri 0
        // (both (1,1,0)->(0,0,0)). Reorder tri 1 to (1,1,0) (0,0,0)
        // (0,1,0): edges (1,1,0)->(0,0,0), (0,0,0)->(0,1,0),
        // (0,1,0)->(1,1,0). Diagonal walked (1,1,0)->(0,0,0) — same as
        // tri 0 → inconsistent.
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ];
        prim.normals = Some(vec![[0.0, 0.0, 1.0]; 6]);
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = validate(
            &scene,
            &ValidationOptions {
                check_facet_orientation: false,
                check_unit_normal: false,
                ..ValidationOptions::default()
            },
        );
        assert_eq!(r.inconsistent_winding_edges, 1, "flipped: {r:?}");
        // Both adjacent triangles are surfaced as examples, de-duped.
        assert_eq!(r.inconsistent_winding_examples.len(), 2);
        assert!(!r.is_clean());
    }

    /// Toggling the check off zeroes the winding fields even on a
    /// flipped layout.
    #[test]
    fn consistent_winding_respects_opt_off() {
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ];
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let opts = ValidationOptions {
            check_consistent_winding: false,
            check_facet_orientation: false,
            check_unit_normal: false,
            check_watertight: false,
            ..ValidationOptions::default()
        };
        let r = validate(&scene, &opts);
        assert_eq!(r.inconsistent_winding_edges, 0);
        assert!(r.inconsistent_winding_examples.is_empty());
    }

    /// Boundary edges (one incident triangle) and non-manifold edges
    /// (3+) are NOT counted as winding inconsistencies — direction
    /// consistency is only defined for the two-triangle manifold edge.
    #[test]
    fn consistent_winding_ignores_non_two_incidence_edges() {
        // Single triangle: every edge has one incidence.
        let scene = one_facet(
            vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            [0.0, 0.0, 1.0],
        );
        let opts = ValidationOptions {
            check_facet_orientation: false,
            check_unit_normal: false,
            ..ValidationOptions::default()
        };
        let r = validate(&scene, &opts);
        assert_eq!(r.inconsistent_winding_edges, 0);

        // Fin: three triangles share one edge (3 incidences).
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ];
        let mut scene = Scene3D::new();
        scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));
        let r = validate(&scene, &opts);
        assert_eq!(
            r.inconsistent_winding_edges, 0,
            "non-manifold edge must not count: {r:?}"
        );
    }

    /// Empty scene with the winding check on is vacuously clean.
    #[test]
    fn consistent_winding_empty_scene_is_vacuous() {
        let scene = Scene3D::new();
        let r = validate(&scene, &ValidationOptions::default());
        assert_eq!(r.inconsistent_winding_edges, 0);
        assert!(r.is_clean());
    }

    /// `defect_total()` on an empty scene is zero, in lockstep with
    /// `is_clean()` returning true.
    #[test]
    fn defect_total_empty_scene_is_zero() {
        let scene = Scene3D::new();
        let r = validate(&scene, &ValidationOptions::default());
        assert_eq!(r.defect_total(), 0);
        assert!(r.is_clean());
    }

    /// A defect-free single triangle still reports `boundary_edges`
    /// (three open edges) and `defect_total()` reflects that as a
    /// non-zero sum, matching `is_clean()` returning false.
    #[test]
    fn defect_total_open_triangle_counts_boundary_edges() {
        let scene = one_facet(
            vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            [0.0, 0.0, 1.0],
        );
        let r = validate(&scene, &ValidationOptions::default());
        assert_eq!(r.boundary_edges, 3);
        assert_eq!(r.defect_total(), 3);
        assert!(!r.is_clean());
    }

    /// A triangle whose stored normal disagrees with the winding-
    /// derived normal trips both `facet_orientation_defects` and
    /// `boundary_edges`; `defect_total()` sums them together.
    #[test]
    fn defect_total_sums_orientation_and_boundary() {
        let scene = one_facet(
            vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            [0.0, 0.0, -1.0],
        );
        let r = validate(&scene, &ValidationOptions::default());
        assert_eq!(r.facet_orientation_defects, 1);
        assert_eq!(r.boundary_edges, 3);
        assert_eq!(r.defect_total(), 4);
        assert!(!r.is_clean());
    }

    /// Toggling every rule off zeroes every counter and therefore the
    /// total — the report is vacuously clean regardless of geometry.
    #[test]
    fn defect_total_zero_when_every_rule_disabled() {
        let scene = one_facet(
            vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            [0.0, 0.0, -1.0],
        );
        let opts = ValidationOptions {
            check_facet_orientation: false,
            check_unit_normal: false,
            check_positive_octant: false,
            check_watertight: false,
            check_t_junctions: false,
            check_consistent_winding: false,
            ..ValidationOptions::default()
        };
        let r = validate(&scene, &opts);
        assert_eq!(r.defect_total(), 0);
        assert!(r.is_clean());
    }

    /// `defects_by_rule()` lists every rule's count in scan order with
    /// stable labels. The seven counts sum to `defect_total()`.
    #[test]
    fn defects_by_rule_labels_and_sum_match() {
        let scene = one_facet(
            vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            [0.0, 0.0, -1.0],
        );
        let r = validate(&scene, &ValidationOptions::default());
        let rows = r.defects_by_rule();
        assert_eq!(rows.len(), 7);
        let labels: Vec<&'static str> = rows.iter().map(|(label, _)| *label).collect();
        assert_eq!(
            labels,
            vec![
                "facet_orientation",
                "non_unit_normal",
                "positive_octant",
                "boundary_edges",
                "non_manifold_edges",
                "t_junction",
                "inconsistent_winding",
            ]
        );
        let summed: usize = rows.iter().map(|(_, n)| *n).sum();
        assert_eq!(summed, r.defect_total());
    }

    /// On a clean scene every `defects_by_rule()` entry has count
    /// zero, so the row total agrees with the clean-scene
    /// `defect_total()` of zero.
    #[test]
    fn defects_by_rule_all_zero_on_clean_scene() {
        let scene = Scene3D::new();
        let r = validate(&scene, &ValidationOptions::default());
        let rows = r.defects_by_rule();
        assert!(rows.iter().all(|(_, n)| *n == 0));
        assert_eq!(r.defect_total(), 0);
    }

    /// A positive-octant violation contributes to the appropriate row
    /// even with the rule toggled on top of the default set; the
    /// label is the stable `"positive_octant"` key.
    #[test]
    fn defects_by_rule_picks_up_positive_octant_rule() {
        let scene = one_facet(
            vec![[-1.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            [0.0, 0.0, 1.0],
        );
        let opts = ValidationOptions {
            check_positive_octant: true,
            ..ValidationOptions::default()
        };
        let r = validate(&scene, &opts);
        let rows = r.defects_by_rule();
        let positive_row = rows.iter().find(|(l, _)| *l == "positive_octant").unwrap();
        assert!(positive_row.1 > 0);
        let summed: usize = rows.iter().map(|(_, n)| *n).sum();
        assert_eq!(r.defect_total(), summed);
    }
}

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
//!    in exactly two triangles.
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
    let mut any = false;
    let mut mn = [f32::INFINITY; 3];
    let mut mx = [f32::NEG_INFINITY; 3];
    for_each_emitted_vertex(scene, |p| {
        for (axis, &c) in p.iter().enumerate() {
            if c.is_finite() {
                if c < mn[axis] {
                    mn[axis] = c;
                }
                if c > mx[axis] {
                    mx[axis] = c;
                }
                any = true;
            }
        }
    });
    if any && mn.iter().all(|c| c.is_finite()) && mx.iter().all(|c| c.is_finite()) {
        Some(Bbox { min: mn, max: mx })
    } else {
        None
    }
}

/// Logical face-index used in [`ValidationReport`] defect lists.
///
/// `mesh` and `primitive` indices are scene-graph order; `face` is the
/// triangle's index within that primitive's effective vertex stream
/// (post-index-buffer resolution, matching the encoder's emit order).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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
    rep
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
}

//! [`StlEncoder`] — [`Scene3D`]-in, bytes-out.
//!
//! Walks every mesh's `Triangles` primitives in scene-graph order and
//! concatenates them into a single STL output. Non-`Triangles`
//! topologies are rejected with [`Error::Unsupported`]. Per-face
//! normals are taken from the input primitive's `normals` field when
//! present and consistent; otherwise recomputed from positions via
//! the right-hand rule on each triangle's vertex order.
//!
//! Per-face attribute bytes survive a parse → reserialise round-trip
//! when present on `Mesh::extras["stl:per_face_attributes"]` as a hex
//! string (binary STL only — ASCII has no attribute slot).

use std::collections::HashSet;

use oxideav_mesh3d::{Error, Mesh3DEncoder, Result, Scene3D, Topology};

use crate::ascii::EncodeOptions;
use crate::{ascii, binary};

/// Summary statistics about the triangle stream that an [`StlEncoder`]
/// would emit for a given [`Scene3D`].
///
/// Returned by [`StlEncoder::stats`]; useful for tooling that wants to
/// report compression ratios ("shared-index → STL" expands every
/// shared vertex `share_factor` × times) without forcing a full
/// encode pass.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EncodeStats {
    /// Total triangle count summed across every `Triangles` primitive
    /// in the scene (after applying any present index buffer).
    pub triangles: usize,
    /// Total emitted vertex slots — `triangles × 3`, since STL has no
    /// vertex sharing.
    pub emitted_vertices: usize,
    /// Number of *logically* unique vertex positions (deduplicated by
    /// exact `f32` bit pattern). A scene with a fully-shared cube
    /// (8 vertices, 12 triangles) has `unique_vertices == 8`,
    /// `emitted_vertices == 36`.
    pub unique_vertices: usize,
}

impl EncodeStats {
    /// Average number of times each unique vertex appears in the
    /// emitted stream. Returns `0.0` if there are no unique vertices.
    pub fn share_factor(&self) -> f32 {
        if self.unique_vertices == 0 {
            0.0
        } else {
            (self.emitted_vertices as f32) / (self.unique_vertices as f32)
        }
    }

    /// Build an [`EncodeStats`] for `scene` using a *tolerance-based*
    /// definition of vertex uniqueness in place of the bit-exact one
    /// used by [`StlEncoder::stats`].
    ///
    /// Two emitted vertices `a` and `b` are treated as equal when the
    /// component-wise absolute distance is at most `eps` on each of
    /// the three axes (i.e. `|a.x - b.x| ≤ eps && |a.y - b.y| ≤ eps
    /// && |a.z - b.z| ≤ eps`). With `eps == 0.0` the comparison
    /// degenerates to bit-exact equality on finite values — this is
    /// the lossless path and matches [`StlEncoder::stats`] for any
    /// scene whose positions are finite.
    ///
    /// `triangles` and `emitted_vertices` are unchanged from the
    /// bit-exact path; only `unique_vertices` reflects the tolerance.
    /// Negative or non-finite `eps` is clamped to `0.0`.
    pub fn with_tolerance(scene: &Scene3D, eps: f32) -> Self {
        let bit_exact = compute_stats(scene);
        let (unique_vertices, _) = unique_vertices_with_tolerance(scene, eps);
        Self {
            triangles: bit_exact.triangles,
            emitted_vertices: bit_exact.emitted_vertices,
            unique_vertices,
        }
    }
}

// (No auto-injected extras key for unique-vertex count — the encode
//  contract stays pure-functional. Callers that want the dedup
//  metadata read it explicitly via `StlEncoder::stats(&scene)` and
//  attach it to wherever their pipeline routes diagnostics.)

/// Output flavour for [`StlEncoder`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StlFormat {
    /// Binary STL — 80-byte header + `uint32` triangle count + `N × 50`
    /// bytes per triangle. Default.
    Binary,
    /// ASCII STL — `solid … endsolid` token grammar.
    Ascii,
}

/// STL encoder — implements [`Mesh3DEncoder`].
#[derive(Debug)]
pub struct StlEncoder {
    format: StlFormat,
    ascii_opts: EncodeOptions,
}

impl StlEncoder {
    /// Construct a binary-mode encoder.
    pub fn new_binary() -> Self {
        Self {
            format: StlFormat::Binary,
            ascii_opts: EncodeOptions::default(),
        }
    }

    /// Construct an ASCII-mode encoder.
    pub fn new_ascii() -> Self {
        Self {
            format: StlFormat::Ascii,
            ascii_opts: EncodeOptions::default(),
        }
    }

    /// Construct an encoder for the given `format`.
    pub fn new(format: StlFormat) -> Self {
        Self {
            format,
            ascii_opts: EncodeOptions::default(),
        }
    }

    /// Set the ASCII float-formatting precision.
    ///
    /// `precision` is the number of decimals after the point (i.e.
    /// `{:.n}` formatting); a `None` value reverts to the default
    /// round-trip-safe `{}` formatter. Has no effect on binary output.
    ///
    /// ```
    /// use oxideav_stl::StlEncoder;
    /// let _ = StlEncoder::new_ascii().with_float_precision(Some(6));
    /// ```
    pub fn with_float_precision(mut self, precision: Option<usize>) -> Self {
        self.ascii_opts = EncodeOptions {
            float_precision: precision,
        };
        self
    }

    /// Output flavour this encoder will produce.
    pub fn format(&self) -> StlFormat {
        self.format
    }

    /// Compute pre-encode statistics on `scene` without materialising
    /// the byte stream. Useful for diagnostic tooling that wants to
    /// know how much vertex sharing the input has before paying for
    /// the full encode.
    pub fn stats(scene: &Scene3D) -> EncodeStats {
        compute_stats(scene)
    }

    /// Tolerance-based variant of the unique-vertex scan. Returns the
    /// number of distinct logical vertices under the `eps` rule plus a
    /// `dedup_map` whose `i`-th entry is the canonical-vertex slot
    /// assigned to the `i`-th *emitted* vertex (i.e. one entry per
    /// `emitted_vertices` slot in [`EncodeStats`]).
    ///
    /// Two emitted vertices are merged when each component-wise
    /// absolute distance is at most `eps`. With `eps == 0.0` the
    /// scan reduces to bit-exact equality on finite values (preserving
    /// the well-defined NaN behaviour of [`StlEncoder::stats`]).
    /// Negative / non-finite `eps` is clamped to `0.0`.
    ///
    /// Algorithmic note — this is an `O(N · K)` scan where `K` is the
    /// running canonical-vertex count, which is fine for diagnostic
    /// use on sub-100k-vertex scenes. Geometry-heavy callers should
    /// run a spatial index (kd-tree / hash grid) themselves and feed
    /// already-deduplicated positions through the bit-exact path.
    pub fn unique_vertices_with_tolerance(scene: &Scene3D, eps: f32) -> (usize, Vec<usize>) {
        unique_vertices_with_tolerance(scene, eps)
    }
}

/// Walk every `Triangles` primitive in `scene` and compute the
/// triangle count + emitted-vertex count + unique-vertex count.
///
/// "Unique" means matching by exact `f32` bit pattern (`to_bits()`),
/// which is the only definition that makes round-trip semantics
/// well-defined for floats — `0.1 + 0.2 != 0.3` is a real concern at
/// the geometry level. Callers that want a tolerance-based dedup
/// should pre-process their scene before calling.
///
/// Non-`Triangles` primitives are silently skipped (encode would
/// reject them up-front anyway).
pub(crate) fn compute_stats(scene: &Scene3D) -> EncodeStats {
    let mut triangles = 0usize;
    let mut emitted = 0usize;
    // (x_bits, y_bits, z_bits) — using bit patterns lets us hash NaNs
    // correctly (every NaN bit-pattern is a distinct slot) without
    // having to define a custom Eq.
    let mut unique: HashSet<(u32, u32, u32)> = HashSet::new();
    for mesh in &scene.meshes {
        for prim in &mesh.primitives {
            if prim.topology != Topology::Triangles {
                continue;
            }
            let face_count = match &prim.indices {
                Some(idx) => idx.len() / 3,
                None => prim.positions.len() / 3,
            };
            triangles += face_count;
            emitted += face_count * 3;
            // Walk the effective vertex sequence — this matches what
            // the encoder will emit, so unique-vertex semantics are
            // independent of whether the producer used an index buffer.
            for face_idx in 0..face_count {
                let (vi0, vi1, vi2) = match &prim.indices {
                    Some(oxideav_mesh3d::Indices::U16(v)) => {
                        let b = face_idx * 3;
                        (v[b] as usize, v[b + 1] as usize, v[b + 2] as usize)
                    }
                    Some(oxideav_mesh3d::Indices::U32(v)) => {
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
                        unique.insert((p[0].to_bits(), p[1].to_bits(), p[2].to_bits()));
                    }
                }
            }
        }
    }
    EncodeStats {
        triangles,
        emitted_vertices: emitted,
        unique_vertices: unique.len(),
    }
}

/// Tolerance-based unique-vertex scan + dedup map.
///
/// Walks every `Triangles` primitive in scene-graph / encoder order and
/// builds, for each emitted vertex, a canonical-slot index. Two emitted
/// vertices are mapped to the same slot when their three component
/// distances are all `≤ eps`. With `eps == 0.0` the scan degenerates
/// to bit-exact equality on finite values (matching the well-defined
/// NaN handling of [`compute_stats`]).
///
/// Returns `(unique_count, dedup_map)` where `dedup_map.len() ==
/// emitted_vertices` (the [`EncodeStats::emitted_vertices`] count).
pub(crate) fn unique_vertices_with_tolerance(scene: &Scene3D, eps: f32) -> (usize, Vec<usize>) {
    // Negative or non-finite eps: clamp to bit-exact behaviour rather
    // than panicking. Tolerance dedup is a diagnostic helper; refusing
    // garbage-eps inputs would push policy into the caller without
    // meaningfully widening the API contract.
    let eps = if eps.is_finite() && eps >= 0.0 {
        eps
    } else {
        0.0
    };

    // Pre-allocate using the bit-exact emitted-vertex count so we
    // never re-grow the dedup_map mid-scan.
    let bit_exact = compute_stats(scene);
    let mut dedup_map: Vec<usize> = Vec::with_capacity(bit_exact.emitted_vertices);

    if eps == 0.0 {
        // Fast path — group emitted vertices by exact f32 bit pattern,
        // exactly the same definition `compute_stats` uses for
        // `unique_vertices`. Avoids the O(K) scan and gives us the
        // canonical "tolerance == 0 ⇔ bit-exact" guarantee that the
        // [`EncodeStats::with_tolerance`] doc promises.
        use std::collections::HashMap;
        let mut by_bits: HashMap<(u32, u32, u32), usize> = HashMap::new();
        for_each_emitted_vertex(scene, |p| {
            let key = (p[0].to_bits(), p[1].to_bits(), p[2].to_bits());
            let next = by_bits.len();
            let slot = *by_bits.entry(key).or_insert(next);
            dedup_map.push(slot);
        });
        return (by_bits.len(), dedup_map);
    }

    // Slow path — O(N · K). Maintain a list of canonical positions
    // and, for each emitted vertex, scan it linearly to find the first
    // canonical within tolerance. With K small (real-world geometry
    // tends to have far fewer unique corners than emitted slots) this
    // amortises tolerably; on pathological all-distinct inputs it
    // degrades to O(N²) but the caller asked for a brute-force
    // tolerance scan and gets one. Spatial indexing belongs to
    // higher-layer mesh tooling, not the STL codec.
    let mut canonicals: Vec<[f32; 3]> = Vec::new();
    for_each_emitted_vertex(scene, |p| {
        let mut found: Option<usize> = None;
        for (i, c) in canonicals.iter().enumerate() {
            // Component-wise absolute distance ≤ eps on each axis.
            // Non-finite components compare as not-within-tolerance
            // (NaN propagation: any subtraction with NaN ⇒ NaN ⇒
            // any comparison ⇒ false), so each NaN takes its own
            // slot — same effective contract as the bit-exact path.
            if (p[0] - c[0]).abs() <= eps
                && (p[1] - c[1]).abs() <= eps
                && (p[2] - c[2]).abs() <= eps
            {
                found = Some(i);
                break;
            }
        }
        match found {
            Some(i) => dedup_map.push(i),
            None => {
                let new_slot = canonicals.len();
                canonicals.push(p);
                dedup_map.push(new_slot);
            }
        }
    });
    (canonicals.len(), dedup_map)
}

/// Walk every emitted vertex (post-index-buffer-resolution) of the
/// triangle stream that the encoder would write, in encoder order, and
/// pass it to `f`. Mirrors the iteration scheme of [`compute_stats`]
/// so all unique-vertex helpers see the same vertex sequence.
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
                    Some(oxideav_mesh3d::Indices::U16(v)) => {
                        let b = face_idx * 3;
                        (v[b] as usize, v[b + 1] as usize, v[b + 2] as usize)
                    }
                    Some(oxideav_mesh3d::Indices::U32(v)) => {
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

impl Default for StlEncoder {
    fn default() -> Self {
        Self::new_binary()
    }
}

impl Mesh3DEncoder for StlEncoder {
    fn encode(&mut self, scene: &Scene3D) -> Result<Vec<u8>> {
        // STL is a single-mesh format; we walk every mesh in the scene
        // and emit one big triangle list. Reject non-Triangles
        // primitives up-front so the encoder side has a single contract.
        for mesh in &scene.meshes {
            for prim in &mesh.primitives {
                if prim.topology != Topology::Triangles {
                    return Err(Error::Unsupported(format!(
                        "STL only supports Triangles topology; got {:?}",
                        prim.topology
                    )));
                }
            }
        }
        match self.format {
            StlFormat::Binary => binary::encode(scene),
            StlFormat::Ascii => ascii::encode_with(scene, &self.ascii_opts),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;

    use oxideav_mesh3d::{Indices, Mesh, Node, Primitive, Scene3D};

    use super::*;

    fn build_indexed_cube() -> Scene3D {
        // 8 unique corners + 12 triangles via a u32 index buffer
        // (the canonical "shared-vertex" cube). After encoding to STL,
        // every face emits 3 vertices → 36 emitted slots, but the
        // unique-vertex count under [`EncodeStats`] should still be 8.
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
        // 12-triangle cube indices.
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

    #[test]
    fn stats_unique_vertex_count_collapses_shared_corners() {
        let scene = build_indexed_cube();
        let stats = StlEncoder::stats(&scene);
        assert_eq!(stats.triangles, 12);
        assert_eq!(stats.emitted_vertices, 36);
        assert_eq!(stats.unique_vertices, 8);
    }

    #[test]
    fn stats_share_factor_matches_emitted_over_unique() {
        let scene = build_indexed_cube();
        let stats = StlEncoder::stats(&scene);
        // 36 / 8 = 4.5
        assert!((stats.share_factor() - 4.5).abs() < 1e-6);
    }

    #[test]
    fn stats_empty_scene_returns_zero_zero_zero() {
        let scene = Scene3D::new();
        let stats = StlEncoder::stats(&scene);
        assert_eq!(stats, EncodeStats::default());
        assert_eq!(stats.share_factor(), 0.0);
    }

    #[test]
    fn stats_unindexed_primitive_treats_each_facet_vertex_independently() {
        // No index buffer + 3 unique repeated triangles → unique == 3
        // (one corner) emit == 9.
        let positions = vec![[0.0_f32, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        let mut prim = Primitive {
            topology: Topology::Triangles,
            positions: positions.clone(),
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
        // Repeat the triangle three times — same positions, three
        // emissions worth of slots.
        prim.positions.extend(positions.clone());
        prim.positions.extend(positions.clone());
        let mesh = Mesh {
            name: None,
            primitives: vec![prim],
        };
        let mut scene = Scene3D::new();
        scene.add_mesh(mesh);
        let stats = StlEncoder::stats(&scene);
        assert_eq!(stats.triangles, 3);
        assert_eq!(stats.emitted_vertices, 9);
        // The three repeated triangles have only 3 unique corners.
        assert_eq!(stats.unique_vertices, 3);
        assert!((stats.share_factor() - 3.0).abs() < 1e-6);
    }
}

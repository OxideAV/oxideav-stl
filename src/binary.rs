//! Binary STL parser + serializer.
//!
//! Layout (per Marshall Burns' transcription, §6.5.3):
//!
//! ```text
//! [0..80]    : 80-byte header — vendor-defined, typically padded with NUL.
//! [80..84]   : uint32 LE  triangle_count
//! repeat triangle_count {
//!     [0..12]  : float32[3] LE   normal
//!     [12..24] : float32[3] LE   vertex 0
//!     [24..36] : float32[3] LE   vertex 1
//!     [36..48] : float32[3] LE   vertex 2
//!     [48..50] : uint16 LE       attribute_byte_count
//! }
//! ```
//!
//! `attribute_byte_count` is "specified to be set to zero" by the
//! original 3D Systems spec, but vendor extensions (Materialise /
//! VisCAM / SolidView) repurpose it for per-face colour. We surface
//! the raw two-byte payloads on the parsed [`Mesh::extras`] field as
//! a single hex string so they survive a parse → reserialise round
//! trip without us having to commit to any one vendor's interpretation.

use std::collections::HashMap;

use oxideav_mesh3d::{Axis, Error, Mesh, Node, Primitive, Result, Scene3D, Topology, Unit};

/// Bytes per triangle record in binary STL (12 + 36 + 2).
pub(crate) const TRIANGLE_BYTES: usize = 50;

/// Header size in bytes (vendor string + triangle count u32).
pub(crate) const HEADER_BYTES: usize = 80 + 4;

/// Default 80-byte header content emitted by [`encode`]. The first
/// six bytes intentionally do NOT begin with `b"solid "`, so a binary
/// file we wrote ourselves will never tickle the ASCII sniffer.
pub(crate) const DEFAULT_HEADER: &[u8; 80] =
    b"oxideav-stl binary STL writer                                                   ";

/// Extras key under which per-face attribute bytes round-trip.
pub(crate) const PER_FACE_ATTRS_KEY: &str = "stl:per_face_attributes";

/// Extras key under which the auto-detected (or caller-supplied)
/// 16-bit per-face colour convention is recorded — `"viscam"` or
/// `"materialise"`. Set on decode whenever
/// [`crate::color::detect`] picks a confident value; the encoder
/// round-trips the raw byte payload through `stl:per_face_attributes`
/// regardless, so this key is metadata for downstream renderers that
/// want to interpret the bytes.
pub(crate) const COLOR_CONVENTION_KEY: &str = "stl:color_convention";

/// Parse a binary STL byte slice into a [`Scene3D`].
pub fn decode(bytes: &[u8]) -> Result<Scene3D> {
    if bytes.len() < HEADER_BYTES {
        return Err(Error::InvalidData(format!(
            "binary STL truncated: need at least {HEADER_BYTES} bytes for header + count, got {}",
            bytes.len()
        )));
    }
    let header = &bytes[..80];
    let triangle_count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;

    #[cfg(feature = "trace")]
    let tracer = crate::trace::Tracer::from_env();
    #[cfg(feature = "trace")]
    if let Some(t) = tracer.as_ref() {
        t.emit(crate::trace::Event::Header {
            format: crate::trace::Format::Binary,
            byte_len: bytes.len(),
            header_hex: Some(header),
            name: None,
        });
        t.emit(crate::trace::Event::TriangleCount {
            count: triangle_count,
        });
    }

    let body_len = triangle_count
        .checked_mul(TRIANGLE_BYTES)
        .ok_or_else(|| Error::InvalidData("binary STL triangle_count overflow".into()))?;
    let total = HEADER_BYTES
        .checked_add(body_len)
        .ok_or_else(|| Error::InvalidData("binary STL total size overflow".into()))?;
    if bytes.len() < total {
        return Err(Error::InvalidData(format!(
            "binary STL truncated: triangle_count={triangle_count} requires {total} bytes, got {}",
            bytes.len()
        )));
    }

    // Per-face normals expand to per-vertex (3 copies per face) so
    // downstream renderers don't have to reconstruct from winding.
    let mut positions = Vec::with_capacity(triangle_count * 3);
    let mut normals = Vec::with_capacity(triangle_count * 3);
    let mut attr_bytes = Vec::with_capacity(triangle_count * 2);
    let mut any_nonzero_attr = false;

    // Hot loop — iterate triangle records as fixed-size 50-byte
    // chunks so the compiler can fold every per-field bounds check
    // into the single chunk-length proof. `chunks_exact` lets us
    // skip its `remainder()` tail since the up-front total-length
    // check above guarantees an exact multiple. Symmetric to the
    // round-175 `pack_triangle_record` optimisation on the encoder
    // side, and the `binary_cube_triangle_records_roundtrip_byte_identical`
    // integration test continues to pin the wire bytes.
    let body = &bytes[HEADER_BYTES..HEADER_BYTES + body_len];
    #[cfg(feature = "trace")]
    let mut tri_index: usize = 0;
    for chunk in body.chunks_exact(TRIANGLE_BYTES) {
        // `chunks_exact` yields slices known to be exactly
        // `TRIANGLE_BYTES` long; the `try_into` turns that into an
        // `&[u8; 50]` so `unpack_triangle_record` can index without
        // bounds checks.
        let record: &[u8; TRIANGLE_BYTES] = chunk
            .try_into()
            .expect("chunks_exact yields TRIANGLE_BYTES");
        let (n, v0, v1, v2, attr_lo, attr_hi) = unpack_triangle_record(record);
        if attr_lo != 0 || attr_hi != 0 {
            any_nonzero_attr = true;
        }
        #[cfg(feature = "trace")]
        if let Some(t) = tracer.as_ref() {
            t.emit(crate::trace::Event::Triangle {
                index: tri_index,
                normal: n,
                v0,
                v1,
                v2,
                attribute_bytes: Some([attr_lo, attr_hi]),
            });
            tri_index += 1;
        }
        positions.push(v0);
        positions.push(v1);
        positions.push(v2);
        normals.push(n);
        normals.push(n);
        normals.push(n);
        attr_bytes.push(attr_lo);
        attr_bytes.push(attr_hi);
    }

    #[cfg(feature = "trace")]
    if let Some(t) = tracer.as_ref() {
        t.emit(crate::trace::Event::Done {
            source: crate::trace::Format::Binary,
            triangles_emitted: triangle_count,
        });
    }

    // The `<name>` after `solid` lives in the binary header — vendors
    // mostly fill it with their writer's signature, so we don't try
    // to extract a logical mesh name. We DO scan the 80-byte header
    // for Materialise's `COLOR=R G B A` and `MATERIAL=…` per-object
    // default lines and surface them on `Primitive::extras` —
    // see `crate::materialise_header`.
    let mut prim_extras: HashMap<String, serde_json::Value> = HashMap::new();
    let (header_color, header_material) = crate::materialise_header::parse_header(header);
    let mut mesh_extras: HashMap<String, serde_json::Value> = HashMap::new();
    crate::materialise_header::insert_extras(&mut mesh_extras, header_color, header_material);
    if any_nonzero_attr {
        mesh_extras.insert(
            PER_FACE_ATTRS_KEY.to_string(),
            serde_json::Value::String(hex_encode(&attr_bytes)),
        );
        // 16-bit per-face colour heuristic (Materialise vs VisCAM).
        // Only record when the detector is confident; if it returns
        // `None` the caller still has the raw bytes via
        // `stl:per_face_attributes` and can pick a convention
        // explicitly.
        if let Some(convention) = crate::color::detect(&attr_bytes) {
            mesh_extras.insert(
                COLOR_CONVENTION_KEY.to_string(),
                serde_json::Value::String(convention.as_str().to_string()),
            );
        }
    }
    // Parser symmetry — primitives don't carry the per-face attrs;
    // the mesh-level extras is the round-trip channel.
    prim_extras.insert(
        "stl:source".to_string(),
        serde_json::Value::String("binary".to_string()),
    );

    // Forward-compatible construction: oxideav-mesh3d's `Primitive`
    // is `#[non_exhaustive]`, so external crates must build via
    // `Primitive::new(Topology::*)` + per-field assignment.
    let mut primitive = Primitive::new(Topology::Triangles);
    primitive.positions = positions;
    primitive.normals = Some(normals);
    primitive.extras = prim_extras;

    // `Mesh` is `#[non_exhaustive]`; build via `Mesh::new` + the
    // `with_primitive` builder so we don't break when mesh3d adds
    // further fields in future minor releases.
    let mesh = Mesh::new(None).with_primitive(primitive);

    let mut scene = Scene3D::new();
    // STL files are universally treated as Z-up + millimetres by the
    // additive-manufacturing toolchain. Surface the metadata; do NOT
    // mutate geometry — downstream consumers re-orient if needed.
    scene.up_axis = Axis::PosZ;
    scene.unit = Unit::Millimetres;
    let mesh_id = scene.add_mesh(mesh);
    if !mesh_extras.is_empty() {
        // Mesh::extras isn't present on the type — we lift onto the
        // primitive's extras instead so the round-trip key has a home.
        if let Some(m) = scene.meshes.get_mut(mesh_id.0 as usize) {
            if let Some(p) = m.primitives.first_mut() {
                for (k, v) in mesh_extras {
                    p.extras.insert(k, v);
                }
            }
        }
    }
    let mut node = Node::new();
    node.mesh = Some(mesh_id);
    let node_id = scene.add_node(node);
    scene.add_root(node_id);

    Ok(scene)
}

/// Serialise a [`Scene3D`] as binary STL.
pub fn encode(scene: &Scene3D) -> Result<Vec<u8>> {
    // Collect every Triangles primitive into a flat triangle list.
    let mut triangles: Vec<(Vec3, Vec3, Vec3, Vec3)> = Vec::new();
    let mut attr_pairs: Vec<(u8, u8)> = Vec::new();

    for mesh in &scene.meshes {
        for prim in &mesh.primitives {
            // Caller already guaranteed Triangles via StlEncoder, but
            // defend in depth if encode() is invoked directly.
            if prim.topology != Topology::Triangles {
                return Err(Error::Unsupported(format!(
                    "STL only supports Triangles topology; got {:?}",
                    prim.topology
                )));
            }
            // Resolve effective vertex order — index buffer if present,
            // otherwise the position vector in order.
            let face_count = match &prim.indices {
                Some(idx) => idx.len() / 3,
                None => prim.positions.len() / 3,
            };
            // Recover per-face attributes from extras when present and
            // long enough to cover this primitive's faces.
            let extra_attrs = prim
                .extras
                .get(PER_FACE_ATTRS_KEY)
                .and_then(|v| v.as_str())
                .and_then(|s| hex_decode(s).ok());

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
                let v0 = prim.positions[vi0];
                let v1 = prim.positions[vi1];
                let v2 = prim.positions[vi2];

                // Normal: prefer first vertex's incoming normal if it
                // exists and the primitive's normal length matches
                // positions; otherwise compute from vertex order.
                let n = match prim.normals.as_ref() {
                    Some(ns) if ns.len() == prim.positions.len() => ns[vi0],
                    _ => face_normal(v0, v1, v2),
                };
                triangles.push((n, v0, v1, v2));

                // Per-face attribute bytes — pulled from extras when
                // available, defaulting to (0, 0) per spec.
                let (lo, hi) = match extra_attrs.as_ref() {
                    Some(bytes) if bytes.len() >= (face_idx + 1) * 2 => {
                        (bytes[face_idx * 2], bytes[face_idx * 2 + 1])
                    }
                    _ => (0u8, 0u8),
                };
                attr_pairs.push((lo, hi));
            }
        }
    }

    let mut out = Vec::with_capacity(HEADER_BYTES + triangles.len() * TRIANGLE_BYTES);
    // If the first primitive carries Materialise default-color or
    // default-material extras, build a compatible header so they
    // round-trip; otherwise fall back to the writer-signature default
    // header. The Materialise header is always NUL-padded to exactly
    // 80 bytes by `build_header`, so the rest of the file layout is
    // unchanged.
    let materialise_header = scene
        .meshes
        .iter()
        .flat_map(|m| m.primitives.iter())
        .next()
        .and_then(|p| {
            let (c, m) = crate::materialise_header::extras_to_payload(&p.extras);
            crate::materialise_header::build_header(c.as_ref(), m.as_ref())
        });
    if let Some(h) = materialise_header.as_ref() {
        out.extend_from_slice(h);
    } else {
        out.extend_from_slice(DEFAULT_HEADER);
    }
    let count: u32 = triangles
        .len()
        .try_into()
        .map_err(|_| Error::InvalidData("STL triangle count exceeds u32::MAX".into()))?;
    out.extend_from_slice(&count.to_le_bytes());

    #[cfg(feature = "trace")]
    let tracer = crate::trace::Tracer::from_env();
    #[cfg(feature = "trace")]
    if let Some(t) = tracer.as_ref() {
        let header_for_trace: &[u8] = match materialise_header.as_ref() {
            Some(h) => h.as_slice(),
            None => DEFAULT_HEADER.as_slice(),
        };
        t.emit(crate::trace::Event::Header {
            format: crate::trace::Format::Binary,
            byte_len: HEADER_BYTES + triangles.len() * TRIANGLE_BYTES,
            header_hex: Some(header_for_trace),
            name: None,
        });
        t.emit(crate::trace::Event::TriangleCount {
            count: triangles.len(),
        });
    }

    #[cfg(feature = "trace")]
    let mut tri_index: usize = 0;
    // Pack each triangle into a stack-resident 50-byte record then
    // copy it out in a single `extend_from_slice` call. Profiling
    // (`examples/profile_encode_binary.rs`, round 175) showed the
    // previous 14-call-per-triangle pattern (12 × `write_vec3` +
    // 2 × `push`) bottlenecked on bounds-check-and-grow overhead on
    // the destination `Vec`; one bulk copy per triangle is materially
    // faster while staying byte-identical.
    for ((n, v0, v1, v2), (lo, hi)) in triangles.iter().zip(attr_pairs.iter()) {
        #[cfg(feature = "trace")]
        if let Some(t) = tracer.as_ref() {
            t.emit(crate::trace::Event::Triangle {
                index: tri_index,
                normal: *n,
                v0: *v0,
                v1: *v1,
                v2: *v2,
                attribute_bytes: Some([*lo, *hi]),
            });
            tri_index += 1;
        }
        let record = pack_triangle_record(*n, *v0, *v1, *v2, *lo, *hi);
        out.extend_from_slice(&record);
    }

    #[cfg(feature = "trace")]
    if let Some(t) = tracer.as_ref() {
        // Bit-exact share-stats summary — emitted between the final
        // `triangle` event and `done` so a JSONL auditor sees the
        // summary in stream order. The encoder only reports the
        // bit-exact path here (`tolerance_eps == None`); callers
        // reaching for the tolerance variant compute it via
        // `EncodeStats::with_tolerance` themselves.
        let stats = crate::encoder::compute_stats(scene);
        t.emit(crate::trace::Event::ShareStats {
            triangles: stats.triangles,
            emitted_vertices: stats.emitted_vertices,
            unique_vertices: stats.unique_vertices,
            share_factor: stats.share_factor(),
            tolerance_eps: None,
        });
        t.emit(crate::trace::Event::Done {
            source: crate::trace::Format::Binary,
            triangles_emitted: triangles.len(),
        });
    }

    Ok(out)
}

type Vec3 = [f32; 3];

/// Unpack one full STL binary triangle record (normal + 3 vertices +
/// 2-byte attribute slot) from a fixed-size 50-byte reference.
///
/// The decode hot loop walks the body with
/// `chunks_exact(TRIANGLE_BYTES)` and converts each chunk to a
/// `&[u8; 50]` reference so the compiler can prove every nested
/// `f32::from_le_bytes` slice is in-bounds at compile time — the
/// per-field bounds checks the previous `read_vec3(&bytes[c..c+12])`
/// pattern carried collapse to a single chunk-length proof.
/// Symmetric counterpart of [`pack_triangle_record`]; the bytewise
/// invariant is pinned by
/// `binary_cube_triangle_records_roundtrip_byte_identical`.
#[inline]
fn unpack_triangle_record(rec: &[u8; TRIANGLE_BYTES]) -> (Vec3, Vec3, Vec3, Vec3, u8, u8) {
    let n: Vec3 = [
        f32::from_le_bytes([rec[0], rec[1], rec[2], rec[3]]),
        f32::from_le_bytes([rec[4], rec[5], rec[6], rec[7]]),
        f32::from_le_bytes([rec[8], rec[9], rec[10], rec[11]]),
    ];
    let v0: Vec3 = [
        f32::from_le_bytes([rec[12], rec[13], rec[14], rec[15]]),
        f32::from_le_bytes([rec[16], rec[17], rec[18], rec[19]]),
        f32::from_le_bytes([rec[20], rec[21], rec[22], rec[23]]),
    ];
    let v1: Vec3 = [
        f32::from_le_bytes([rec[24], rec[25], rec[26], rec[27]]),
        f32::from_le_bytes([rec[28], rec[29], rec[30], rec[31]]),
        f32::from_le_bytes([rec[32], rec[33], rec[34], rec[35]]),
    ];
    let v2: Vec3 = [
        f32::from_le_bytes([rec[36], rec[37], rec[38], rec[39]]),
        f32::from_le_bytes([rec[40], rec[41], rec[42], rec[43]]),
        f32::from_le_bytes([rec[44], rec[45], rec[46], rec[47]]),
    ];
    (n, v0, v1, v2, rec[48], rec[49])
}

/// Pack one full STL binary triangle record (normal + 3 vertices +
/// 2-byte attribute slot) into a stack-resident 50-byte array.
///
/// The encoder hot loop calls this once per triangle then performs a
/// single `Vec::extend_from_slice` of the result — materially faster
/// than the previous 14-call-per-triangle pattern (12 four-byte
/// writes via `write_vec3` plus two single-byte `push`es) since the
/// destination `Vec` is grown once per record rather than per field.
/// Layout matches §6.5.3 of the format spec byte-for-byte; the
/// `binary_cube_triangle_records_roundtrip_byte_identical`
/// integration test pins this invariant.
#[inline]
fn pack_triangle_record(n: Vec3, v0: Vec3, v1: Vec3, v2: Vec3, lo: u8, hi: u8) -> [u8; 50] {
    let mut rec = [0u8; 50];
    rec[0..4].copy_from_slice(&n[0].to_le_bytes());
    rec[4..8].copy_from_slice(&n[1].to_le_bytes());
    rec[8..12].copy_from_slice(&n[2].to_le_bytes());
    rec[12..16].copy_from_slice(&v0[0].to_le_bytes());
    rec[16..20].copy_from_slice(&v0[1].to_le_bytes());
    rec[20..24].copy_from_slice(&v0[2].to_le_bytes());
    rec[24..28].copy_from_slice(&v1[0].to_le_bytes());
    rec[28..32].copy_from_slice(&v1[1].to_le_bytes());
    rec[32..36].copy_from_slice(&v1[2].to_le_bytes());
    rec[36..40].copy_from_slice(&v2[0].to_le_bytes());
    rec[40..44].copy_from_slice(&v2[1].to_le_bytes());
    rec[44..48].copy_from_slice(&v2[2].to_le_bytes());
    rec[48] = lo;
    rec[49] = hi;
    rec
}

/// Right-hand-rule face normal from three vertices.
fn face_normal(a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    let u = sub(b, a);
    let v = sub(c, a);
    normalise(cross(u, v))
}

fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn normalise(v: Vec3) -> Vec3 {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len > f32::EPSILON {
        [v[0] / len, v[1] / len, v[2] / len]
    } else {
        // Degenerate triangle — leave a zero normal, which is the
        // documented STL convention for "the consumer should
        // recompute from winding".
        [0.0, 0.0, 0.0]
    }
}

fn hex_encode(bytes: &[u8]) -> String {
    let mut s = String::with_capacity(bytes.len() * 2);
    const HEX: &[u8; 16] = b"0123456789abcdef";
    for b in bytes {
        s.push(HEX[(b >> 4) as usize] as char);
        s.push(HEX[(b & 0x0f) as usize] as char);
    }
    s
}

fn hex_decode(s: &str) -> std::result::Result<Vec<u8>, &'static str> {
    if s.len() % 2 != 0 {
        return Err("odd-length hex string");
    }
    let mut out = Vec::with_capacity(s.len() / 2);
    let bytes = s.as_bytes();
    for chunk in bytes.chunks_exact(2) {
        let hi = nib(chunk[0])?;
        let lo = nib(chunk[1])?;
        out.push((hi << 4) | lo);
    }
    Ok(out)
}

fn nib(b: u8) -> std::result::Result<u8, &'static str> {
    match b {
        b'0'..=b'9' => Ok(b - b'0'),
        b'a'..=b'f' => Ok(b - b'a' + 10),
        b'A'..=b'F' => Ok(b - b'A' + 10),
        _ => Err("non-hex character in attribute string"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn header_does_not_start_with_solid_space() {
        // Critical for the round-trip — our binary writer must NOT
        // produce a header that the ASCII sniffer would mistake.
        assert!(!DEFAULT_HEADER.starts_with(b"solid "));
    }

    #[test]
    fn empty_scene_yields_zero_triangle_binary() {
        let s = Scene3D::new();
        let out = encode(&s).unwrap();
        assert_eq!(out.len(), HEADER_BYTES);
        let count = u32::from_le_bytes([out[80], out[81], out[82], out[83]]);
        assert_eq!(count, 0);
    }

    #[test]
    fn hex_roundtrip() {
        let raw = vec![0x00, 0xff, 0x12, 0xab, 0xcd];
        let s = hex_encode(&raw);
        assert_eq!(s, "00ff12abcd");
        assert_eq!(hex_decode(&s).unwrap(), raw);
    }

    #[test]
    fn pack_unpack_record_symmetry() {
        // Round 189: `unpack_triangle_record` is the symmetric
        // inverse of `pack_triangle_record`. Bit-pattern-preserving
        // for every f32 (including the spec-sentinel zeros, the
        // 1989 worked example's `±1.234e±05` style values, and
        // negative zero) so a triangle-record byte stream round-trips
        // through pack-then-unpack with f32 bit-pattern equality.
        let cases: &[(Vec3, Vec3, Vec3, Vec3, u8, u8)] = &[
            (
                [0.0, 0.0, -1.0],
                [0.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [1.0, 0.0, 0.0],
                0,
                0,
            ),
            (
                [1.234e5, -1.234e-5, 0.0],
                [-0.0, f32::INFINITY, f32::NEG_INFINITY],
                [f32::MIN, f32::MAX, f32::EPSILON],
                [42.0, 0.5, -0.25],
                0xab,
                0xcd,
            ),
        ];
        for (n, v0, v1, v2, lo, hi) in cases {
            let packed = pack_triangle_record(*n, *v0, *v1, *v2, *lo, *hi);
            let (n2, v0_2, v1_2, v2_2, lo2, hi2) = unpack_triangle_record(&packed);
            for (a, b) in n.iter().zip(n2.iter()) {
                assert_eq!(a.to_bits(), b.to_bits(), "normal bit mismatch");
            }
            for (a, b) in v0.iter().zip(v0_2.iter()) {
                assert_eq!(a.to_bits(), b.to_bits(), "v0 bit mismatch");
            }
            for (a, b) in v1.iter().zip(v1_2.iter()) {
                assert_eq!(a.to_bits(), b.to_bits(), "v1 bit mismatch");
            }
            for (a, b) in v2.iter().zip(v2_2.iter()) {
                assert_eq!(a.to_bits(), b.to_bits(), "v2 bit mismatch");
            }
            assert_eq!(*lo, lo2);
            assert_eq!(*hi, hi2);
        }
    }
}

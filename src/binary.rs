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

    let mut cursor = HEADER_BYTES;
    #[cfg(feature = "trace")]
    let mut tri_index: usize = 0;
    for _ in 0..triangle_count {
        let n = read_vec3(&bytes[cursor..cursor + 12]);
        let v0 = read_vec3(&bytes[cursor + 12..cursor + 24]);
        let v1 = read_vec3(&bytes[cursor + 24..cursor + 36]);
        let v2 = read_vec3(&bytes[cursor + 36..cursor + 48]);
        let attr_lo = bytes[cursor + 48];
        let attr_hi = bytes[cursor + 49];
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
        cursor += TRIANGLE_BYTES;
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

    let primitive = Primitive {
        topology: Topology::Triangles,
        positions,
        normals: Some(normals),
        tangents: None,
        uvs: Vec::new(),
        colors: Vec::new(),
        joints: None,
        weights: None,
        indices: None,
        material: None,
        extras: prim_extras,
    };

    let mesh = Mesh {
        name: None,
        primitives: vec![primitive],
    };

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
        write_vec3(&mut out, *n);
        write_vec3(&mut out, *v0);
        write_vec3(&mut out, *v1);
        write_vec3(&mut out, *v2);
        out.push(*lo);
        out.push(*hi);
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

fn read_vec3(bytes: &[u8]) -> Vec3 {
    [
        f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
        f32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
        f32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]),
    ]
}

fn write_vec3(out: &mut Vec<u8>, v: Vec3) {
    for c in v {
        out.extend_from_slice(&c.to_le_bytes());
    }
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
}

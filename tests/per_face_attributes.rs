//! Per-face attribute byte preservation through parse → reserialise.
//!
//! When a binary STL's per-face `uint16` attribute byte count is
//! non-zero (Materialise / VisCAM colour conventions stash data
//! there), our decoder surfaces the raw bytes on
//! `Primitive::extras["stl:per_face_attributes"]` as a hex string,
//! and our binary encoder reads that key back to round-trip the
//! payload identically.

use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};
use oxideav_stl::{StlDecoder, StlEncoder};

fn synth_with_attrs() -> Vec<u8> {
    let mut buf = Vec::new();
    // 80-byte header (no `solid ` prefix so we route to binary).
    let mut header = [b' '; 80];
    header[..18].copy_from_slice(b"oxideav-stl-attrs ");
    buf.extend_from_slice(&header);
    // Two triangles.
    buf.extend_from_slice(&2u32.to_le_bytes());
    let push_tri =
        |buf: &mut Vec<u8>, n: [f32; 3], v0: [f32; 3], v1: [f32; 3], v2: [f32; 3], attr: u16| {
            for c in n {
                buf.extend_from_slice(&c.to_le_bytes());
            }
            for c in v0 {
                buf.extend_from_slice(&c.to_le_bytes());
            }
            for c in v1 {
                buf.extend_from_slice(&c.to_le_bytes());
            }
            for c in v2 {
                buf.extend_from_slice(&c.to_le_bytes());
            }
            buf.extend_from_slice(&attr.to_le_bytes());
        };
    push_tri(
        &mut buf,
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        0x1234, // payload — VisCAM-style packed-RGB, doesn't matter to us
    );
    push_tri(
        &mut buf,
        [0.0, 0.0, -1.0],
        [0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        0xabcd,
    );
    buf
}

#[test]
fn per_face_attribute_bytes_round_trip_via_extras() {
    let bytes = synth_with_attrs();
    let scene = StlDecoder::new().decode(&bytes).unwrap();

    let prim = &scene.meshes[0].primitives[0];
    let extras_hex = prim
        .extras
        .get("stl:per_face_attributes")
        .and_then(|v| v.as_str())
        .expect("decoder should surface non-zero attribute bytes");
    // Two triangles × 2 bytes = 4 bytes = 8 hex chars. Bytes are
    // emitted in file order (LE-encoded u16, lo first then hi).
    // 0x1234 LE → 0x34, 0x12 → "3412"; 0xabcd LE → 0xcd, 0xab → "cdab".
    assert_eq!(extras_hex, "3412cdab");

    // Re-encode and confirm the per-face attribute slot survives.
    let reserialized = StlEncoder::new_binary().encode(&scene).unwrap();

    // Triangle 0 attribute slot lives at bytes 84+48..84+50.
    let a0 = u16::from_le_bytes([reserialized[84 + 48], reserialized[84 + 49]]);
    assert_eq!(a0, 0x1234);
    // Triangle 1 attribute slot lives at bytes 84+50+48..84+50+50.
    let a1 = u16::from_le_bytes([reserialized[84 + 50 + 48], reserialized[84 + 50 + 49]]);
    assert_eq!(a1, 0xabcd);
}

#[test]
fn zero_attribute_bytes_do_not_populate_extras() {
    // Ordinary (no-attrs) binary STL → no extras key.
    let mut buf = Vec::new();
    buf.extend_from_slice(&[b' '; 80]);
    buf.extend_from_slice(&1u32.to_le_bytes());
    for f in [0.0_f32, 0.0, 1.0] {
        buf.extend_from_slice(&f.to_le_bytes());
    }
    for v in [[0.0_f32, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]] {
        for c in v {
            buf.extend_from_slice(&c.to_le_bytes());
        }
    }
    buf.extend_from_slice(&0u16.to_le_bytes());

    let scene = StlDecoder::new().decode(&buf).unwrap();
    let prim = &scene.meshes[0].primitives[0];
    assert!(!prim.extras.contains_key("stl:per_face_attributes"));
}

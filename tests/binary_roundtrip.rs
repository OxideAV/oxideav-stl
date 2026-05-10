//! Binary STL → Scene3D → binary STL round-trip.
//!
//! Builds a 12-triangle cube as binary STL (synthesised from the
//! same vertex set as the ASCII test), parses it, reserialises, and
//! verifies that the triangle-record bytes round-trip identically.
//! The 80-byte header and `attribute_byte_count` are NOT required to
//! match the original — the spec lets the writer pick any header
//! string and the attribute slot is "specified to be set to zero".

use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};
use oxideav_stl::{StlDecoder, StlEncoder};

/// One STL triangle record's worth of vertex data: (normal, v0, v1, v2).
type Tri = ([f32; 3], [f32; 3], [f32; 3], [f32; 3]);

fn synth_binary_cube() -> Vec<u8> {
    // Per-face data: (normal, v0, v1, v2)
    let faces: &[Tri] = &[
        (
            [0.0, 0.0, -1.0],
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
        ),
        (
            [0.0, 0.0, -1.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
        ),
        (
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
        ),
        (
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
            [0.0, 1.0, 1.0],
        ),
        (
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 1.0],
        ),
        (
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 1.0],
            [0.0, 0.0, 1.0],
        ),
        (
            [0.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 1.0, 1.0],
            [1.0, 1.0, 0.0],
        ),
        (
            [0.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 1.0, 1.0],
            [1.0, 1.0, 1.0],
        ),
        (
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, 1.0, 1.0],
        ),
        (
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 1.0, 1.0],
            [0.0, 1.0, 0.0],
        ),
        (
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [1.0, 1.0, 1.0],
        ),
        (
            [1.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
            [1.0, 0.0, 1.0],
        ),
    ];
    let mut buf = Vec::with_capacity(84 + faces.len() * 50);
    // Header — our cube uses an 80-byte header that does NOT begin
    // with `b"solid "` so the ASCII sniffer doesn't false-positive.
    let mut header = [b' '; 80];
    let sig = b"oxideav-stl-test cube";
    header[..sig.len()].copy_from_slice(sig);
    buf.extend_from_slice(&header);
    buf.extend_from_slice(&(faces.len() as u32).to_le_bytes());
    for (n, v0, v1, v2) in faces {
        for &c in n {
            buf.extend_from_slice(&c.to_le_bytes());
        }
        for &c in v0 {
            buf.extend_from_slice(&c.to_le_bytes());
        }
        for &c in v1 {
            buf.extend_from_slice(&c.to_le_bytes());
        }
        for &c in v2 {
            buf.extend_from_slice(&c.to_le_bytes());
        }
        buf.extend_from_slice(&0u16.to_le_bytes()); // attribute byte count
    }
    buf
}

#[test]
fn binary_cube_triangle_records_roundtrip_byte_identical() {
    let original = synth_binary_cube();
    let scene = StlDecoder::new().decode(&original).unwrap();
    assert_eq!(scene.triangle_count(), 12);

    let reserialized = StlEncoder::new_binary().encode(&scene).unwrap();
    assert_eq!(reserialized.len(), original.len());

    // Triangle count slot must match.
    let orig_count = u32::from_le_bytes([original[80], original[81], original[82], original[83]]);
    let new_count = u32::from_le_bytes([
        reserialized[80],
        reserialized[81],
        reserialized[82],
        reserialized[83],
    ]);
    assert_eq!(orig_count, new_count);

    // Compare triangle records byte-for-byte. The 80-byte header is
    // expected to differ (we re-emit our own writer signature), but
    // every triangle record from byte 84 onward should be identical
    // because we round-trip f32 values and zero attribute bytes
    // verbatim.
    assert_eq!(&reserialized[84..], &original[84..]);
}

#[test]
fn binary_cube_yields_z_up_and_millimetres() {
    let bytes = synth_binary_cube();
    let scene = StlDecoder::new().decode(&bytes).unwrap();
    assert_eq!(scene.up_axis, oxideav_mesh3d::Axis::PosZ);
    assert_eq!(scene.unit, oxideav_mesh3d::Unit::Millimetres);
}

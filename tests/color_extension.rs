//! 16-bit per-face colour extension — VisCAM/SolidView and Materialise
//! conventions.
//!
//! Two complementary tests:
//!
//! 1. Synthesise a binary STL whose attribute slots are bit15-set
//!    (VisCAM-style); decode, verify
//!    `stl:color_convention == "viscam"`, and confirm the raw bytes
//!    survive a re-encode.
//! 2. Same with bit15-clear (Materialise-style) and
//!    `"materialise"`.
//! 3. Synthesise a checkered-cube STL with explicit
//!    [`Stl16BitColor`] values, encode, re-decode, and verify the
//!    colour values survive a full round trip.

use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};
use oxideav_stl::{ColorConvention, Stl16BitColor, StlDecoder, StlEncoder};

/// Build a 2-triangle binary STL with the supplied attribute slots.
fn synth_with_slots(slot0: u16, slot1: u16) -> Vec<u8> {
    let mut buf = Vec::new();
    let mut header = [b' '; 80];
    header[..14].copy_from_slice(b"color-fixture ");
    buf.extend_from_slice(&header);
    buf.extend_from_slice(&2u32.to_le_bytes());
    let push_tri = |buf: &mut Vec<u8>, slot: u16| {
        // Normal + 3 vertices, all zero-padded.
        for _ in 0..12 {
            buf.extend_from_slice(&0.0_f32.to_le_bytes());
        }
        buf.extend_from_slice(&slot.to_le_bytes());
    };
    push_tri(&mut buf, slot0);
    push_tri(&mut buf, slot1);
    buf
}

#[test]
fn viscam_style_attributes_are_detected_and_round_trip() {
    // Two slots with bit 15 set → the detector should pick `viscam`.
    // Slot 0: pure red (R=31, G=0, B=0, valid=true) → 0x8000 | 31 << 10 = 0xfc00
    // Slot 1: pure green                                     → 0x83e0
    let bytes = synth_with_slots(0xfc00, 0x83e0);
    let scene = StlDecoder::new().decode(&bytes).unwrap();

    let prim = &scene.meshes[0].primitives[0];
    let convention = prim
        .extras
        .get("stl:color_convention")
        .and_then(|v| v.as_str())
        .expect("decoder should record a colour convention");
    assert_eq!(convention, "viscam");

    let hex = prim
        .extras
        .get("stl:per_face_attributes")
        .and_then(|v| v.as_str())
        .expect("hex round-trip slot should be populated");
    // 0xfc00 LE = 0x00 0xfc; 0x83e0 LE = 0xe0 0x83
    assert_eq!(hex, "00fce083");

    // Re-encode and confirm slots survive byte-identically.
    let reencoded = StlEncoder::new_binary().encode(&scene).unwrap();
    let s0 = u16::from_le_bytes([reencoded[84 + 48], reencoded[84 + 49]]);
    let s1 = u16::from_le_bytes([reencoded[84 + 50 + 48], reencoded[84 + 50 + 49]]);
    assert_eq!(s0, 0xfc00);
    assert_eq!(s1, 0x83e0);

    // Confirm `Stl16BitColor::from_word` agrees with what we encoded.
    let c0 = Stl16BitColor::from_word(ColorConvention::ViscamSolidView, s0);
    assert_eq!(
        c0,
        Stl16BitColor {
            r: 31,
            g: 0,
            b: 0,
            valid: true,
        }
    );
    let c1 = Stl16BitColor::from_word(ColorConvention::ViscamSolidView, s1);
    assert_eq!(
        c1,
        Stl16BitColor {
            r: 0,
            g: 31,
            b: 0,
            valid: true,
        }
    );
}

#[test]
fn materialise_style_attributes_are_detected_and_round_trip() {
    // Two slots with bit 15 clear, non-zero low bits → materialise.
    // Slot 0: red (R=31, valid=true; bit15=0; low 5 bits = 31) → 0x001f
    // Slot 1: blue (B=31, valid=true; bit15=0; bits 10-14 = 31) → 0x7c00
    let bytes = synth_with_slots(0x001f, 0x7c00);
    let scene = StlDecoder::new().decode(&bytes).unwrap();

    let prim = &scene.meshes[0].primitives[0];
    let convention = prim
        .extras
        .get("stl:color_convention")
        .and_then(|v| v.as_str())
        .expect("materialise convention should be detected");
    assert_eq!(convention, "materialise");

    // Re-encode and verify the slot bytes are unchanged.
    let reencoded = StlEncoder::new_binary().encode(&scene).unwrap();
    let s0 = u16::from_le_bytes([reencoded[84 + 48], reencoded[84 + 49]]);
    let s1 = u16::from_le_bytes([reencoded[84 + 50 + 48], reencoded[84 + 50 + 49]]);
    assert_eq!(s0, 0x001f);
    assert_eq!(s1, 0x7c00);

    // Confirm interpretation under the detected convention.
    let c0 = Stl16BitColor::from_word(ColorConvention::Materialise, s0);
    assert_eq!(
        c0,
        Stl16BitColor {
            r: 31,
            g: 0,
            b: 0,
            valid: true,
        }
    );
    let c1 = Stl16BitColor::from_word(ColorConvention::Materialise, s1);
    assert_eq!(
        c1,
        Stl16BitColor {
            r: 0,
            g: 0,
            b: 31,
            valid: true,
        }
    );
}

#[test]
fn ambiguous_bit15_split_yields_no_convention() {
    // Exactly one bit15-set, one bit15-clear → tie → no convention.
    let bytes = synth_with_slots(0x8000, 0x001f);
    let scene = StlDecoder::new().decode(&bytes).unwrap();

    let prim = &scene.meshes[0].primitives[0];
    assert!(
        !prim.extras.contains_key("stl:color_convention"),
        "ambiguous attribute population must not assert a convention"
    );
    // Hex round-trip is still populated.
    assert!(prim.extras.contains_key("stl:per_face_attributes"));
}

#[test]
fn full_round_trip_synthetic_viscam_colors() {
    // Author a four-triangle mesh with explicit VisCAM colours;
    // encode → decode → verify the colour bytes survived.
    let convention = ColorConvention::ViscamSolidView;
    let palette = [
        Stl16BitColor {
            r: 31,
            g: 0,
            b: 0,
            valid: true,
        },
        Stl16BitColor {
            r: 0,
            g: 31,
            b: 0,
            valid: true,
        },
        Stl16BitColor {
            r: 0,
            g: 0,
            b: 31,
            valid: true,
        },
        Stl16BitColor {
            r: 16,
            g: 16,
            b: 16,
            valid: true,
        },
    ];
    let mut buf = Vec::new();
    let mut header = [b' '; 80];
    header[..6].copy_from_slice(b"viscam");
    buf.extend_from_slice(&header);
    buf.extend_from_slice(&(palette.len() as u32).to_le_bytes());
    for c in palette {
        for _ in 0..12 {
            buf.extend_from_slice(&0.0_f32.to_le_bytes());
        }
        buf.extend_from_slice(&c.to_word(convention).to_le_bytes());
    }

    let scene = StlDecoder::new().decode(&buf).unwrap();
    assert_eq!(scene.triangle_count(), 4);
    assert_eq!(
        scene.meshes[0].primitives[0]
            .extras
            .get("stl:color_convention")
            .and_then(|v| v.as_str()),
        Some("viscam")
    );

    let reencoded = StlEncoder::new_binary().encode(&scene).unwrap();
    for (i, expected) in palette.iter().enumerate() {
        let off = 84 + i * 50 + 48;
        let slot = u16::from_le_bytes([reencoded[off], reencoded[off + 1]]);
        let got = Stl16BitColor::from_word(convention, slot);
        assert_eq!(&got, expected, "palette[{}] mismatch", i);
    }
}

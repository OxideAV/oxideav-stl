//! Public-API integration test for [`oxideav_stl::inspect_binary_header`].
//!
//! Exercises the parse-and-inspect path against a vendor-style binary
//! STL whose per-face attribute slot is populated, and confirms the
//! reported counts agree with the full-decode round-trip path on the
//! same byte stream.

use oxideav_stl::{inspect_binary_header, BinaryHeaderReport};

/// Build a small binary STL with caller-supplied attribute pairs.
fn synth(triangles: usize, attrs: &[(u8, u8)]) -> Vec<u8> {
    assert_eq!(attrs.len(), triangles);
    let mut buf = Vec::new();
    buf.extend_from_slice(&[b' '; 80]);
    buf.extend_from_slice(&(triangles as u32).to_le_bytes());
    for &(lo, hi) in attrs {
        // 12 byte normal + 36 byte vertices (all zero is fine — the
        // inspector ignores them) + 2 byte attribute slot.
        buf.extend_from_slice(&[0u8; 48]);
        buf.push(lo);
        buf.push(hi);
    }
    buf
}

#[test]
fn inspector_round_trip_against_synthetic_vendor_style_stream() {
    // Four triangles, two of which carry non-zero (vendor-extension-
    // populated) attribute slots.
    let bytes = synth(4, &[(0, 0), (0x12, 0x34), (0, 0), (0xab, 0xcd)]);

    let rep: BinaryHeaderReport = inspect_binary_header(&bytes).unwrap();
    assert_eq!(rep.triangle_count, 4);
    assert_eq!(rep.actual_byte_length, bytes.len());
    assert_eq!(rep.expected_byte_length, Some(bytes.len()));
    assert!(rep.length_matches_exactly);
    assert_eq!(rep.triangles_walked, 4);
    assert_eq!(rep.non_zero_attribute_count, 2);
    assert!((rep.non_zero_attribute_fraction - 0.5).abs() < 1e-6);
    assert!(!rep.spec_compliant_attributes);

    // The inspector + decoder must agree on the non-zero attribute
    // count for the same byte stream.
    use oxideav_mesh3d::Mesh3DDecoder;
    use oxideav_stl::StlDecoder;
    let scene = StlDecoder::new().decode(&bytes).unwrap();
    let prim = &scene.meshes[0].primitives[0];
    let hex = prim
        .extras
        .get("stl:per_face_attributes")
        .and_then(|v| v.as_str())
        .expect("non-zero attrs surface on decoded extras");
    let mut decoder_non_zero = 0usize;
    for chunk in hex.as_bytes().chunks_exact(4) {
        let lo = u8::from_str_radix(std::str::from_utf8(&chunk[..2]).unwrap(), 16).unwrap();
        let hi = u8::from_str_radix(std::str::from_utf8(&chunk[2..]).unwrap(), 16).unwrap();
        if lo != 0 || hi != 0 {
            decoder_non_zero += 1;
        }
    }
    assert_eq!(decoder_non_zero, rep.non_zero_attribute_count);
}

#[test]
fn empty_triangle_list_is_spec_compliant() {
    // 84-byte file: header + count=0, no triangle records.
    let bytes = synth(0, &[]);
    let rep = inspect_binary_header(&bytes).unwrap();
    assert!(rep.spec_compliant_attributes);
    assert!(rep.length_matches_exactly);
    assert!(rep.non_zero_attribute_fraction.is_nan());
}

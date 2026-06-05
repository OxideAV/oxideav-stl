//! Byte-stream-level binary-STL header inspector.
//!
//! [`inspect_binary_header`] walks a binary STL byte slice WITHOUT
//! constructing a [`Scene3D`]; it returns a typed
//! [`BinaryHeaderReport`] summarising the header layout and the
//! distribution of the per-triangle `uint16` attribute-byte-count
//! slot.
//!
//! The 1989 spec (per `docs/3d/stl/fabbers-stl-format.html` §6.5.3)
//! states that the per-triangle attribute byte count "should be set
//! to zero". Vendors (Materialise / VisCAM / SolidView) repurpose the
//! slot for per-face colour packing, so a non-zero population is a
//! strong hint that a vendor extension is in play. This inspector
//! surfaces that hint as a `spec_compliant_attributes` boolean +
//! `non_zero_attribute_count` cardinality so consumers can decide
//! pre-decode whether to expect vendor-extension payloads (or to
//! reject strict-spec inputs that carry them).
//!
//! Distinct from [`crate::color::detect`], which classifies a *bit
//! distribution* across vendor conventions: this inspector reports
//! the raw header-level facts only.
//!
//! The function is allocation-free aside from the
//! [`BinaryHeaderReport`] return value (which is `Copy`-sized but
//! returned by value); it never builds a `Scene3D`, never copies the
//! triangle records, and never allocates intermediate `Vec`s.

use oxideav_mesh3d::{Error, Result};

use crate::binary::{HEADER_BYTES, TRIANGLE_BYTES};

/// Outcome of an [`inspect_binary_header`] call.
///
/// Every field is derived from a single forward pass over the byte
/// stream; the struct is `Copy` so the caller can stash it in a log
/// or scoreboard without juggling lifetimes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BinaryHeaderReport {
    /// `triangle_count` field at offset 80 (little-endian `u32`).
    pub triangle_count: u32,
    /// Exact byte length the file *should* have for
    /// [`Self::triangle_count`] triangles: `84 + N * 50`. Stays
    /// `None` when the multiplication would overflow `usize`.
    pub expected_byte_length: Option<usize>,
    /// Actual length of the input slice.
    pub actual_byte_length: usize,
    /// `true` iff [`Self::expected_byte_length`] matches
    /// [`Self::actual_byte_length`] exactly. Files with a trailing
    /// vendor footer or padding read `false`; truncated files read
    /// `false`.
    pub length_matches_exactly: bool,
    /// Number of triangle records whose `uint16` attribute slot is
    /// non-zero. Zero on a strictly-spec-compliant file. Walks at
    /// most `min(triangle_count, observed records)` slots — the
    /// inspector stops at the end of the slice rather than reading
    /// past it.
    pub non_zero_attribute_count: usize,
    /// `non_zero_attribute_count / triangle_count` as a fraction,
    /// clamped to `[0.0, 1.0]`. `NaN` when `triangle_count == 0`
    /// (no facets to inspect).
    pub non_zero_attribute_fraction: f32,
    /// `true` iff every walked triangle's attribute slot equalled
    /// zero (the 1989 spec rule). Vacuously `true` on an empty
    /// triangle list. Equivalent to
    /// `non_zero_attribute_count == 0`.
    pub spec_compliant_attributes: bool,
    /// Number of triangle records the inspector was actually able to
    /// walk before reaching the end of the slice. Equals
    /// [`Self::triangle_count`] iff the file is at least
    /// `84 + N * 50` bytes long.
    pub triangles_walked: usize,
}

/// Inspect the header + per-triangle attribute slots of a binary STL
/// byte slice.
///
/// Returns [`Error::InvalidData`] when the slice is shorter than the
/// 84-byte header-plus-count prefix (no valid binary STL is shorter).
/// Otherwise the function always succeeds — a slice whose
/// `triangle_count` field claims more triangles than the slice can
/// physically hold is *not* an error here; the report reflects what
/// could be walked under [`BinaryHeaderReport::triangles_walked`]
/// with [`BinaryHeaderReport::length_matches_exactly`] = `false`.
/// Callers reaching for the full decode path should use
/// [`crate::binary::decode`] (or [`crate::StlDecoder`]), which does
/// reject truncated bodies.
///
/// This function does NOT attempt to distinguish ASCII vs binary —
/// it is the caller's responsibility to route binary bytes here. The
/// 80-byte header is treated as opaque (the report does not parse
/// out any Materialise `COLOR=` / `MATERIAL=` lines; see
/// [`crate::materialise_header`] for that).
pub fn inspect_binary_header(bytes: &[u8]) -> Result<BinaryHeaderReport> {
    if bytes.len() < HEADER_BYTES {
        return Err(Error::InvalidData(format!(
            "binary STL inspector needs at least {HEADER_BYTES} bytes for header + count, got {}",
            bytes.len()
        )));
    }
    let triangle_count = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]);

    // `usize::checked_mul` rules out the pathological case of a 4 GiB
    // triangle count on a 32-bit usize host. Falls back to `None` so
    // the report can still report what was observed.
    let expected_byte_length = (triangle_count as usize)
        .checked_mul(TRIANGLE_BYTES)
        .and_then(|body| body.checked_add(HEADER_BYTES));
    let length_matches_exactly = expected_byte_length == Some(bytes.len());

    // Decide how many triangle records the inspector can actually
    // see. The slice may be shorter than `triangle_count` claims; we
    // cap at the observed body length and let the report's
    // `length_matches_exactly` flag surface the truncation.
    let body = &bytes[HEADER_BYTES..];
    let observed_records = body.len() / TRIANGLE_BYTES;
    let triangles_walked = observed_records.min(triangle_count as usize);

    // Single pass over the attribute slot of each walked triangle.
    // No allocations, no Vec growth, no Scene3D construction.
    let mut non_zero_attribute_count: usize = 0;
    for record_idx in 0..triangles_walked {
        let base = record_idx * TRIANGLE_BYTES;
        // Layout per §6.5.3: 12 bytes normal + 36 bytes vertices +
        // 2 bytes attribute_byte_count (LE u16). We only need the
        // last two bytes.
        let lo = body[base + 48];
        let hi = body[base + 49];
        if lo != 0 || hi != 0 {
            non_zero_attribute_count += 1;
        }
    }

    let non_zero_attribute_fraction = if triangle_count == 0 {
        f32::NAN
    } else {
        let walked = triangles_walked as f32;
        if walked == 0.0 {
            0.0
        } else {
            (non_zero_attribute_count as f32 / walked).clamp(0.0, 1.0)
        }
    };

    Ok(BinaryHeaderReport {
        triangle_count,
        expected_byte_length,
        actual_byte_length: bytes.len(),
        length_matches_exactly,
        non_zero_attribute_count,
        non_zero_attribute_fraction,
        spec_compliant_attributes: non_zero_attribute_count == 0,
        triangles_walked,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a single-triangle binary STL with caller-chosen
    /// attribute bytes. Header is filled with spaces so the prefix
    /// never collides with `b"solid "` — irrelevant for the
    /// inspector but a habit worth keeping.
    fn synth(triangles: usize, attrs: &[(u8, u8)]) -> Vec<u8> {
        assert_eq!(
            attrs.len(),
            triangles,
            "test helper requires one (lo, hi) pair per triangle"
        );
        let mut buf = Vec::new();
        buf.extend_from_slice(&[b' '; 80]);
        buf.extend_from_slice(&(triangles as u32).to_le_bytes());
        for &(lo, hi) in attrs {
            // 12 zero normal bytes + 36 zero vertex bytes + 2 attr bytes.
            buf.extend_from_slice(&[0u8; 48]);
            buf.push(lo);
            buf.push(hi);
        }
        buf
    }

    #[test]
    fn rejects_slice_shorter_than_header() {
        let err = inspect_binary_header(&[0u8; 10]).unwrap_err();
        let msg = format!("{err:?}");
        assert!(msg.contains("inspector"), "{msg}");
    }

    #[test]
    fn empty_triangle_list_is_spec_compliant_and_length_exact() {
        let bytes = synth(0, &[]);
        let rep = inspect_binary_header(&bytes).unwrap();
        assert_eq!(rep.triangle_count, 0);
        assert_eq!(rep.expected_byte_length, Some(HEADER_BYTES));
        assert_eq!(rep.actual_byte_length, HEADER_BYTES);
        assert!(rep.length_matches_exactly);
        assert_eq!(rep.non_zero_attribute_count, 0);
        assert!(rep.non_zero_attribute_fraction.is_nan());
        assert!(rep.spec_compliant_attributes);
        assert_eq!(rep.triangles_walked, 0);
    }

    #[test]
    fn all_zero_attributes_classify_as_spec_compliant() {
        let bytes = synth(3, &[(0, 0), (0, 0), (0, 0)]);
        let rep = inspect_binary_header(&bytes).unwrap();
        assert_eq!(rep.triangle_count, 3);
        assert_eq!(
            rep.expected_byte_length,
            Some(HEADER_BYTES + 3 * TRIANGLE_BYTES)
        );
        assert_eq!(rep.actual_byte_length, bytes.len());
        assert!(rep.length_matches_exactly);
        assert_eq!(rep.triangles_walked, 3);
        assert_eq!(rep.non_zero_attribute_count, 0);
        assert_eq!(rep.non_zero_attribute_fraction, 0.0);
        assert!(rep.spec_compliant_attributes);
    }

    #[test]
    fn mixed_attribute_population_reports_fraction() {
        // Three triangles, one with non-zero attributes — fraction
        // 1/3 ≈ 0.333…
        let bytes = synth(3, &[(0, 0), (0x12, 0x34), (0, 0)]);
        let rep = inspect_binary_header(&bytes).unwrap();
        assert_eq!(rep.non_zero_attribute_count, 1);
        assert!((rep.non_zero_attribute_fraction - (1.0 / 3.0)).abs() < 1e-6);
        assert!(!rep.spec_compliant_attributes);
        assert!(rep.length_matches_exactly);
    }

    #[test]
    fn all_non_zero_attributes_yield_fraction_one() {
        let bytes = synth(2, &[(0xab, 0xcd), (0xff, 0xff)]);
        let rep = inspect_binary_header(&bytes).unwrap();
        assert_eq!(rep.non_zero_attribute_count, 2);
        assert_eq!(rep.non_zero_attribute_fraction, 1.0);
        assert!(!rep.spec_compliant_attributes);
    }

    #[test]
    fn truncated_body_walks_only_observed_records() {
        // Claim 5 triangles, but provide only 2 records worth of body.
        let mut bytes = synth(2, &[(0, 0), (0xaa, 0xbb)]);
        // Rewrite the count field to lie about 5 triangles.
        bytes[80..84].copy_from_slice(&5u32.to_le_bytes());
        let rep = inspect_binary_header(&bytes).unwrap();
        assert_eq!(rep.triangle_count, 5);
        assert_eq!(rep.triangles_walked, 2);
        assert!(!rep.length_matches_exactly);
        assert_eq!(rep.non_zero_attribute_count, 1);
        // Fraction is over walked, not claimed.
        assert!((rep.non_zero_attribute_fraction - 0.5).abs() < 1e-6);
        assert!(!rep.spec_compliant_attributes);
    }

    #[test]
    fn inspector_then_decode_round_trip_consistent() {
        // The inspector's `non_zero_attribute_count` must match what
        // the full decoder surfaces via
        // `stl:per_face_attributes` (hex string of 2 × N bytes).
        let bytes = synth(4, &[(0, 0), (0x12, 0x34), (0, 0), (0xab, 0xcd)]);
        let rep = inspect_binary_header(&bytes).unwrap();
        assert_eq!(rep.non_zero_attribute_count, 2);

        // Now decode and compare. The decoder records the bytes as a
        // single hex string under the per-face-attributes key; we
        // re-count non-zero pairs from that string and confirm
        // the inspector's lightweight pass agrees.
        let scene = crate::binary::decode(&bytes).unwrap();
        let prim = &scene.meshes[0].primitives[0];
        let hex = prim
            .extras
            .get("stl:per_face_attributes")
            .and_then(|v| v.as_str())
            .expect("non-zero attrs should surface on decoded extras");
        // Each 4 hex chars = 2 bytes = one triangle's attribute slot.
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
    fn report_is_copy_and_debug() {
        // The fixed-size report is `Copy` so callers can stash it in
        // a log/scoreboard without lifetime juggling.
        let bytes = synth(1, &[(0, 0)]);
        let rep = inspect_binary_header(&bytes).unwrap();
        let copied = rep;
        assert_eq!(rep, copied);
        // And `Debug`-printable for diagnostic logs.
        let _ = format!("{rep:?}");
    }
}

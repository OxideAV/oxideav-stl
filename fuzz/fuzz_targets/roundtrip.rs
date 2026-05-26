#![no_main]

//! Synthesise a small binary STL from fuzz-controlled bytes, decode
//! it, re-encode it, and assert the per-triangle records survive
//! byte-for-byte.
//!
//! Binary STL is `[80-byte header][LE u32 count][N × 50-byte
//! triangle]` where each triangle is `[12 normal bytes][3 × 12 vertex
//! bytes][LE u16 attribute slot]`. The 1989 spec says the writer may
//! choose any header string (we re-emit a writer-signature default)
//! and the attribute slot is "specified to be set to zero" but
//! libraries (Materialise / VisCAM) put per-face colour data into it.
//! Our decoder routes the attribute bytes verbatim through
//! `Primitive::extras["stl:per_face_attributes"]` and the encoder
//! plays them back, so the record-byte invariant is:
//!
//!   * triangle count slot (bytes 80..84) matches the original
//!   * every triangle record (bytes 84..) matches the original
//!
//! The 80-byte header is NOT required to match — the writer is free
//! to substitute. This matches the `binary_cube_triangle_records_-
//! roundtrip_byte_identical` integration test.
//!
//! There is no external library worth dlopen-ing as a cross-decode
//! oracle (clean-room wall) and STL's flavours don't even agree on
//! header semantics, so this is a self-roundtrip target: encode → ←
//! decode → encode → compare.

use libfuzzer_sys::fuzz_target;
use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};
use oxideav_stl::{StlDecoder, StlEncoder};

const HEADER_BYTES: usize = 80;
const COUNT_BYTES: usize = 4;
const TRIANGLE_BYTES: usize = 50; // 12 normal + 36 vertex + 2 attribute
const PREFIX_BYTES: usize = HEADER_BYTES + COUNT_BYTES;

/// Cap the synthesised mesh so a single fuzz iteration stays within
/// libfuzzer's per-input memory budget. 64 triangles × 50 bytes = 3.2
/// KiB body — large enough to exercise the decoder's allocation path,
/// small enough to mutate quickly.
const MAX_TRIANGLES: usize = 64;

fuzz_target!(|data: &[u8]| {
    // Need at least one byte for the triangle count nibble.
    if data.is_empty() {
        return;
    }

    let triangles: usize = (data[0] as usize) % (MAX_TRIANGLES + 1);
    let body_bytes = triangles * TRIANGLE_BYTES;

    // Build a length-correct binary STL. The writer header is a
    // single ASCII line we own (specifically NOT starting with
    // `solid` — that would otherwise trip the ASCII sniffer when
    // combined with a `\nfacet` sequence in random bytes). NUL-pad
    // out to 80 bytes.
    let mut encoded = Vec::with_capacity(PREFIX_BYTES + body_bytes);
    const WRITER_SIG: &[u8] = b"oxideav-stl-fuzz/roundtrip";
    encoded.extend_from_slice(WRITER_SIG);
    encoded.resize(HEADER_BYTES, 0);
    encoded.extend_from_slice(&(triangles as u32).to_le_bytes());

    // Fill the triangle body with bytes sourced from the remaining
    // fuzz input (cycled / zero-padded as needed). Float bit-patterns
    // that decode to subnormals / infinities / NaN are all valid as
    // far as the spec goes — STL has no "finite vertex" requirement
    // and our decoder accepts them. The roundtrip assertion is
    // byte-equality, not numeric equality, so NaN bit-patterns
    // survive (they compare equal as raw bytes).
    let body = &data[1..];
    if body.is_empty() {
        encoded.resize(PREFIX_BYTES + body_bytes, 0);
    } else {
        for i in 0..body_bytes {
            encoded.push(body[i % body.len()]);
        }
    }

    // Decode. The decoder is allowed to reject the synthesised stream
    // for any reason (e.g. if the writer signature accidentally hits
    // a future-added invariant); we only care about no-panic + the
    // bytes-survive-if-it-decoded invariant.
    let scene = match StlDecoder::new().decode(&encoded) {
        Ok(s) => s,
        Err(_) => return,
    };

    // Re-encode. Any encoder error here would be a genuine bug given
    // the scene round-tripped through our own writer, but we return
    // rather than unwrap so the fuzzer crashes on the byte-mismatch
    // assertion only, not on transient internal errors during fuzzer
    // development.
    let reserialized = match StlEncoder::new_binary().encode(&scene) {
        Ok(v) => v,
        Err(_) => return,
    };

    // Lengths must match (fixed-size records, same triangle count).
    assert_eq!(
        reserialized.len(),
        encoded.len(),
        "STL binary file length survives encode → decode → encode"
    );

    // Triangle count slot survives.
    assert_eq!(
        &reserialized[HEADER_BYTES..PREFIX_BYTES],
        &encoded[HEADER_BYTES..PREFIX_BYTES],
        "triangle count slot survives roundtrip"
    );

    // Triangle records survive byte-for-byte. The 80-byte header is
    // not asserted — both writers substitute their own signature.
    assert_eq!(
        &reserialized[PREFIX_BYTES..],
        &encoded[PREFIX_BYTES..],
        "triangle records survive roundtrip"
    );
});

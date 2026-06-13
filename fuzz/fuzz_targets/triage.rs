#![no_main]

//! Drive the three attacker-reachable *triage* entry points on
//! arbitrary fuzz-supplied bytes: `lint_ascii`, `inspect_binary_header`,
//! and `detect_color_convention`.
//!
//! These are the pre-decode inspectors a pipeline reaches for *before*
//! it commits to a full `StlDecoder::decode`. Each one walks the input
//! bytes with its **own** hand-rolled scanner — none of them route
//! through the decoder — so each is a distinct parser surface the
//! `decode` / `roundtrip` targets never exercise:
//!
//! * **`lint_ascii`** — re-walks the ASCII grammar with the same
//!   tolerance as `ascii::decode` but builds an `AsciiLintReport`
//!   instead of a `Scene3D`. Independent token loop, independent
//!   leading-whitespace/tab scan, independent `vertex`-coordinate
//!   sign lexing, independent multi-`solid` counter. Danger spots:
//!   a truncated facet block, a keyword that ends exactly at EOF, a
//!   line that is all leading whitespace, a UTF-8 BOM with nothing
//!   after it, a `vertex` token that is a bare `-`, and the
//!   `MAX_REPORTED_LINT_FINDINGS` example-list cap (the counter must
//!   keep counting after the example vector stops growing). The call
//!   must always return a `Result`, never panic / index past the
//!   slice / overflow a counter.
//!
//! * **`inspect_binary_header`** — reads the 80-byte header, the LE
//!   `u32` triangle count at offset 80, and the per-triangle `u16`
//!   attribute slot, **without** allocating `count * 50` bytes. A
//!   hostile `0xFFFF_FFFF` count on a 84-byte slice must walk zero
//!   records (the `checked_mul` / `observed_records.min(count)` cap),
//!   not read past the slice; the `triangle_count == 0` branch must
//!   produce a `NaN` fraction without dividing by a live zero. The
//!   only `Err` is the sub-header-length slice; everything else must
//!   classify.
//!
//! * **`detect_color_convention`** — classifies a flat byte buffer of
//!   per-face `u16` attribute slots into the VisCAM vs Materialise
//!   15-bit-RGB conventions from the bit-15 distribution. An
//!   odd-length buffer (dangling final byte), an empty buffer, and an
//!   all-`0xFF` buffer must each classify-or-decline without panicking
//!   on the `chunks_exact(2)` remainder or a divide-by-zero in the
//!   distribution ratio.
//!
//! The input is split so a single fuzz buffer drives all three: the
//! whole slice goes to `lint_ascii` and `inspect_binary_header`
//! (both tolerate any bytes), and the slice past the 84-byte binary
//! prefix — the attribute-bearing body region — feeds the colour
//! detector, mirroring how a real consumer would hand the detector the
//! concatenated attribute slots.
//!
//! Contract under test: *the call returns*. The return values are
//! intentionally discarded. No external library is consulted as an
//! oracle (clean-room wall); panic-freedom is the whole invariant.

use libfuzzer_sys::fuzz_target;
use oxideav_stl::{detect_color_convention, inspect_binary_header, lint_ascii};

/// Binary STL fixed prefix: 80-byte header + LE u32 triangle count.
const BINARY_PREFIX_BYTES: usize = 84;

fuzz_target!(|data: &[u8]| {
    // ASCII strict-spec lint: independent token walk + tab/sign/BOM
    // scanners. Must return a Result on every input.
    let _ = lint_ascii(data);

    // Binary header inspector: count-slot + attribute-slot walk with
    // no `count * 50` pre-allocation. Returns Err only on a slice
    // shorter than the 84-byte prefix; classifies everything else.
    let _ = inspect_binary_header(data);

    // Colour-convention detector over the attribute-body region. Feed
    // it the bytes past the binary prefix when present (where the
    // per-face u16 attribute slots live), else the whole slice — both
    // must tolerate any length, including an odd dangling byte.
    let attribute_region = if data.len() > BINARY_PREFIX_BYTES {
        &data[BINARY_PREFIX_BYTES..]
    } else {
        data
    };
    let _ = detect_color_convention(attribute_region);
});

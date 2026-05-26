#![no_main]

//! Decode arbitrary fuzz-supplied bytes through `StlDecoder::decode`.
//!
//! The decoder must always return a `Result` and never panic / abort
//! / OOM, regardless of how malformed the input is. The return value
//! is intentionally discarded — the contract under test is *the call
//! returns*, not what it returns.
//!
//! Classic STL danger spots this target drives:
//!
//! * **ASCII vs binary sniff** — a 6-byte input of `b"solid "` plus a
//!   few stray bytes will pass prefix 1+2 but should fail the
//!   token-cross-check (3) and then route to binary, which will
//!   itself reject the tiny payload. Hostile inputs that satisfy
//!   `84 + N*50` exactly with a `b"solid"` prefix should route to
//!   binary regardless.
//! * **Triangle-count over-allocation (binary)** — bytes 80..84 are
//!   an LE `uint32`. With `0xFFFFFFFF` and a 100-byte input the
//!   decoder must cross-check the announced count against the
//!   remaining slice (`count * 50` ≤ remaining) before allocating
//!   anything; a hostile count is rejected as `Err(Error::…)`.
//! * **ASCII keyword walk** — `solid` / `facet normal` / `outer
//!   loop` / `vertex` / `endloop` / `endfacet` / `endsolid`. A
//!   truncated facet block, a missing keyword, or a non-finite float
//!   literal must surface as `Err(…)` rather than arithmetic-overflow
//!   or index-out-of-bounds.
//! * **Multi-`solid` ASCII** — concatenated `solid NAME … endsolid
//!   NAME` blocks each become a `Mesh`. A hostile file declaring
//!   thousands of empty solids must not blow the mesh-vector
//!   allocator without an upper bound on the byte budget.
//! * **Materialise / VisCAM per-face colour detection** — the
//!   16-bit attribute slot is interpreted as `[valid:1][R:5][G:5]
//!   [B:5]` (VisCAM) or `[valid:1][B:5][G:5][R:5]` (Materialise) and
//!   the convention is picked from the bit-15 distribution. Any
//!   distribution must classify (or refuse to classify) without
//!   panicking.
//! * **UTF-8 BOM + whitespace skip** — the sniffer strips a leading
//!   BOM (`EF BB BF`) and ASCII whitespace before applying the
//!   `solid` prefix test. Pathological whitespace inputs must not
//!   make it spin or index past the slice.
//! * **`84 + N*50` size override** — even when the bytes look like
//!   ASCII (`b"solid"` prefix + `\nfacet` token), an exact-size match
//!   classifies as binary. A 1-byte input or an `N=0` declaration
//!   must take the right branch and not divide by zero.

use libfuzzer_sys::fuzz_target;
use oxideav_mesh3d::Mesh3DDecoder;
use oxideav_stl::StlDecoder;

fuzz_target!(|data: &[u8]| {
    let _ = StlDecoder::new().decode(data);
});

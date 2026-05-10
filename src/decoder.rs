//! [`StlDecoder`] — bytes-in, [`Scene3D`]-out.
//!
//! Sniffs the first six bytes of the input: `b"solid "` followed by a
//! `\nfacet` / `\n  facet` token in the first 1 KiB → ASCII path,
//! otherwise binary. The sniff defends against the well-known trap of
//! binary STL files whose 80-byte vendor header begins with the
//! literal `solid` ASCII string (Microsoft toolchains, CADKey, some
//! AutoCAD exports).

use oxideav_mesh3d::{Mesh3DDecoder, Result, Scene3D};

use crate::{ascii, binary};

/// STL decoder — implements [`Mesh3DDecoder`].
#[derive(Debug, Default)]
pub struct StlDecoder {
    _private: (),
}

impl StlDecoder {
    /// Construct a fresh decoder with default options.
    pub fn new() -> Self {
        Self::default()
    }
}

impl Mesh3DDecoder for StlDecoder {
    fn decode(&mut self, bytes: &[u8]) -> Result<Scene3D> {
        if is_ascii_stl(bytes) {
            ascii::decode(bytes)
        } else {
            binary::decode(bytes)
        }
    }
}

/// UTF-8 byte-order mark — sometimes prepended by Windows-side text
/// editors to ASCII STL files. We accept it so the prefix sniff still
/// fires after the BOM.
const UTF8_BOM: &[u8] = &[0xEF, 0xBB, 0xBF];

/// Heuristic ASCII-vs-binary detector.
///
/// Returns `true` when the input is *probably* ASCII STL. The detector
/// applies a layered set of signals:
///
/// 1. **Optional UTF-8 BOM and leading whitespace** are skipped before
///    sniffing — some text-editor pipelines insert these into ASCII
///    files and they would otherwise prevent the prefix match.
/// 2. **Prefix** must be `b"solid"` followed by a whitespace byte
///    (space, tab, CR, LF). The trailing whitespace catches the
///    Microsoft-toolchain trap header `b"solid "` while still rejecting
///    binary headers like `b"solidWORKS"` whose first 6 bytes happen to
///    be `b"solidW"`.
/// 3. **Token cross-check** — within the first 1 KiB after the prefix
///    we must see a `\n`+optional-whitespace+`facet` token. Binary
///    payload bytes are overwhelmingly non-printable and won't form
///    that sequence by accident.
/// 4. **Binary-size sanity check (NEW r2)** — even when 1+2+3 all pass,
///    if the file length matches `84 + N*50` exactly where `N` is the
///    little-endian `u32` at offset 80 AND `N > 0`, we override to
///    binary. This catches CADKey-2002 / Microsoft / AutoCAD binary
///    headers that happen to embed the substring `\n facet` in their
///    80-byte vendor string. False-positive risk on a real ASCII file
///    is negligible: the file would have to be exactly the right size
///    for some triangle count chosen by the four-byte slot at offset
///    80, which is astronomically unlikely.
pub(crate) fn is_ascii_stl(bytes: &[u8]) -> bool {
    let trimmed = strip_bom_and_leading_ws(bytes);
    if !looks_like_ascii_prefix(trimmed) {
        return false;
    }
    // Token cross-check on the 1 KiB ASCII-grammar window.
    let window_end = trimmed.len().min(1024);
    let window = &trimmed[..window_end];
    if !has_facet_keyword(window) {
        return false;
    }
    // Binary-size cross-check (r2). Only apply when the input is large
    // enough to have a triangle-count slot AND the slot indicates a
    // non-empty mesh — empty STLs (count = 0) are usually authoring-
    // tool placeholders that we let the token cross-check handle.
    if bytes.len() >= crate::binary::HEADER_BYTES {
        let n = u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]) as usize;
        if n > 0 {
            // Use checked arithmetic — `n * 50` could overflow on a
            // pathological count. If overflow occurs, fall through to
            // ASCII (the binary parser will reject it anyway).
            if let Some(body) = n.checked_mul(crate::binary::TRIANGLE_BYTES) {
                if let Some(total) = body.checked_add(crate::binary::HEADER_BYTES) {
                    if bytes.len() == total {
                        return false;
                    }
                }
            }
        }
    }
    true
}

/// Skip a leading UTF-8 BOM and ASCII whitespace (`' '`, `'\t'`,
/// `'\r'`, `'\n'`). Returns the suffix that remains.
fn strip_bom_and_leading_ws(bytes: &[u8]) -> &[u8] {
    let mut s = bytes;
    if s.starts_with(UTF8_BOM) {
        s = &s[UTF8_BOM.len()..];
    }
    let mut i = 0;
    while i < s.len() {
        let b = s[i];
        if b == b' ' || b == b'\t' || b == b'\r' || b == b'\n' {
            i += 1;
        } else {
            break;
        }
    }
    &s[i..]
}

/// Returns `true` if `s` begins with the keyword `solid` followed by
/// an ASCII whitespace byte (space, tab, CR, LF). A bare `b"solid"`
/// suffix (zero-length file or `b"solidWORKS"` binary header) is
/// rejected.
fn looks_like_ascii_prefix(s: &[u8]) -> bool {
    if s.len() < 6 {
        return false;
    }
    if !s[..5].eq_ignore_ascii_case(b"solid") {
        return false;
    }
    matches!(s[5], b' ' | b'\t' | b'\r' | b'\n')
}

fn has_facet_keyword(window: &[u8]) -> bool {
    // Scan for `\n`+ optional space/tab run + `facet`.
    let mut i = 0;
    while i < window.len() {
        if window[i] == b'\n' {
            let mut j = i + 1;
            while j < window.len() && (window[j] == b' ' || window[j] == b'\t') {
                j += 1;
            }
            if window.len() - j >= 5 && window[j..j + 5].eq_ignore_ascii_case(b"facet") {
                return true;
            }
        }
        i += 1;
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ascii_sniff_matches_canonical_ascii() {
        let s = "solid foo\n  facet normal 0 0 1\n";
        assert!(is_ascii_stl(s.as_bytes()));
    }

    #[test]
    fn ascii_sniff_rejects_binary_with_solid_header() {
        // 80-byte header beginning with the literal `solid`, then 0
        // triangles. The first six bytes pass the prefix check but
        // there's no `\nfacet` token in the buffer.
        let mut buf = vec![b' '; 80];
        buf[..5].copy_from_slice(b"solid");
        buf.extend_from_slice(&0u32.to_le_bytes());
        // Pad header so prefix is `b"solid "` (the ' ' at byte 5).
        buf[5] = b' ';
        assert!(!is_ascii_stl(&buf));
    }

    #[test]
    fn ascii_sniff_rejects_short_input() {
        assert!(!is_ascii_stl(b""));
        assert!(!is_ascii_stl(b"sol"));
    }

    #[test]
    fn ascii_sniff_rejects_other_prefixes() {
        // Binary file whose header starts with something else.
        let mut buf = vec![0u8; 84];
        buf[0..5].copy_from_slice(b"abcde");
        assert!(!is_ascii_stl(&buf));
    }

    #[test]
    fn ascii_sniff_rejects_solidworks_binary_prefix() {
        // `b"solidWORKS"` — no whitespace after `solid`, so the prefix
        // check rejects this without ever inspecting the body.
        let mut buf = vec![0u8; 84];
        buf[..10].copy_from_slice(b"solidWORKS");
        assert!(!is_ascii_stl(&buf));
    }

    #[test]
    fn ascii_sniff_skips_utf8_bom() {
        // Some Windows text editors prepend a UTF-8 BOM to ASCII STL
        // files; the prefix check must succeed despite the leading 3
        // bytes.
        let mut buf = Vec::new();
        buf.extend_from_slice(UTF8_BOM);
        buf.extend_from_slice(b"solid foo\n  facet normal 0 0 1\n");
        assert!(is_ascii_stl(&buf));
    }

    #[test]
    fn ascii_sniff_skips_leading_whitespace() {
        // Real-world ASCII files occasionally have a leading newline
        // or spaces before `solid`.
        let buf = b"\n\nsolid foo\n  facet normal 0 0 1\n";
        assert!(is_ascii_stl(buf));
    }

    #[test]
    fn binary_size_cross_check_overrides_when_n_matches() {
        // Synthesise a binary STL whose 80-byte header contains
        // BOTH the `b"solid "` prefix AND a `\nfacet` token (a CADKey-
        // style adversarial header). The size cross-check should still
        // route it to the binary parser because the byte count matches
        // 84 + 1*50 = 134.
        let mut header = [b' '; 80];
        header[..6].copy_from_slice(b"solid ");
        // Embed `\n facet` partway through the header — the prefix +
        // token sniff alone would mistake this for ASCII.
        header[40..48].copy_from_slice(b"\n facet ");
        let mut buf = Vec::with_capacity(134);
        buf.extend_from_slice(&header);
        buf.extend_from_slice(&1u32.to_le_bytes());
        // One triangle (50 bytes) of zeros.
        buf.extend_from_slice(&[0u8; 50]);
        assert_eq!(buf.len(), 134);
        // Without the size cross-check this would be misclassified;
        // with it, we route to binary.
        assert!(!is_ascii_stl(&buf));
    }

    #[test]
    fn binary_size_cross_check_skips_when_n_zero() {
        // If the count slot is zero, the size check is inert — fall
        // back to the token cross-check. A header beginning with
        // `b"solid "` but no `\nfacet` still routes to binary because
        // the token check fails.
        let mut buf = vec![b' '; 80];
        buf[..6].copy_from_slice(b"solid ");
        buf.extend_from_slice(&0u32.to_le_bytes());
        assert!(!is_ascii_stl(&buf));
    }

    #[test]
    fn ascii_with_size_collision_still_decodes_as_ascii() {
        // Pathological case: an ASCII file whose total length happens
        // to equal `84 + N*50` for the N spelled out by the bytes at
        // offset 80. Constructing such a file deliberately would
        // require very tight control over the float formatter — in
        // practice ASCII files have variable per-vertex widths and
        // never hit a collision with this formula. Sanity-check a
        // canonical small ASCII file and confirm we still classify
        // it as ASCII.
        let s = "solid x\n  facet normal 0 0 1\n    outer loop\n      vertex 0 0 0\n      vertex 1 0 0\n      vertex 0 1 0\n    endloop\n  endfacet\nendsolid x\n";
        assert!(is_ascii_stl(s.as_bytes()));
    }
}

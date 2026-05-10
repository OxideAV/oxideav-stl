//! [`StlDecoder`] — bytes-in, [`Scene3D`]-out.
//!
//! Sniffs the first six bytes of the input: `b"solid "` followed by a
//! `\nfacet` / `\n  facet` token in the first 1 KiB → ASCII path,
//! otherwise binary. The sniff defends against the well-known trap of
//! binary STL files whose 80-byte vendor header begins with the
//! literal `solid` ASCII string.

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

/// Heuristic ASCII-vs-binary detector.
///
/// Returns `true` when the input is *probably* ASCII STL:
///
/// - Starts with `b"solid "` (note: trailing space — a bare `b"solid"`
///   match would also catch the binary header trap).
/// - Within the first 1 KiB we see one of the keyword pairs that has
///   no realistic collision with a binary header: `\nfacet`,
///   `\n facet`, or `\n\tfacet`.
///
/// We require *both* signals so binary files whose 80-byte header
/// begins with the literal `solid` (some Microsoft toolchains do this)
/// don't get false-positively dispatched to the ASCII parser.
pub(crate) fn is_ascii_stl(bytes: &[u8]) -> bool {
    if bytes.len() < 6 || !bytes.starts_with(b"solid ") {
        return false;
    }
    // Search a bounded window for a clear ASCII-format token.
    let window_end = bytes.len().min(1024);
    let window = &bytes[..window_end];
    // Look for `facet` preceded by a newline and any whitespace
    // (spaces, tabs, or directly after the newline). This is robust
    // against a binary STL whose header happened to start with the
    // ASCII string `solid` because the binary payload past byte 80
    // is overwhelmingly non-printable and won't contain `\nfacet`.
    has_facet_keyword(window)
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
}

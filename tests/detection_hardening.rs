//! Round-2 false-positive hardening for ASCII-vs-binary detection.
//!
//! Survey of known real-world headers that defeat the naive prefix
//! sniff (`b"solid"` alone), plus the layered defences this crate
//! adds in r2:
//!
//! - **CADKey-2002** — emits binary STL files whose 80-byte header
//!   starts with `b"solid CADKey 2002"`. The byte-count cross-check
//!   (`bytes.len() == 84 + N*50`) overrides the prefix sniff.
//! - **AutoCAD STL exports with leading whitespace** — some pipelines
//!   prepend a stray newline or BOM; the detector strips both before
//!   inspecting the prefix.
//! - **SolidWorks-style adversarial headers** — `b"solidWORKS"` lacks
//!   the trailing whitespace and is rejected at the prefix stage.

use oxideav_mesh3d::Mesh3DDecoder;
use oxideav_stl::StlDecoder;

/// Build a minimal valid binary STL with the supplied 80-byte header
/// content + `tri_count` zero-payload triangles.
fn binary_with_header(header_text: &[u8], tri_count: u32) -> Vec<u8> {
    assert!(
        header_text.len() <= 80,
        "test fixture: header text must fit"
    );
    let mut buf = Vec::with_capacity(84 + (tri_count as usize) * 50);
    let mut header = [b' '; 80];
    header[..header_text.len()].copy_from_slice(header_text);
    buf.extend_from_slice(&header);
    buf.extend_from_slice(&tri_count.to_le_bytes());
    for _ in 0..tri_count {
        // normal + 3 vertices, all zero — a degenerate but parsable triangle.
        for _ in 0..12 {
            buf.extend_from_slice(&0.0_f32.to_le_bytes());
        }
        buf.extend_from_slice(&0u16.to_le_bytes());
    }
    buf
}

#[test]
fn cadkey_2002_header_decodes_as_binary() {
    // `solid CADKey 2002` — observed in CADKey-exported STLs from the
    // early 2000s. Starts with `b"solid "` so the prefix sniff would
    // misfire; relies on the size cross-check to route to binary.
    let bytes = binary_with_header(b"solid CADKey 2002", 1);
    let scene = StlDecoder::new()
        .decode(&bytes)
        .expect("CADKey 2002 binary STL must decode");
    assert_eq!(scene.triangle_count(), 1);
}

#[test]
fn solidworks_binary_prefix_routes_to_binary() {
    // `solidWORKS` has no whitespace after `solid` — caught at the
    // prefix-validation stage, never reaches the size cross-check.
    let bytes = binary_with_header(b"solidWORKS export tool", 1);
    let scene = StlDecoder::new()
        .decode(&bytes)
        .expect("SolidWorks-prefixed binary STL must decode");
    assert_eq!(scene.triangle_count(), 1);
}

#[test]
fn autocad_export_with_leading_newline_decodes_as_ascii() {
    // Some AutoCAD pipelines emit ASCII STL with a stray leading
    // newline before `solid`. The detector strips leading whitespace
    // before sniffing.
    let s = "\n\nsolid acad\n  facet normal 0 0 1\n    outer loop\n      vertex 0 0 0\n      vertex 1 0 0\n      vertex 0 1 0\n    endloop\n  endfacet\nendsolid acad\n";
    let scene = StlDecoder::new()
        .decode(s.as_bytes())
        .expect("ASCII STL with leading whitespace must decode");
    assert_eq!(scene.triangle_count(), 1);
    assert_eq!(scene.meshes[0].name.as_deref(), Some("acad"));
}

#[test]
fn ascii_with_utf8_bom_decodes_as_ascii() {
    // Windows-side text editors sometimes save ASCII STL with a UTF-8
    // BOM prepended. The detector strips it before sniffing.
    let mut bytes = Vec::new();
    bytes.extend_from_slice(&[0xEF, 0xBB, 0xBF]);
    bytes.extend_from_slice(b"solid bom\n  facet normal 0 0 1\n    outer loop\n      vertex 0 0 0\n      vertex 1 0 0\n      vertex 0 1 0\n    endloop\n  endfacet\nendsolid bom\n");
    let scene = StlDecoder::new()
        .decode(&bytes)
        .expect("ASCII STL with UTF-8 BOM must decode");
    assert_eq!(scene.triangle_count(), 1);
}

#[test]
fn adversarial_binary_with_embedded_facet_token_in_header_routes_to_binary() {
    // Worst case: an 80-byte binary header that contains BOTH `solid `
    // at the start AND a `\n facet ` substring midway. Only the size
    // cross-check distinguishes this from ASCII.
    let mut header = [b' '; 80];
    header[..6].copy_from_slice(b"solid ");
    header[20..28].copy_from_slice(b"\n facet ");
    let bytes = {
        let mut buf = Vec::with_capacity(134);
        buf.extend_from_slice(&header);
        buf.extend_from_slice(&1u32.to_le_bytes());
        buf.extend_from_slice(&[0u8; 50]);
        buf
    };
    let scene = StlDecoder::new()
        .decode(&bytes)
        .expect("adversarial header binary must route to binary parser");
    assert_eq!(scene.triangle_count(), 1);
}

//! Format-detection tests — the well-known trap is binary STL files
//! whose 80-byte vendor header begins with the literal `solid` ASCII
//! string. Some Microsoft toolchains have written exactly that for
//! decades; if our sniffer trusts the prefix alone, the file gets
//! routed to the ASCII parser and decoding fails.

use oxideav_mesh3d::Mesh3DDecoder;
use oxideav_stl::StlDecoder;

fn synth_solid_prefixed_binary() -> Vec<u8> {
    // 80-byte header literally beginning with "solid by Microsoft".
    let header_text = b"solid by Microsoft";
    let mut header = [0u8; 80];
    header[..header_text.len()].copy_from_slice(header_text);
    let mut buf = Vec::new();
    buf.extend_from_slice(&header);
    // One triangle so the sniffer's `\nfacet` lookahead cannot succeed
    // accidentally — the binary payload past byte 80 is well-defined
    // numerics with zero printable-ASCII content beyond what's already
    // in the header.
    buf.extend_from_slice(&1u32.to_le_bytes());
    // normal
    for f in [0.0_f32, 0.0, 1.0] {
        buf.extend_from_slice(&f.to_le_bytes());
    }
    // v0, v1, v2
    for v in [[0.0_f32, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]] {
        for c in v {
            buf.extend_from_slice(&c.to_le_bytes());
        }
    }
    buf.extend_from_slice(&0u16.to_le_bytes());
    buf
}

#[test]
fn solid_prefixed_binary_decodes_as_binary_not_ascii() {
    let bytes = synth_solid_prefixed_binary();
    // First six bytes ARE `b"solid "` — the trap.
    assert_eq!(&bytes[..6], b"solid ");

    let scene = StlDecoder::new()
        .decode(&bytes)
        .expect("should decode as binary");
    assert_eq!(scene.triangle_count(), 1);
}

#[test]
fn canonical_ascii_decodes_as_ascii() {
    let s = "solid x\n  facet normal 0 0 1\n    outer loop\n      vertex 0 0 0\n      vertex 1 0 0\n      vertex 0 1 0\n    endloop\n  endfacet\nendsolid x\n";
    let scene = StlDecoder::new().decode(s.as_bytes()).unwrap();
    assert_eq!(scene.triangle_count(), 1);
    assert_eq!(scene.meshes[0].name.as_deref(), Some("x"));
}

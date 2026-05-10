//! End-to-end trace-tape verification.
//!
//! Compiled in only when `--features trace` is enabled. Pins the
//! per-thread trace path, runs a decode (and an encode), reads the
//! tape back, and asserts the JSONL content.

#![cfg(feature = "trace")]

use std::path::PathBuf;

use oxideav_mesh3d::{Mesh, Mesh3DDecoder, Mesh3DEncoder, Primitive, Scene3D, Topology};
use oxideav_stl::{trace, StlDecoder, StlEncoder};

fn tmp_path(stem: &str) -> PathBuf {
    let mut p = std::env::temp_dir();
    p.push(format!(
        "oxideav-stl-trace-{}-{}.jsonl",
        stem,
        std::process::id()
    ));
    p
}

fn read_lines(path: &std::path::Path) -> Vec<String> {
    let s = std::fs::read_to_string(path).expect("trace tape exists");
    s.lines().map(|l| l.to_string()).collect()
}

fn synth_minimal_binary() -> Vec<u8> {
    let mut buf = Vec::new();
    let mut header = [b' '; 80];
    header[..14].copy_from_slice(b"trace-fixture ");
    buf.extend_from_slice(&header);
    buf.extend_from_slice(&1u32.to_le_bytes());
    // normal
    for f in [0.0_f32, 0.0, 1.0] {
        buf.extend_from_slice(&f.to_le_bytes());
    }
    // v0..v2
    for v in [[0.0_f32, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]] {
        for c in v {
            buf.extend_from_slice(&c.to_le_bytes());
        }
    }
    buf.extend_from_slice(&0x1234_u16.to_le_bytes());
    buf
}

#[test]
fn binary_decode_emits_header_count_triangle_done() {
    let path = tmp_path("decode-binary");
    let _ = std::fs::remove_file(&path);
    trace::set_thread_trace_path(Some(path.clone()));

    let bytes = synth_minimal_binary();
    let scene = StlDecoder::new().decode(&bytes).unwrap();
    assert_eq!(scene.triangle_count(), 1);

    // Drop the tracer (going out of scope at end of decode flushes).
    let lines = read_lines(&path);
    assert_eq!(lines.len(), 4, "expected 4 events; got {:?}", lines);
    assert!(
        lines[0].contains(r#""kind":"header""#) && lines[0].contains(r#""format":"binary""#),
        "line 0: {}",
        lines[0]
    );
    assert!(lines[0].contains(r#""byte_len":134"#));
    assert!(lines[1].contains(r#""kind":"triangle_count""#) && lines[1].contains(r#""count":1"#));
    assert!(lines[2].contains(r#""kind":"triangle""#) && lines[2].contains(r#""index":0"#));
    assert!(lines[2].contains(r#""normal":[0,0,1]"#));
    assert!(lines[2].contains(r#""attribute_bytes":"3412""#));
    assert!(
        lines[3].contains(r#""kind":"done""#)
            && lines[3].contains(r#""source":"binary""#)
            && lines[3].contains(r#""triangles_emitted":1"#)
    );

    trace::set_thread_trace_path(None);
    let _ = std::fs::remove_file(&path);
}

#[test]
fn ascii_decode_emits_header_then_triangles() {
    let path = tmp_path("decode-ascii");
    let _ = std::fs::remove_file(&path);
    trace::set_thread_trace_path(Some(path.clone()));

    let s = "solid foo\n  facet normal 0 0 1\n    outer loop\n      vertex 0 0 0\n      vertex 1 0 0\n      vertex 0 1 0\n    endloop\n  endfacet\nendsolid foo\n";
    let scene = StlDecoder::new().decode(s.as_bytes()).unwrap();
    assert_eq!(scene.triangle_count(), 1);

    let lines = read_lines(&path);
    assert_eq!(lines.len(), 4);
    assert!(lines[0].contains(r#""format":"ascii""#) && lines[0].contains(r#""name":"foo""#));
    assert!(
        lines[1].contains(r#""kind":"triangle""#)
            && lines[1].contains(r#""index":0"#)
            && lines[1].contains(r#""normal":[0,0,1]"#)
    );
    // ASCII triangles do NOT carry attribute_bytes.
    assert!(!lines[1].contains("attribute_bytes"));
    assert!(lines[2].contains(r#""kind":"triangle_count""#) && lines[2].contains(r#""count":1"#));
    assert!(lines[3].contains(r#""kind":"done""#) && lines[3].contains(r#""source":"ascii""#));

    trace::set_thread_trace_path(None);
    let _ = std::fs::remove_file(&path);
}

#[test]
fn binary_encode_emits_full_event_sequence() {
    let path = tmp_path("encode-binary");
    let _ = std::fs::remove_file(&path);
    trace::set_thread_trace_path(Some(path.clone()));

    let mut s = Scene3D::new();
    s.add_mesh(Mesh {
        name: Some("t".into()),
        primitives: vec![Primitive {
            topology: Topology::Triangles,
            positions: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            normals: Some(vec![[0.0, 0.0, 1.0]; 3]),
            tangents: None,
            uvs: Vec::new(),
            colors: Vec::new(),
            joints: None,
            weights: None,
            indices: None,
            material: None,
            extras: std::collections::HashMap::new(),
        }],
    });
    let _ = StlEncoder::new_binary().encode(&s).unwrap();

    let lines = read_lines(&path);
    assert_eq!(lines.len(), 4, "tape: {:?}", lines);
    assert!(lines[0].contains(r#""kind":"header""#) && lines[0].contains(r#""format":"binary""#));
    assert!(lines[1].contains(r#""count":1"#));
    assert!(lines[2].contains(r#""kind":"triangle""#) && lines[2].contains(r#""index":0"#));
    assert!(lines[3].contains(r#""kind":"done""#));

    trace::set_thread_trace_path(None);
    let _ = std::fs::remove_file(&path);
}

#[test]
fn no_trace_path_means_no_tape() {
    // Sanity: with neither env var nor override set, the tracer is
    // None and decode runs cleanly. This test guards against a
    // regression where a missing override silently picked up the
    // process-global env var from a sibling test.
    trace::set_thread_trace_path(None);
    // Ensure env var is also unset — set to empty so the open_from_env
    // returns None.
    std::env::set_var("OXIDEAV_STL_TRACE_FILE", "");
    let bytes = synth_minimal_binary();
    let _ = StlDecoder::new().decode(&bytes).unwrap();
    std::env::remove_var("OXIDEAV_STL_TRACE_FILE");
}

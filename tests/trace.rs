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
            targets: Vec::new(),
            extras: std::collections::HashMap::new(),
        }],
        weights: Vec::new(),
    });
    let _ = StlEncoder::new_binary().encode(&s).unwrap();

    let lines = read_lines(&path);
    // Encoder tape has 5 events: header, triangle_count, triangle,
    // share_stats, done. (Decode tape stays at 4 — share_stats is
    // encoder-only because it needs `&Scene3D`.)
    assert_eq!(lines.len(), 5, "tape: {:?}", lines);
    assert!(lines[0].contains(r#""kind":"header""#) && lines[0].contains(r#""format":"binary""#));
    assert!(lines[1].contains(r#""count":1"#));
    assert!(lines[2].contains(r#""kind":"triangle""#) && lines[2].contains(r#""index":0"#));
    assert!(
        lines[3].contains(r#""kind":"share_stats""#)
            && lines[3].contains(r#""triangles":1"#)
            && lines[3].contains(r#""emitted_vertices":3"#)
            && lines[3].contains(r#""unique_vertices":3"#)
            && lines[3].contains(r#""share_factor":1"#)
            && lines[3].contains(r#""tolerance_eps":null"#)
    );
    assert!(lines[4].contains(r#""kind":"done""#));

    trace::set_thread_trace_path(None);
    let _ = std::fs::remove_file(&path);
}

#[test]
fn binary_encode_share_stats_reflects_indexed_cube_collapse() {
    // Indexed cube — 8 unique corners, 12 triangles, 36 emitted slots,
    // share_factor = 4.5. Verifies the share_stats event picks up the
    // bit-exact unique-vertex count (i.e. it really walks the index
    // buffer rather than positions.len()).
    use oxideav_mesh3d::{Indices, Mesh, Mesh3DEncoder, Primitive, Scene3D, Topology};

    let path = tmp_path("encode-binary-share-stats");
    let _ = std::fs::remove_file(&path);
    trace::set_thread_trace_path(Some(path.clone()));

    let positions: Vec<[f32; 3]> = vec![
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [1.0, 1.0, 1.0],
        [0.0, 1.0, 1.0],
    ];
    let indices: Vec<u32> = vec![
        0, 2, 1, 0, 3, 2, // bottom
        4, 5, 6, 4, 6, 7, // top
        0, 1, 5, 0, 5, 4, // front
        2, 3, 7, 2, 7, 6, // back
        1, 2, 6, 1, 6, 5, // right
        0, 4, 7, 0, 7, 3, // left
    ];
    let mut s = Scene3D::new();
    s.add_mesh(Mesh {
        name: Some("cube".into()),
        primitives: vec![Primitive {
            topology: Topology::Triangles,
            positions,
            normals: None,
            tangents: None,
            uvs: Vec::new(),
            colors: Vec::new(),
            joints: None,
            weights: None,
            indices: Some(Indices::U32(indices)),
            material: None,
            targets: Vec::new(),
            extras: std::collections::HashMap::new(),
        }],
        weights: Vec::new(),
    });
    let _ = StlEncoder::new_binary().encode(&s).unwrap();

    let lines = read_lines(&path);
    // 1 header + 1 triangle_count + 12 triangle + 1 share_stats + 1 done = 16
    assert_eq!(lines.len(), 16, "tape: {:?}", lines);
    let share = lines
        .iter()
        .find(|l| l.contains(r#""kind":"share_stats""#))
        .expect("share_stats event present");
    assert!(share.contains(r#""triangles":12"#), "share: {share}");
    assert!(share.contains(r#""emitted_vertices":36"#), "share: {share}");
    assert!(share.contains(r#""unique_vertices":8"#), "share: {share}");
    assert!(share.contains(r#""share_factor":4.5"#), "share: {share}");
    assert!(share.contains(r#""tolerance_eps":null"#), "share: {share}");

    trace::set_thread_trace_path(None);
    let _ = std::fs::remove_file(&path);
}

#[test]
fn ascii_encode_emits_share_stats_before_done() {
    // ASCII encode tape carries the same 5-event encoder sequence as
    // binary, with `tolerance_eps == null` and a bit-exact summary.
    use oxideav_mesh3d::{Mesh, Mesh3DEncoder, Primitive, Scene3D, Topology};

    let path = tmp_path("encode-ascii-share-stats");
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
            targets: Vec::new(),
            extras: std::collections::HashMap::new(),
        }],
        weights: Vec::new(),
    });
    let _ = StlEncoder::new_ascii().encode(&s).unwrap();

    let lines = read_lines(&path);
    // header + triangle + triangle_count + share_stats + done = 5
    assert_eq!(lines.len(), 5, "tape: {:?}", lines);
    // ASCII encoder emits triangle_count after the triangles (mirroring
    // the ASCII decoder), and share_stats sits between triangle_count
    // and done.
    assert!(lines[0].contains(r#""kind":"header""#) && lines[0].contains(r#""format":"ascii""#));
    assert!(lines[1].contains(r#""kind":"triangle""#));
    assert!(lines[2].contains(r#""kind":"triangle_count""#));
    assert!(
        lines[3].contains(r#""kind":"share_stats""#)
            && lines[3].contains(r#""triangles":1"#)
            && lines[3].contains(r#""emitted_vertices":3"#)
            && lines[3].contains(r#""unique_vertices":3"#)
            && lines[3].contains(r#""tolerance_eps":null"#)
    );
    assert!(lines[4].contains(r#""kind":"done""#) && lines[4].contains(r#""source":"ascii""#));

    trace::set_thread_trace_path(None);
    let _ = std::fs::remove_file(&path);
}

#[test]
fn decode_tape_does_not_emit_share_stats() {
    // share_stats is encoder-only — the decoder has no `&Scene3D` at
    // emit time and the event vocabulary tolerates its absence on
    // decode tapes by design.
    let path = tmp_path("decode-no-share-stats");
    let _ = std::fs::remove_file(&path);
    trace::set_thread_trace_path(Some(path.clone()));

    let bytes = synth_minimal_binary();
    let _ = StlDecoder::new().decode(&bytes).unwrap();

    let lines = read_lines(&path);
    assert!(
        !lines.iter().any(|l| l.contains(r#""kind":"share_stats""#)),
        "decode tape should not emit share_stats: {:?}",
        lines
    );

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

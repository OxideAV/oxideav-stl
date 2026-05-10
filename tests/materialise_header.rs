//! Materialise binary-header `COLOR=R G B A` / `MATERIAL=Ar Ag Ab Sa
//! Dr Dg Db Sd Sr Sg Sb Ss` round-trip.
//!
//! Materialise Magics writes per-object default colour and material
//! tokens into the 80-byte STL vendor-header slot. We surface those
//! tokens on `Primitive::extras["stl:default_color"]` (4-element u8
//! array) and `Primitive::extras["stl:default_material"]` (12-element
//! u8 array), and re-emit them on encode so the round-trip is faithful.
//!
//! These tests synthesise a one-triangle binary STL whose header
//! contains the textual payload, decode it, verify the extras keys are
//! populated with the expected values, then re-encode and confirm the
//! tokens survive byte-identically inside the rebuilt header.

use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};
use oxideav_stl::{StlDecoder, StlEncoder};
use serde_json::Value;

/// Build a one-triangle binary STL with the supplied 80-byte header.
fn synth_with_header(header: [u8; 80]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(84 + 50);
    buf.extend_from_slice(&header);
    buf.extend_from_slice(&1u32.to_le_bytes());
    // 1 triangle: zero normal, three zero vertices, zero attribute slot.
    for _ in 0..12 {
        buf.extend_from_slice(&0.0_f32.to_le_bytes());
    }
    buf.extend_from_slice(&0u16.to_le_bytes());
    buf
}

fn header_with(text: &[u8]) -> [u8; 80] {
    assert!(
        text.len() <= 80,
        "test fixture text overflows the 80-byte header slot"
    );
    let mut h = [0u8; 80];
    h[..text.len()].copy_from_slice(text);
    h
}

#[test]
fn materialise_default_color_is_lifted_to_extras() {
    let header = header_with(b"COLOR=200 100 50 25\n");
    let bytes = synth_with_header(header);
    let scene = StlDecoder::new().decode(&bytes).unwrap();
    let prim = &scene.meshes[0].primitives[0];
    let arr = prim
        .extras
        .get("stl:default_color")
        .and_then(|v| v.as_array())
        .expect("default colour should be lifted onto extras");
    let vals: Vec<u8> = arr.iter().map(|v| v.as_u64().unwrap() as u8).collect();
    assert_eq!(vals, vec![200, 100, 50, 25]);
}

#[test]
fn materialise_default_material_is_lifted_to_extras() {
    let header = header_with(b"MATERIAL=10 20 30 40 50 60 70 80 90 100 110 120\n");
    let bytes = synth_with_header(header);
    let scene = StlDecoder::new().decode(&bytes).unwrap();
    let prim = &scene.meshes[0].primitives[0];
    let arr = prim
        .extras
        .get("stl:default_material")
        .and_then(|v| v.as_array())
        .expect("default material should be lifted onto extras");
    let vals: Vec<u8> = arr.iter().map(|v| v.as_u64().unwrap() as u8).collect();
    assert_eq!(
        vals,
        vec![10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120]
    );
}

#[test]
fn materialise_color_and_material_round_trip_through_re_encode() {
    let header = header_with(b"COLOR=250 200 150 100\nMATERIAL=1 2 3 4 5 6 7 8 9 10 11 12\n");
    let bytes = synth_with_header(header);
    let scene = StlDecoder::new().decode(&bytes).unwrap();

    // Re-encode and confirm the rebuilt header still parses to the
    // same color + material payload.
    let reencoded = StlEncoder::new_binary().encode(&scene).unwrap();
    let scene2 = StlDecoder::new().decode(&reencoded).unwrap();
    let prim2 = &scene2.meshes[0].primitives[0];
    let color_arr = prim2
        .extras
        .get("stl:default_color")
        .and_then(|v| v.as_array())
        .expect("colour preserved across re-encode");
    let mat_arr = prim2
        .extras
        .get("stl:default_material")
        .and_then(|v| v.as_array())
        .expect("material preserved across re-encode");
    assert_eq!(
        color_arr
            .iter()
            .map(|v| v.as_u64().unwrap())
            .collect::<Vec<_>>(),
        vec![250, 200, 150, 100]
    );
    assert_eq!(
        mat_arr
            .iter()
            .map(|v| v.as_u64().unwrap())
            .collect::<Vec<_>>(),
        vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
    );
}

#[test]
fn header_without_materialise_tokens_yields_no_extras_keys() {
    // A vanilla writer-signature header should not synthesise spurious
    // colour/material entries.
    let header = header_with(b"vanilla writer signature ");
    let bytes = synth_with_header(header);
    let scene = StlDecoder::new().decode(&bytes).unwrap();
    let prim = &scene.meshes[0].primitives[0];
    assert!(!prim.extras.contains_key("stl:default_color"));
    assert!(!prim.extras.contains_key("stl:default_material"));
}

#[test]
fn malformed_color_payload_is_silently_skipped() {
    // 3 tokens instead of 4 → not a valid `COLOR=` line; decoder
    // should NOT populate the key (and must not panic).
    let header = header_with(b"COLOR=1 2 3\n");
    let bytes = synth_with_header(header);
    let scene = StlDecoder::new().decode(&bytes).unwrap();
    let prim = &scene.meshes[0].primitives[0];
    assert!(!prim.extras.contains_key("stl:default_color"));
}

#[test]
fn malformed_extras_value_falls_back_to_default_header() {
    // If a downstream user puts a wrong-shape value into the extras
    // map (e.g. 3-element colour), the encoder should fall back to
    // the writer-signature default header rather than producing
    // garbage bytes.
    use std::collections::HashMap;

    use oxideav_mesh3d::{Axis, Mesh, Node, Primitive, Scene3D, Topology, Unit};

    let mut extras = HashMap::new();
    extras.insert(
        "stl:default_color".to_string(),
        Value::Array(vec![Value::from(1u8), Value::from(2u8), Value::from(3u8)]), // length 3 != 4
    );
    let mesh = Mesh {
        name: None,
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
            extras,
        }],
        weights: Vec::new(),
    };
    let mut scene = Scene3D::new();
    scene.up_axis = Axis::PosZ;
    scene.unit = Unit::Millimetres;
    let mid = scene.add_mesh(mesh);
    let mut node = Node::new();
    node.mesh = Some(mid);
    let nid = scene.add_node(node);
    scene.add_root(nid);

    let bytes = StlEncoder::new_binary().encode(&scene).unwrap();
    // The first byte of a fallback header is `'o'` (writer signature
    // begins with `b"oxideav-stl"`); a successfully-built Materialise
    // header would begin with `'C'`. Either way it must NOT begin
    // with `b"solid "` to avoid the ASCII sniffer trap.
    assert_ne!(&bytes[..6], b"solid ");
    assert_eq!(bytes[0], b'o');
}

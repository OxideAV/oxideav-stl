//! ASCII pretty-printer with configurable float precision.
//!
//! By default the encoder uses Rust's round-trip-safe `{}` formatter
//! for f32 values, which yields strings like `0.1` for `0.1f32` and
//! `1.4142135` for `(2.0_f32).sqrt()`. Real-world consumers — code
//! review, schema diffs, human-readable inspection — sometimes prefer
//! a fixed-decimal width (`{:.6}` style). [`StlEncoder::with_float_precision`]
//! exposes that knob.

use oxideav_mesh3d::{
    Axis, Mesh, Mesh3DDecoder, Mesh3DEncoder, Node, Primitive, Scene3D, Topology, Unit,
};
use oxideav_stl::{StlDecoder, StlEncoder};

fn synthesise_one_facet_scene() -> Scene3D {
    let mut s = Scene3D::new();
    s.up_axis = Axis::PosZ;
    s.unit = Unit::Millimetres;
    let mesh = Mesh {
        name: Some("t".into()),
        primitives: vec![Primitive {
            topology: Topology::Triangles,
            positions: vec![
                [0.123_456_78_f32, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
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
    };
    let mid = s.add_mesh(mesh);
    let mut node = Node::new();
    node.mesh = Some(mid);
    let nid = s.add_node(node);
    s.add_root(nid);
    s
}

#[test]
fn default_precision_emits_round_trip_safe_floats() {
    // No precision override → same as historical `{}` formatting.
    let scene = synthesise_one_facet_scene();
    let bytes = StlEncoder::new_ascii().encode(&scene).unwrap();
    let txt = std::str::from_utf8(&bytes).unwrap();
    // Round-trip safe: round-trips back through the parser at f32
    // identity (we don't assert the exact string here, just that the
    // value re-parses to bit-identical f32 values).
    let scene2 = StlDecoder::new().decode(&bytes).unwrap();
    let p = &scene2.meshes[0].primitives[0];
    assert!(
        (p.positions[0][0] - scene.meshes[0].primitives[0].positions[0][0]).abs() < 1e-7,
        "default formatter must be round-trip safe; got {txt}"
    );
}

#[test]
fn precision_six_emits_fixed_decimal() {
    let scene = synthesise_one_facet_scene();
    let bytes = StlEncoder::new_ascii()
        .with_float_precision(Some(6))
        .encode(&scene)
        .unwrap();
    let txt = std::str::from_utf8(&bytes).unwrap();
    // 0.12345678 → "0.123457" at 6 decimals.
    assert!(
        txt.contains("vertex 0.123457 0.000000 0.000000"),
        "precision-6 vertex line missing; got:\n{txt}"
    );
    // Normal also rounded: 0 → "0.000000".
    assert!(
        txt.contains("facet normal 0.000000 0.000000 1.000000"),
        "precision-6 facet normal line missing; got:\n{txt}"
    );
}

#[test]
fn precision_zero_emits_integer_like_floats() {
    let scene = synthesise_one_facet_scene();
    let bytes = StlEncoder::new_ascii()
        .with_float_precision(Some(0))
        .encode(&scene)
        .unwrap();
    let txt = std::str::from_utf8(&bytes).unwrap();
    // 0.123 rounds to 0; 1.0 stays 1.
    assert!(
        txt.contains("vertex 0 0 0"),
        "precision-0 should round 0.123 to 0; got:\n{txt}"
    );
    assert!(
        txt.contains("vertex 1 0 0"),
        "precision-0 should keep 1.0 as 1; got:\n{txt}"
    );
}

#[test]
fn precision_revert_to_default_via_none() {
    let scene = synthesise_one_facet_scene();
    let mut with = StlEncoder::new_ascii()
        .with_float_precision(Some(6))
        .with_float_precision(None);
    let bytes = with.encode(&scene).unwrap();
    let txt = std::str::from_utf8(&bytes).unwrap();
    // After reverting to None, no fixed-decimal `0.000000` should appear.
    assert!(
        !txt.contains("0.000000"),
        "reverting precision to None should drop fixed decimals; got:\n{txt}"
    );
}

#[test]
fn binary_format_ignores_float_precision() {
    let scene = synthesise_one_facet_scene();
    let mut a = StlEncoder::new_binary();
    let bytes_default = a.encode(&scene).unwrap();
    let mut b = StlEncoder::new_binary().with_float_precision(Some(2));
    let bytes_with_precision = b.encode(&scene).unwrap();
    assert_eq!(
        bytes_default[84..],
        bytes_with_precision[84..],
        "binary triangle records should be byte-identical regardless of float-precision knob"
    );
}

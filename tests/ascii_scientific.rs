//! Spec-style scientific ASCII formatter — emits the `1.23456E+789`
//! flavour the 1989 spec uses as its worked example.
//!
//! The default ASCII formatter uses Rust's round-trip-safe `{}`. The
//! spec text (Marshall Burns' transcription of §6.5.2) says: "The
//! numerical data in the **facet normal** and **vertex** lines are
//! single precision floats, for example, 1.23456E+789." Some strict
//! 1989-era StL toolchains key off the literal `E+nnn` form; the new
//! `with_spec_scientific(precision)` knob produces it verbatim.

use oxideav_mesh3d::{
    Axis, Mesh, Mesh3DDecoder, Mesh3DEncoder, Node, Primitive, Scene3D, Topology, Unit,
};
use oxideav_stl::{AsciiNumberFormat, StlDecoder, StlEncoder};

fn synthesise_one_facet_scene() -> Scene3D {
    let mut s = Scene3D::new();
    s.up_axis = Axis::PosZ;
    s.unit = Unit::Millimetres;
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = vec![
        [1.234_567_8_f32, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ];
    prim.normals = Some(vec![[0.0, 0.0, 1.0]; 3]);
    let mesh = Mesh::new(Some("t".to_string())).with_primitive(prim);
    let mid = s.add_mesh(mesh);
    let mut node = Node::new();
    node.mesh = Some(mid);
    let nid = s.add_node(node);
    s.add_root(nid);
    s
}

#[test]
fn spec_scientific_emits_explicit_exponent_sign() {
    let scene = synthesise_one_facet_scene();
    let bytes = StlEncoder::new_ascii()
        .with_spec_scientific(Some(5))
        .encode(&scene)
        .unwrap();
    let txt = std::str::from_utf8(&bytes).unwrap();
    // Mantissa precision 5 → e.g. "1.23457E0" with explicit sign.
    // 1.2345678 rounds to 1.23457 with 5-digit mantissa.
    assert!(
        txt.contains("vertex 1.23457E+0 0.00000E+0 0.00000E+0"),
        "expected spec-style E+0 form for vertex line; got:\n{txt}"
    );
    // The facet normal also goes through the formatter.
    assert!(
        txt.contains("facet normal 0.00000E+0 0.00000E+0 1.00000E+0"),
        "facet normal line missing or wrongly formatted; got:\n{txt}"
    );
}

#[test]
fn spec_scientific_negative_exponent_uses_minus_sign() {
    // A small value: 0.001 → 1.000E-3 at 3-digit mantissa precision.
    let mut s = Scene3D::new();
    s.up_axis = Axis::PosZ;
    s.unit = Unit::Millimetres;
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = vec![[0.001_f32, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    prim.normals = Some(vec![[0.0, 0.0, 1.0]; 3]);
    let mid = s.add_mesh(Mesh::new(Some("t".to_string())).with_primitive(prim));
    let mut node = Node::new();
    node.mesh = Some(mid);
    let nid = s.add_node(node);
    s.add_root(nid);

    let bytes = StlEncoder::new_ascii()
        .with_spec_scientific(Some(3))
        .encode(&s)
        .unwrap();
    let txt = std::str::from_utf8(&bytes).unwrap();
    assert!(
        txt.contains("1.000E-3"),
        "expected 1.000E-3 for 0.001 at precision-3; got:\n{txt}"
    );
}

#[test]
fn spec_scientific_round_trips_through_parser() {
    // Output must still re-parse to the same f32 within precision.
    let scene = synthesise_one_facet_scene();
    let bytes = StlEncoder::new_ascii()
        .with_spec_scientific(Some(7))
        .encode(&scene)
        .unwrap();
    let scene2 = StlDecoder::new().decode(&bytes).expect("re-parse ok");
    let p_in = scene.meshes[0].primitives[0].positions[0][0];
    let p_out = scene2.meshes[0].primitives[0].positions[0][0];
    // 7 digits of mantissa precision → tight tolerance.
    assert!(
        (p_in - p_out).abs() < 1e-5,
        "round-trip mismatch: in={p_in} out={p_out}"
    );
}

#[test]
fn spec_scientific_revert_to_default_via_none() {
    let scene = synthesise_one_facet_scene();
    // Configure scientific then revert; output should look like
    // default round-trip.
    let bytes = StlEncoder::new_ascii()
        .with_spec_scientific(Some(5))
        .with_spec_scientific(None)
        .encode(&scene)
        .unwrap();
    let txt = std::str::from_utf8(&bytes).unwrap();
    // No `E+` or `E-` substrings should be present in the round-trip
    // form for the values we used.
    assert!(
        !txt.contains("E+") && !txt.contains("E-"),
        "reverting scientific to None should suppress E+/E-; got:\n{txt}"
    );
}

#[test]
fn spec_scientific_binary_format_unchanged() {
    let scene = synthesise_one_facet_scene();
    let bytes_default = StlEncoder::new_binary().encode(&scene).unwrap();
    let bytes_scientific = StlEncoder::new_binary()
        .with_spec_scientific(Some(2))
        .encode(&scene)
        .unwrap();
    assert_eq!(
        bytes_default[84..],
        bytes_scientific[84..],
        "binary triangle records must be byte-identical regardless of ASCII knob"
    );
}

#[test]
fn with_number_format_setter_matches_helper() {
    let scene = synthesise_one_facet_scene();
    let via_helper = StlEncoder::new_ascii()
        .with_spec_scientific(Some(4))
        .encode(&scene)
        .unwrap();
    let via_setter = StlEncoder::new_ascii()
        .with_number_format(AsciiNumberFormat::SpecScientific { precision: 4 })
        .encode(&scene)
        .unwrap();
    assert_eq!(via_helper, via_setter);
}

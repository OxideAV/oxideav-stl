//! Encoding non-`Triangles` topologies must yield
//! `Error::Unsupported(...)`.

use std::collections::HashMap;

use oxideav_mesh3d::{Error, Mesh, Mesh3DEncoder, Primitive, Scene3D, Topology};
use oxideav_stl::StlEncoder;

fn make_lines_scene() -> Scene3D {
    let mut s = Scene3D::new();
    let prim = Primitive {
        topology: Topology::Lines,
        positions: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
        normals: None,
        tangents: None,
        uvs: Vec::new(),
        colors: Vec::new(),
        joints: None,
        weights: None,
        indices: None,
        material: None,
        extras: HashMap::new(),
    };
    s.add_mesh(Mesh {
        name: None,
        primitives: vec![prim],
    });
    s
}

#[test]
fn binary_encoder_rejects_lines_topology() {
    let scene = make_lines_scene();
    let err = StlEncoder::new_binary().encode(&scene).unwrap_err();
    match err {
        Error::Unsupported(msg) => {
            assert!(
                msg.contains("Triangles"),
                "expected diagnostic to mention Triangles, got: {msg}"
            );
            assert!(
                msg.contains("Lines"),
                "expected diagnostic to mention Lines, got: {msg}"
            );
        }
        other => panic!("expected Error::Unsupported, got {other:?}"),
    }
}

#[test]
fn ascii_encoder_rejects_lines_topology() {
    let scene = make_lines_scene();
    let err = StlEncoder::new_ascii().encode(&scene).unwrap_err();
    matches!(err, Error::Unsupported(_));
}

#[test]
fn encoder_rejects_points_topology() {
    let mut s = Scene3D::new();
    let prim = Primitive {
        topology: Topology::Points,
        positions: vec![[0.0, 0.0, 0.0]],
        normals: None,
        tangents: None,
        uvs: Vec::new(),
        colors: Vec::new(),
        joints: None,
        weights: None,
        indices: None,
        material: None,
        extras: HashMap::new(),
    };
    s.add_mesh(Mesh {
        name: None,
        primitives: vec![prim],
    });
    let err = StlEncoder::new_binary().encode(&s).unwrap_err();
    matches!(err, Error::Unsupported(_));
}

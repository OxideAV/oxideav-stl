//! Encoder robustness: a caller-built scene whose index buffer carries
//! entries past the position slice (or whose positions are shorter than
//! the index buffer implies) must be **refused** with
//! `Error::InvalidData`, not panic with an out-of-bounds index.
//!
//! STL has no vertex sharing, so the *decode* path never builds an
//! index buffer — these scenes can only arise from the direct-encode
//! API on a caller-constructed `Scene3D` (e.g. a glTF / OBJ scene
//! handed straight to the STL encoder). STL defines no meaning for a
//! dangling index, so the correct response is a typed error rather than
//! a panic. Regression test for the out-of-bounds panic the `encode`
//! fuzz target surfaced at `binary.rs` / `ascii.rs` (resolved vertex
//! index used to index `prim.positions` without a bounds check).

use oxideav_mesh3d::{
    Axis, Error, Indices, Mesh, Mesh3DEncoder, Node, Primitive, Scene3D, Topology, Unit,
};
use oxideav_stl::StlEncoder;

/// Build a single-`Triangles`-primitive scene with the supplied
/// positions and index buffer.
fn scene_with_indices(positions: Vec<[f32; 3]>, indices: Indices) -> Scene3D {
    let mut s = Scene3D::new();
    s.up_axis = Axis::PosZ;
    s.unit = Unit::Millimetres;
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = positions;
    prim.indices = Some(indices);
    let mesh = Mesh::new(Some("t".to_string())).with_primitive(prim);
    let mid = s.add_mesh(mesh);
    let mut node = Node::new();
    node.mesh = Some(mid);
    let nid = s.add_node(node);
    s.add_root(nid);
    s
}

#[test]
fn binary_encode_rejects_u32_index_past_positions() {
    // One face, index buffer points at vertex 168 but there are zero
    // positions — the exact shape the fuzzer minimised to.
    let scene = scene_with_indices(Vec::new(), Indices::U32(vec![168, 169, 170]));
    let err = StlEncoder::new_binary().encode(&scene).unwrap_err();
    assert!(
        matches!(err, Error::InvalidData(_)),
        "expected InvalidData, got {err:?}"
    );
}

#[test]
fn ascii_encode_rejects_u32_index_past_positions() {
    let scene = scene_with_indices(Vec::new(), Indices::U32(vec![168, 169, 170]));
    let err = StlEncoder::new_ascii().encode(&scene).unwrap_err();
    assert!(
        matches!(err, Error::InvalidData(_)),
        "expected InvalidData, got {err:?}"
    );
}

#[test]
fn binary_encode_rejects_u16_index_past_positions() {
    // Three real positions but the index buffer references vertex 9.
    let positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    let scene = scene_with_indices(positions, Indices::U16(vec![0, 1, 9]));
    let err = StlEncoder::new_binary().encode(&scene).unwrap_err();
    assert!(
        matches!(err, Error::InvalidData(_)),
        "expected InvalidData, got {err:?}"
    );
}

#[test]
fn ascii_encode_rejects_u16_index_past_positions() {
    let positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    let scene = scene_with_indices(positions, Indices::U16(vec![0, 1, 9]));
    let err = StlEncoder::new_ascii().encode(&scene).unwrap_err();
    assert!(
        matches!(err, Error::InvalidData(_)),
        "expected InvalidData, got {err:?}"
    );
}

#[test]
fn binary_encode_accepts_in_range_index_buffer() {
    // A well-formed indexed triangle still encodes — the bounds check
    // only fires on genuinely out-of-range entries.
    let positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    let scene = scene_with_indices(positions, Indices::U16(vec![0, 1, 2]));
    let bytes = StlEncoder::new_binary().encode(&scene).unwrap();
    // 80-byte header + u32 count + one 50-byte record.
    assert_eq!(bytes.len(), 80 + 4 + 50);
    assert_eq!(
        u32::from_le_bytes([bytes[80], bytes[81], bytes[82], bytes[83]]),
        1,
        "one triangle encoded"
    );
}

#[test]
fn ascii_encode_accepts_in_range_index_buffer() {
    let positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    let scene = scene_with_indices(positions, Indices::U32(vec![0, 1, 2]));
    let text = String::from_utf8(StlEncoder::new_ascii().encode(&scene).unwrap()).unwrap();
    assert!(text.contains("facet normal"), "emitted a facet");
    assert_eq!(text.matches("vertex").count(), 3, "three vertex lines");
}

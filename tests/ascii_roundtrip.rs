//! ASCII STL → Scene3D → ASCII round-trip.
//!
//! Loads a hand-written 12-triangle cube ASCII STL, parses it,
//! reserialises with the ASCII encoder, then verifies that the
//! triangle count, vertex positions, and per-face normal vectors
//! survive the round trip. We don't compare bytes literally because
//! float formatting may change punctuation (`0` vs `0.0`); instead we
//! re-parse the encoded output and compare the typed model.

use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};
use oxideav_stl::{StlDecoder, StlEncoder};

const CUBE_ASCII: &str = "\
solid cube
  facet normal 0 0 -1
    outer loop
      vertex 0 0 0
      vertex 1 1 0
      vertex 1 0 0
    endloop
  endfacet
  facet normal 0 0 -1
    outer loop
      vertex 0 0 0
      vertex 0 1 0
      vertex 1 1 0
    endloop
  endfacet
  facet normal 0 0 1
    outer loop
      vertex 0 0 1
      vertex 1 0 1
      vertex 1 1 1
    endloop
  endfacet
  facet normal 0 0 1
    outer loop
      vertex 0 0 1
      vertex 1 1 1
      vertex 0 1 1
    endloop
  endfacet
  facet normal 0 -1 0
    outer loop
      vertex 0 0 0
      vertex 1 0 0
      vertex 1 0 1
    endloop
  endfacet
  facet normal 0 -1 0
    outer loop
      vertex 0 0 0
      vertex 1 0 1
      vertex 0 0 1
    endloop
  endfacet
  facet normal 0 1 0
    outer loop
      vertex 0 1 0
      vertex 1 1 1
      vertex 1 1 0
    endloop
  endfacet
  facet normal 0 1 0
    outer loop
      vertex 0 1 0
      vertex 0 1 1
      vertex 1 1 1
    endloop
  endfacet
  facet normal -1 0 0
    outer loop
      vertex 0 0 0
      vertex 0 0 1
      vertex 0 1 1
    endloop
  endfacet
  facet normal -1 0 0
    outer loop
      vertex 0 0 0
      vertex 0 1 1
      vertex 0 1 0
    endloop
  endfacet
  facet normal 1 0 0
    outer loop
      vertex 1 0 0
      vertex 1 1 0
      vertex 1 1 1
    endloop
  endfacet
  facet normal 1 0 0
    outer loop
      vertex 1 0 0
      vertex 1 1 1
      vertex 1 0 1
    endloop
  endfacet
endsolid cube
";

#[test]
fn ascii_cube_roundtrip_preserves_triangles() {
    let scene = StlDecoder::new().decode(CUBE_ASCII.as_bytes()).unwrap();
    assert_eq!(scene.meshes.len(), 1);
    let mesh = &scene.meshes[0];
    assert_eq!(mesh.name.as_deref(), Some("cube"));
    let prim = &mesh.primitives[0];
    assert_eq!(prim.positions.len(), 36);
    assert_eq!(prim.normals.as_ref().unwrap().len(), 36);
    assert_eq!(scene.triangle_count(), 12);

    let encoded = StlEncoder::new_ascii().encode(&scene).unwrap();
    // Re-parse the encoded bytes and compare typed equivalents.
    let scene2 = StlDecoder::new().decode(&encoded).unwrap();
    let prim2 = &scene2.meshes[0].primitives[0];

    assert_eq!(prim.positions.len(), prim2.positions.len());
    for (a, b) in prim.positions.iter().zip(prim2.positions.iter()) {
        for axis in 0..3 {
            assert!((a[axis] - b[axis]).abs() < 1e-6, "vertex axis mismatch");
        }
    }
    let na = prim.normals.as_ref().unwrap();
    let nb = prim2.normals.as_ref().unwrap();
    for (a, b) in na.iter().zip(nb.iter()) {
        for axis in 0..3 {
            assert!((a[axis] - b[axis]).abs() < 1e-6, "normal axis mismatch");
        }
    }
}

#[test]
fn ascii_decoded_scene_has_z_up_and_millimetres() {
    let scene = StlDecoder::new().decode(CUBE_ASCII.as_bytes()).unwrap();
    assert_eq!(scene.up_axis, oxideav_mesh3d::Axis::PosZ);
    assert_eq!(scene.unit, oxideav_mesh3d::Unit::Millimetres);
}

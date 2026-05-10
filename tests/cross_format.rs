//! ASCII STL → Scene3D → binary STL → Scene3D structural-equality test.

use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};
use oxideav_stl::{StlDecoder, StlEncoder};

const TET: &str = "\
solid tet
  facet normal 0 0 -1
    outer loop
      vertex 0 0 0
      vertex 1 0 0
      vertex 0 1 0
    endloop
  endfacet
  facet normal -1 -1 1
    outer loop
      vertex 0 0 0
      vertex 0 1 0
      vertex 0 0 1
    endloop
  endfacet
  facet normal 1 -1 1
    outer loop
      vertex 0 0 0
      vertex 0 0 1
      vertex 1 0 0
    endloop
  endfacet
  facet normal 1 1 1
    outer loop
      vertex 1 0 0
      vertex 0 0 1
      vertex 0 1 0
    endloop
  endfacet
endsolid tet
";

#[test]
fn ascii_to_binary_to_scene_preserves_structure() {
    let scene_a = StlDecoder::new().decode(TET.as_bytes()).unwrap();
    assert_eq!(scene_a.triangle_count(), 4);

    let bin = StlEncoder::new_binary().encode(&scene_a).unwrap();
    let scene_b = StlDecoder::new().decode(&bin).unwrap();

    assert_eq!(scene_b.triangle_count(), 4);
    let pa = &scene_a.meshes[0].primitives[0];
    let pb = &scene_b.meshes[0].primitives[0];
    assert_eq!(pa.positions.len(), pb.positions.len());
    for (a, b) in pa.positions.iter().zip(pb.positions.iter()) {
        for axis in 0..3 {
            assert!((a[axis] - b[axis]).abs() < 1e-6);
        }
    }
}

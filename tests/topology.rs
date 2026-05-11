//! Integration tests for the topology utilities — exercise the
//! shell-finder, the Euler-characteristic helper, and the weld pass
//! through the decoder so the same `Scene3D` shape production
//! callers see is what the topology API consumes.

use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};
use oxideav_stl::{repair_weld_vertices, shells, StlDecoder, StlEncoder};

/// Build a binary STL byte stream for a closed unit cube — 12
/// triangles, 36 emitted vertices, every edge shared by exactly two
/// triangles when bit-exact compared. Mirrors the helper in
/// `tests/validate.rs` but isolated here so the topology test file
/// is self-contained.
fn unit_cube_binary_bytes() -> Vec<u8> {
    let mut buf = Vec::new();
    let mut header = [0u8; 80];
    header[..7].copy_from_slice(b"oxideav");
    buf.extend_from_slice(&header);
    buf.extend_from_slice(&12u32.to_le_bytes());

    let push_tri = |buf: &mut Vec<u8>, n: [f32; 3], v0: [f32; 3], v1: [f32; 3], v2: [f32; 3]| {
        for c in n {
            buf.extend_from_slice(&c.to_le_bytes());
        }
        for c in v0 {
            buf.extend_from_slice(&c.to_le_bytes());
        }
        for c in v1 {
            buf.extend_from_slice(&c.to_le_bytes());
        }
        for c in v2 {
            buf.extend_from_slice(&c.to_le_bytes());
        }
        buf.extend_from_slice(&0u16.to_le_bytes());
    };

    // -Z (bottom)
    push_tri(
        &mut buf,
        [0.0, 0.0, -1.0],
        [0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
    );
    push_tri(
        &mut buf,
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
        [1.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
    );
    // +Z (top)
    push_tri(
        &mut buf,
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [1.0, 1.0, 1.0],
    );
    push_tri(
        &mut buf,
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
        [1.0, 1.0, 1.0],
        [0.0, 1.0, 1.0],
    );
    // -Y (front)
    push_tri(
        &mut buf,
        [0.0, -1.0, 0.0],
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 0.0, 1.0],
    );
    push_tri(
        &mut buf,
        [0.0, -1.0, 0.0],
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
    );
    // +Y (back)
    push_tri(
        &mut buf,
        [0.0, 1.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 1.0, 1.0],
    );
    push_tri(
        &mut buf,
        [0.0, 1.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 1.0],
        [1.0, 1.0, 1.0],
    );
    // +X (right)
    push_tri(
        &mut buf,
        [1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [1.0, 1.0, 1.0],
    );
    push_tri(
        &mut buf,
        [1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 1.0],
        [1.0, 0.0, 1.0],
    );
    // -X (left)
    push_tri(
        &mut buf,
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 1.0],
    );
    push_tri(
        &mut buf,
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0],
        [0.0, 1.0, 1.0],
        [0.0, 1.0, 0.0],
    );
    buf
}

#[test]
fn decoded_cube_is_one_closed_shell_with_chi_two() {
    let bytes = unit_cube_binary_bytes();
    let scene = StlDecoder::new().decode(&bytes).unwrap();
    let s = shells(&scene);
    assert_eq!(s.len(), 1);
    assert_eq!(s[0].faces, 12);
    assert_eq!(s[0].vertices, 8);
    assert_eq!(s[0].edges, 18);
    assert_eq!(s[0].euler_characteristic(), 2);
    assert!(s[0].is_closed_manifold());
    assert_eq!(s[0].genus(), Some(0));
}

#[test]
fn weld_decoded_cube_yields_eight_canonical_corners() {
    let bytes = unit_cube_binary_bytes();
    let mut scene = StlDecoder::new().decode(&bytes).unwrap();
    let report = repair_weld_vertices(&mut scene);
    assert_eq!(report.triangles_inspected, 12);
    // 36 emitted slots → 8 canonical corners.
    assert_eq!(report.slots_collapsed, 28);
    // The decoded scene starts with positions.len() == 36 (decoder
    // expands per-face vertices); after welding, it's 8.
    assert_eq!(report.positions_collapsed, 28);
    assert_eq!(report.degenerate_triangles, 0);
    let prim = &scene.meshes[0].primitives[0];
    assert_eq!(prim.positions.len(), 8);
    match &prim.indices {
        Some(oxideav_mesh3d::Indices::U32(idx)) => assert_eq!(idx.len(), 36),
        _ => panic!("indices must be U32 after weld"),
    }
}

#[test]
fn welded_scene_re_encodes_to_the_same_36_triangle_record_count() {
    // After welding, the encoder still emits 12 triangles → 36
    // vertex slots in the binary STL output. The byte stream length
    // is the canonical "encoder doesn't care about index buffers"
    // invariant.
    let bytes = unit_cube_binary_bytes();
    let mut scene = StlDecoder::new().decode(&bytes).unwrap();
    repair_weld_vertices(&mut scene);
    let out = StlEncoder::new_binary().encode(&scene).unwrap();
    // 80 header + 4 count + 12 * 50 = 684 bytes.
    assert_eq!(out.len(), 684);
}

#[test]
fn two_disjoint_cubes_in_ascii_decode_to_two_shells() {
    // Multi-`solid` ASCII file: cube A at the origin + cube B
    // offset by (10, 0, 0). After decode → 2 meshes; topology says
    // 2 shells.
    let ascii = r#"solid A
  facet normal 0 0 1
    outer loop
      vertex 0 0 0
      vertex 1 0 0
      vertex 0 1 0
    endloop
  endfacet
endsolid A
solid B
  facet normal 0 0 1
    outer loop
      vertex 10 10 10
      vertex 11 10 10
      vertex 10 11 10
    endloop
  endfacet
endsolid B
"#;
    let scene = StlDecoder::new().decode(ascii.as_bytes()).unwrap();
    assert_eq!(scene.meshes.len(), 2);
    let s = shells(&scene);
    assert_eq!(s.len(), 2);
    assert_eq!(s[0].faces, 1);
    assert_eq!(s[1].faces, 1);
}

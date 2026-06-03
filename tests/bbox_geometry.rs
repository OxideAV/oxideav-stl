//! Integration tests for the round-219 `Bbox` geometry accessors
//! (`volume`, `surface_area`, `diagonal_length`, `longest_axis`,
//! `contains_point`) and the per-mesh / per-primitive `bbox_of_mesh`
//! / `bbox_of_primitive` variants. Round 225 extends these with
//! `Bbox::point` / `merge` / `expanded_by` composition tests.

use oxideav_mesh3d::{Mesh, Mesh3DDecoder, Mesh3DEncoder, Primitive, Scene3D, Topology};
use oxideav_stl::{bbox, bbox_of_mesh, bbox_of_primitive, Bbox, StlDecoder, StlEncoder};

/// Build a 2x3x4 ASCII STL "brick" — vertices at (0..2, 0..3, 0..4)
/// via two triangles on one bottom face. The bbox of the encoded
/// scene then has the same min/max regardless of triangle layout.
fn brick_2_3_4_scene() -> Scene3D {
    let positions: Vec<[f32; 3]> = vec![
        [0.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
        [2.0, 3.0, 0.0],
        [0.0, 3.0, 0.0],
        [0.0, 0.0, 4.0],
        [2.0, 0.0, 4.0],
        [2.0, 3.0, 4.0],
        [0.0, 3.0, 4.0],
    ];
    // 12-triangle box just for the bbox — winding is incidental.
    let face_pairs: Vec<[usize; 3]> = vec![
        [0, 2, 1],
        [0, 3, 2],
        [4, 5, 6],
        [4, 6, 7],
        [0, 1, 5],
        [0, 5, 4],
        [2, 3, 7],
        [2, 7, 6],
        [1, 2, 6],
        [1, 6, 5],
        [0, 4, 7],
        [0, 7, 3],
    ];
    let indices: Vec<u32> = face_pairs.into_iter().flatten().map(|i| i as u32).collect();
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = positions;
    prim.indices = Some(oxideav_mesh3d::Indices::U32(indices));
    let mut scene = Scene3D::new();
    scene.add_mesh(Mesh::new(Some("brick".to_string())).with_primitive(prim));
    scene
}

#[test]
fn brick_bbox_geometry_matches_extents() {
    let scene = brick_2_3_4_scene();
    let bb = bbox(&scene).expect("brick has a bbox");
    assert_eq!(bb.extents(), [2.0, 3.0, 4.0]);
    // V = 2 * 3 * 4 = 24.
    assert_eq!(bb.volume(), 24.0);
    // SA = 2 * (2*3 + 3*4 + 2*4) = 2 * (6 + 12 + 8) = 52.
    assert_eq!(bb.surface_area(), 52.0);
    // d = sqrt(4 + 9 + 16) = sqrt(29).
    let diag = bb.diagonal_length();
    assert!((diag - 29.0_f32.sqrt()).abs() < 1.0e-6);
    // Z (axis 2) is the longest.
    assert_eq!(bb.longest_axis(), Some(2));
}

#[test]
fn bbox_geometry_survives_binary_roundtrip() {
    // Decode/re-encode/re-decode shouldn't perturb the bbox accessors —
    // the floats are bit-identical through the binary path.
    let scene = brick_2_3_4_scene();
    let bytes = StlEncoder::new_binary().encode(&scene).unwrap();
    let decoded = StlDecoder::new().decode(&bytes).unwrap();
    let bb = bbox(&decoded).unwrap();
    assert_eq!(bb.extents(), [2.0, 3.0, 4.0]);
    assert_eq!(bb.volume(), 24.0);
    assert_eq!(bb.surface_area(), 52.0);
    assert_eq!(bb.longest_axis(), Some(2));
}

#[test]
fn bbox_contains_point_in_brick() {
    let scene = brick_2_3_4_scene();
    let bb = bbox(&scene).unwrap();
    // Strict interior + every corner.
    assert!(bb.contains_point([1.0, 1.5, 2.0]));
    for c in &[
        [0.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
        [0.0, 3.0, 0.0],
        [2.0, 3.0, 0.0],
        [0.0, 0.0, 4.0],
        [2.0, 0.0, 4.0],
        [0.0, 3.0, 4.0],
        [2.0, 3.0, 4.0],
    ] {
        assert!(bb.contains_point(*c), "corner {:?} should be inside", c);
    }
    // Just outside on each axis.
    assert!(!bb.contains_point([2.000001, 1.0, 1.0]));
    assert!(!bb.contains_point([1.0, 3.000001, 1.0]));
    assert!(!bb.contains_point([1.0, 1.0, 4.000001]));
}

#[test]
fn bbox_of_mesh_handles_multi_solid_ascii() {
    // Build a two-mesh scene via the multi-`solid` ASCII flavour:
    // mesh "a" at [0..1, 0..1, 0..1], mesh "b" at [10..12, 10..11, 10..11].
    let ascii = b"solid a
facet normal 0 0 1
outer loop
vertex 0.0 0.0 0.0
vertex 1.0 0.0 0.0
vertex 0.0 1.0 0.0
endloop
endfacet
endsolid a
solid b
facet normal 0 0 1
outer loop
vertex 10.0 10.0 10.0
vertex 12.0 10.0 10.0
vertex 10.0 11.0 10.0
endloop
endfacet
endsolid b
";
    let scene = StlDecoder::new().decode(ascii).unwrap();
    assert_eq!(scene.meshes.len(), 2);

    let a = bbox_of_mesh(&scene, 0).unwrap();
    assert_eq!(a.min, [0.0, 0.0, 0.0]);
    assert_eq!(a.max, [1.0, 1.0, 0.0]);

    let b = bbox_of_mesh(&scene, 1).unwrap();
    assert_eq!(b.min, [10.0, 10.0, 10.0]);
    assert_eq!(b.max, [12.0, 11.0, 10.0]);

    // The scene-level bbox spans both.
    let whole = bbox(&scene).unwrap();
    assert_eq!(whole.min, [0.0, 0.0, 0.0]);
    assert_eq!(whole.max, [12.0, 11.0, 10.0]);

    // Index past the last mesh.
    assert!(bbox_of_mesh(&scene, 2).is_none());
}

#[test]
fn bbox_of_primitive_isolates_within_a_single_mesh() {
    // Hand-build a Scene3D with two `Triangles` primitives on one mesh.
    let mut scene = Scene3D::new();
    let mut p1 = Primitive::new(Topology::Triangles);
    p1.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    let mut p2 = Primitive::new(Topology::Triangles);
    p2.positions = vec![[5.0, 5.0, 5.0], [6.0, 5.0, 5.0], [5.0, 6.0, 5.0]];
    scene.add_mesh(
        Mesh::new(Some("two_prims".to_string()))
            .with_primitive(p1)
            .with_primitive(p2),
    );

    let first = bbox_of_primitive(&scene, 0, 0).unwrap();
    assert_eq!(first.min, [0.0, 0.0, 0.0]);
    assert_eq!(first.max, [1.0, 1.0, 0.0]);

    let second = bbox_of_primitive(&scene, 0, 1).unwrap();
    assert_eq!(second.min, [5.0, 5.0, 5.0]);
    assert_eq!(second.max, [6.0, 6.0, 5.0]);

    // Out-of-range.
    assert!(bbox_of_primitive(&scene, 0, 2).is_none());
    assert!(bbox_of_primitive(&scene, 1, 0).is_none());
}

#[test]
fn longest_axis_picks_the_longer_dimension_after_translation() {
    // A scene whose Y extent dominates X and Z.
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = vec![
        [0.0, 0.0, 0.0],
        [1.0, 5.0, 0.0],
        [0.0, 5.0, 0.5],
        [1.0, 0.0, 0.5],
    ];
    prim.indices = Some(oxideav_mesh3d::Indices::U32(vec![0, 1, 2, 0, 3, 1]));
    let mut scene = Scene3D::new();
    scene.add_mesh(Mesh::new(None::<String>).with_primitive(prim));

    let bb = bbox(&scene).unwrap();
    assert_eq!(bb.extents(), [1.0, 5.0, 0.5]);
    assert_eq!(bb.longest_axis(), Some(1));
}

#[test]
fn merge_of_per_mesh_bboxes_equals_scene_wide_bbox() {
    // Build a multi-mesh scene: brick at the origin + a second mesh
    // offset to (+10, +10, +10). `bbox(&scene)` should match
    // `bbox_of_mesh(0).merge(bbox_of_mesh(1))`.
    let mut scene = brick_2_3_4_scene();
    let mut prim2 = Primitive::new(Topology::Triangles);
    prim2.positions = vec![
        [10.0, 10.0, 10.0],
        [12.0, 10.0, 10.0],
        [10.0, 13.0, 10.0],
        [10.0, 10.0, 14.0],
    ];
    prim2.indices = Some(oxideav_mesh3d::Indices::U32(vec![
        0, 1, 2, 0, 2, 3, 0, 3, 1,
    ]));
    scene.add_mesh(Mesh::new(Some("offset".to_string())).with_primitive(prim2));

    let whole = bbox(&scene).expect("non-empty scene");
    let m0 = bbox_of_mesh(&scene, 0).expect("mesh 0 bbox");
    let m1 = bbox_of_mesh(&scene, 1).expect("mesh 1 bbox");
    assert_eq!(m0.merge(&m1), whole);
    // And ordering is irrelevant.
    assert_eq!(m1.merge(&m0), whole);

    // Seed-from-point + merge reaches the same hull.
    let seed = Bbox::point([0.0, 0.0, 0.0]);
    let chained = seed.merge(&m0).merge(&m1);
    assert_eq!(chained, whole);
}

#[test]
fn expanded_bbox_contains_every_vertex_in_the_decoded_scene() {
    // Decode the brick through the binary path, then check that an
    // `expanded_by(margin)` envelope (a slicer pre-flight build-plate
    // safety margin) contains every emitted vertex of the round-
    // tripped scene with room to spare.
    let scene = brick_2_3_4_scene();
    let bytes = StlEncoder::new_binary()
        .encode(&scene)
        .expect("brick encodes");
    let decoded = StlDecoder::new().decode(&bytes).expect("brick decodes");

    let bb = bbox(&decoded).expect("decoded brick has a bbox");
    let envelope = bb.expanded_by(0.5);

    // Every emitted vertex sits inside the expanded envelope.
    for mesh in &decoded.meshes {
        for prim in &mesh.primitives {
            for p in &prim.positions {
                assert!(envelope.contains_point(*p), "vertex {p:?} not in envelope");
            }
        }
    }

    // The envelope strictly contains the original bbox's corners.
    assert!(envelope.contains_point(bb.min));
    assert!(envelope.contains_point(bb.max));
    // And the envelope's centre matches the original centre (symmetric expand).
    assert_eq!(envelope.centre(), bb.centre());
}

#[test]
fn point_merge_accumulator_matches_brute_force_bbox() {
    // Composition pattern: seed the accumulator with the first vertex,
    // then `merge` each subsequent `Bbox::point` and compare against
    // the brute-force `bbox(&scene)` walker.
    let scene = brick_2_3_4_scene();
    let mut iter = scene
        .meshes
        .iter()
        .flat_map(|m| m.primitives.iter())
        .filter(|p| p.topology == Topology::Triangles)
        .flat_map(|p| p.positions.iter().copied());

    let first = iter.next().expect("brick has at least one vertex");
    let mut acc = Bbox::point(first);
    for v in iter {
        acc = acc.merge(&Bbox::point(v));
    }

    let walker = bbox(&scene).expect("brick has a bbox");
    assert_eq!(acc, walker);
}

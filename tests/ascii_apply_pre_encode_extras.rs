//! ASCII-mode parity for `StlEncoder::apply_pre_encode_extras`.
//!
//! `apply_pre_encode_extras` mutates a `Scene3D` independently of the
//! eventual emit format — the round-4 tests already exercised it via
//! the binary encoder, but the contract is identical for ASCII. This
//! file mirrors the binary suite against `StlEncoder::new_ascii` so
//! the path is covered in both formats; if a future refactor ever
//! makes the hook format-aware, both halves of the suite will catch
//! the regression in lockstep.

use std::collections::HashMap;

use oxideav_mesh3d::{
    Indices, Mesh, Mesh3DDecoder, Mesh3DEncoder, Node, Primitive, Scene3D, Topology,
};
use oxideav_stl::{
    StlDecoder, StlEncoder, AUTO_INJECT_SHARE_FACTOR_THRESHOLD, UNIQUE_VERTEX_COUNT_EXTRAS_KEY,
};

fn build_indexed_cube() -> Scene3D {
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
    let mesh = Mesh {
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
            extras: HashMap::new(),
        }],
    };
    let mut scene = Scene3D::new();
    let mid = scene.add_mesh(mesh);
    let mut node = Node::new();
    node.mesh = Some(mid);
    let nid = scene.add_node(node);
    scene.add_root(nid);
    scene
}

fn build_unique_triangle_strip(triangles: usize) -> Scene3D {
    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(triangles * 3);
    for i in 0..triangles {
        let f = i as f32;
        positions.push([f, 0.0, 0.0]);
        positions.push([f, 1.0, 0.0]);
        positions.push([f, 0.0, 1.0]);
    }
    let prim = Primitive {
        topology: Topology::Triangles,
        positions,
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
    let mesh = Mesh {
        name: None,
        primitives: vec![prim],
    };
    let mut scene = Scene3D::new();
    scene.add_mesh(mesh);
    scene
}

#[test]
fn ascii_default_encoder_does_not_inject_extras() {
    let mut scene = build_indexed_cube();
    let enc = StlEncoder::new_ascii();
    assert!(!enc.auto_inject_unique_count());
    enc.apply_pre_encode_extras(&mut scene);
    let prim = &scene.meshes[0].primitives[0];
    assert!(!prim.extras.contains_key(UNIQUE_VERTEX_COUNT_EXTRAS_KEY));
}

#[test]
fn ascii_auto_inject_off_no_op_even_above_threshold() {
    let mut scene = build_indexed_cube();
    let enc = StlEncoder::new_ascii().with_auto_inject_unique_count(false);
    enc.apply_pre_encode_extras(&mut scene);
    assert!(!scene.meshes[0].primitives[0]
        .extras
        .contains_key(UNIQUE_VERTEX_COUNT_EXTRAS_KEY));
}

#[test]
fn ascii_auto_inject_on_high_share_factor_writes_unique_count() {
    let mut scene = build_indexed_cube();
    let pre_stats = StlEncoder::stats(&scene);
    assert!(pre_stats.share_factor() > AUTO_INJECT_SHARE_FACTOR_THRESHOLD);
    let enc = StlEncoder::new_ascii().with_auto_inject_unique_count(true);
    enc.apply_pre_encode_extras(&mut scene);
    let prim = &scene.meshes[0].primitives[0];
    let v = prim
        .extras
        .get(UNIQUE_VERTEX_COUNT_EXTRAS_KEY)
        .expect("key set");
    let count = v.as_u64().expect("integer");
    assert_eq!(count, pre_stats.unique_vertices as u64);
    assert_eq!(count, 8);
}

#[test]
fn ascii_auto_inject_on_low_share_factor_no_op() {
    let mut scene = build_unique_triangle_strip(4);
    let stats = StlEncoder::stats(&scene);
    assert!((stats.share_factor() - 1.0).abs() < 1e-6);
    let enc = StlEncoder::new_ascii().with_auto_inject_unique_count(true);
    enc.apply_pre_encode_extras(&mut scene);
    assert!(!scene.meshes[0].primitives[0]
        .extras
        .contains_key(UNIQUE_VERTEX_COUNT_EXTRAS_KEY));
}

#[test]
fn ascii_auto_inject_idempotent_overwrites() {
    let mut scene = build_indexed_cube();
    let enc = StlEncoder::new_ascii().with_auto_inject_unique_count(true);
    enc.apply_pre_encode_extras(&mut scene);
    enc.apply_pre_encode_extras(&mut scene);
    let v = scene.meshes[0].primitives[0]
        .extras
        .get(UNIQUE_VERTEX_COUNT_EXTRAS_KEY)
        .unwrap();
    assert_eq!(v.as_u64().unwrap(), 8);
}

#[test]
fn ascii_encode_pass_stays_pure_functional_on_immutable_scene() {
    let scene = build_indexed_cube();
    let mut enc = StlEncoder::new_ascii().with_auto_inject_unique_count(true);
    let _bytes = enc.encode(&scene).unwrap();
    assert!(!scene.meshes[0].primitives[0]
        .extras
        .contains_key(UNIQUE_VERTEX_COUNT_EXTRAS_KEY));
}

#[test]
fn ascii_injected_extras_round_trip_through_ascii_decoder_unchanged() {
    // Like the binary parity test: the decoder must NOT synthesise
    // the key. (The encoder doesn't carry it into the byte stream
    // either — STL has no native vertex sharing — so a clean
    // encode → decode yields a scene without the key.)
    let mut scene = build_indexed_cube();
    let enc = StlEncoder::new_ascii().with_auto_inject_unique_count(true);
    enc.apply_pre_encode_extras(&mut scene);
    let mut ascii_enc = StlEncoder::new_ascii().with_auto_inject_unique_count(true);
    let bytes = ascii_enc.encode(&scene).unwrap();
    let decoded = StlDecoder::new().decode(&bytes).unwrap();
    let prim = &decoded.meshes[0].primitives[0];
    assert!(!prim.extras.contains_key(UNIQUE_VERTEX_COUNT_EXTRAS_KEY));
}

#[test]
fn ascii_auto_inject_skips_non_triangles_primitives() {
    let mut scene = build_indexed_cube();
    scene.meshes[0].primitives.push(Primitive {
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
    });
    let enc = StlEncoder::new_ascii().with_auto_inject_unique_count(true);
    enc.apply_pre_encode_extras(&mut scene);
    let prims = &scene.meshes[0].primitives;
    assert!(prims[0].extras.contains_key(UNIQUE_VERTEX_COUNT_EXTRAS_KEY));
    assert!(!prims[1].extras.contains_key(UNIQUE_VERTEX_COUNT_EXTRAS_KEY));
}

#[test]
fn ascii_empty_scene_is_no_op_share_factor_zero() {
    let mut scene = Scene3D::new();
    let enc = StlEncoder::new_ascii().with_auto_inject_unique_count(true);
    enc.apply_pre_encode_extras(&mut scene);
    assert_eq!(scene.meshes.len(), 0);
}

#[test]
fn ascii_apply_pre_encode_extras_matches_binary_outcome() {
    // Direct parity check: invoking the hook through an ASCII-format
    // encoder vs a binary-format encoder must leave the scene in
    // identical states (same key → same value on the same primitive
    // slot). This is the test that would fail loudest if a future
    // refactor accidentally made the hook format-aware.
    let mut scene_ascii = build_indexed_cube();
    let mut scene_binary = build_indexed_cube();
    StlEncoder::new_ascii()
        .with_auto_inject_unique_count(true)
        .apply_pre_encode_extras(&mut scene_ascii);
    StlEncoder::new_binary()
        .with_auto_inject_unique_count(true)
        .apply_pre_encode_extras(&mut scene_binary);
    let a = scene_ascii.meshes[0].primitives[0]
        .extras
        .get(UNIQUE_VERTEX_COUNT_EXTRAS_KEY)
        .and_then(|v| v.as_u64());
    let b = scene_binary.meshes[0].primitives[0]
        .extras
        .get(UNIQUE_VERTEX_COUNT_EXTRAS_KEY)
        .and_then(|v| v.as_u64());
    assert_eq!(a, b);
    assert_eq!(a, Some(8));
}

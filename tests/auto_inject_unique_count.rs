//! Auto-injection of the `stl:unique_vertex_count` extras.
//!
//! When `StlEncoder::with_auto_inject_unique_count(true)` is set AND
//! the scene's `EncodeStats::share_factor()` exceeds
//! `AUTO_INJECT_SHARE_FACTOR_THRESHOLD` (1.5), the encoder's
//! `apply_pre_encode_extras` hook stamps every triangle primitive's
//! `extras["stl:unique_vertex_count"]` with the bit-exact unique-vertex
//! count from `StlEncoder::stats`. The actual `encode()` pass stays
//! pure-functional on `&Scene3D`; callers opt in to the mutation by
//! invoking the hook explicitly between configure-and-emit.

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
            targets: Vec::new(),
            extras: HashMap::new(),
        }],
        weights: Vec::new(),
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
    // No vertex sharing — share_factor will be exactly 1.0 because
    // every emitted vertex is unique.
    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(triangles * 3);
    for i in 0..triangles {
        let f = i as f32;
        // Three corners with no overlap with any other triangle.
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
        targets: Vec::new(),
        extras: HashMap::new(),
    };
    let mesh = Mesh {
        name: None,
        primitives: vec![prim],
        weights: Vec::new(),
    };
    let mut scene = Scene3D::new();
    scene.add_mesh(mesh);
    scene
}

#[test]
fn default_encoder_does_not_inject_extras() {
    let mut scene = build_indexed_cube();
    let enc = StlEncoder::new_binary();
    assert!(!enc.auto_inject_unique_count());
    enc.apply_pre_encode_extras(&mut scene);
    let prim = &scene.meshes[0].primitives[0];
    assert!(!prim.extras.contains_key(UNIQUE_VERTEX_COUNT_EXTRAS_KEY));
}

#[test]
fn auto_inject_off_no_op_even_above_threshold() {
    // Cube share_factor is 4.5 > 1.5; injection MUST still be inert
    // because the toggle is the gate.
    let mut scene = build_indexed_cube();
    let enc = StlEncoder::new_binary().with_auto_inject_unique_count(false);
    enc.apply_pre_encode_extras(&mut scene);
    assert!(!scene.meshes[0].primitives[0]
        .extras
        .contains_key(UNIQUE_VERTEX_COUNT_EXTRAS_KEY));
}

#[test]
fn auto_inject_on_high_share_factor_writes_unique_count() {
    let mut scene = build_indexed_cube();
    let pre_stats = StlEncoder::stats(&scene);
    assert!(pre_stats.share_factor() > AUTO_INJECT_SHARE_FACTOR_THRESHOLD);
    let enc = StlEncoder::new_binary().with_auto_inject_unique_count(true);
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
fn auto_inject_on_low_share_factor_no_op() {
    // share_factor = 1.0 (no sharing) → MUST NOT inject.
    let mut scene = build_unique_triangle_strip(4);
    let stats = StlEncoder::stats(&scene);
    assert!((stats.share_factor() - 1.0).abs() < 1e-6);
    let enc = StlEncoder::new_binary().with_auto_inject_unique_count(true);
    enc.apply_pre_encode_extras(&mut scene);
    assert!(!scene.meshes[0].primitives[0]
        .extras
        .contains_key(UNIQUE_VERTEX_COUNT_EXTRAS_KEY));
}

#[test]
fn auto_inject_idempotent_overwrites() {
    let mut scene = build_indexed_cube();
    let enc = StlEncoder::new_binary().with_auto_inject_unique_count(true);
    enc.apply_pre_encode_extras(&mut scene);
    enc.apply_pre_encode_extras(&mut scene);
    let v = scene.meshes[0].primitives[0]
        .extras
        .get(UNIQUE_VERTEX_COUNT_EXTRAS_KEY)
        .unwrap();
    assert_eq!(v.as_u64().unwrap(), 8);
}

#[test]
fn encode_pass_stays_pure_functional_on_immutable_scene() {
    // The `encode()` path takes `&Scene3D` and must NOT mutate; the
    // injection only happens via the explicit hook. After encoding
    // without invoking the hook, the scene's extras are untouched.
    let scene = build_indexed_cube();
    let mut enc = StlEncoder::new_binary().with_auto_inject_unique_count(true);
    let _bytes = enc.encode(&scene).unwrap();
    assert!(!scene.meshes[0].primitives[0]
        .extras
        .contains_key(UNIQUE_VERTEX_COUNT_EXTRAS_KEY));
}

#[test]
fn injected_extras_round_trip_through_binary_decoder_unchanged() {
    // Per the contract, the decoder leaves the injected key alone
    // (STL has no native vertex sharing — it's metadata only). After
    // a full encode → decode cycle the key is naturally absent (the
    // encoder doesn't carry it into the byte stream), so what we
    // really verify here is "the decoder doesn't synthesise it".
    let mut scene = build_indexed_cube();
    let enc = StlEncoder::new_binary().with_auto_inject_unique_count(true);
    enc.apply_pre_encode_extras(&mut scene);
    let mut bin_enc = StlEncoder::new_binary().with_auto_inject_unique_count(true);
    let bytes = bin_enc.encode(&scene).unwrap();
    let decoded = StlDecoder::new().decode(&bytes).unwrap();
    let prim = &decoded.meshes[0].primitives[0];
    assert!(!prim.extras.contains_key(UNIQUE_VERTEX_COUNT_EXTRAS_KEY));
}

#[test]
fn auto_inject_skips_non_triangles_primitives() {
    // A scene with a non-Triangles primitive shouldn't get the key
    // stamped on it (the encoder rejects such primitives anyway).
    // We mix one Triangles primitive (share_factor sustaining > 1.5)
    // with one Lines primitive in the SAME mesh and verify only the
    // Triangles slot picks up the extras.
    let mut scene = build_indexed_cube();
    // Append a Lines primitive to the same mesh.
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
        targets: Vec::new(),
        extras: HashMap::new(),
    });
    let enc = StlEncoder::new_binary().with_auto_inject_unique_count(true);
    enc.apply_pre_encode_extras(&mut scene);
    let prims = &scene.meshes[0].primitives;
    assert!(prims[0].extras.contains_key(UNIQUE_VERTEX_COUNT_EXTRAS_KEY));
    assert!(!prims[1].extras.contains_key(UNIQUE_VERTEX_COUNT_EXTRAS_KEY));
}

#[test]
fn empty_scene_is_no_op_share_factor_zero() {
    let mut scene = Scene3D::new();
    let enc = StlEncoder::new_binary().with_auto_inject_unique_count(true);
    enc.apply_pre_encode_extras(&mut scene);
    assert_eq!(scene.meshes.len(), 0);
}

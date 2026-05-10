//! Multi-`solid` ASCII STL parsing.
//!
//! Older Pro/E and AutoCAD ASCII exporters concatenate multiple
//! `solid NAME … endsolid NAME` blocks into a single file. The strict
//! 1989 spec defines exactly one block per file but the de-facto
//! tolerance across modern readers is to accept additional blocks
//! back-to-back.
//!
//! Coverage:
//! - Two-block file → two `Mesh` entries with the right names + face
//!   counts, two scene-root `Node`s, in source order.
//! - Three-block file with mixed names (named, anonymous, named) →
//!   each block lands as its own mesh.
//! - Round-trip: parse → encode → parse → confirm both names and
//!   geometry are preserved.

use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};
use oxideav_stl::{StlDecoder, StlEncoder};

const TWO_TRIANGLE_BLOCK: &str = "  facet normal 0 0 1\n    outer loop\n      vertex 0 0 0\n      vertex 1 0 0\n      vertex 0 1 0\n    endloop\n  endfacet\n  facet normal 0 0 1\n    outer loop\n      vertex 1 0 0\n      vertex 1 1 0\n      vertex 0 1 0\n    endloop\n  endfacet\n";

const ONE_TRIANGLE_BLOCK: &str = "  facet normal 0 0 1\n    outer loop\n      vertex 0 0 0\n      vertex 1 0 0\n      vertex 0 1 0\n    endloop\n  endfacet\n";

#[test]
fn two_solid_blocks_yield_two_meshes() {
    let s = format!(
        "solid first\n{TWO_TRIANGLE_BLOCK}endsolid first\nsolid second\n{ONE_TRIANGLE_BLOCK}endsolid second\n"
    );
    let scene = StlDecoder::new().decode(s.as_bytes()).unwrap();
    assert_eq!(scene.meshes.len(), 2);
    assert_eq!(scene.meshes[0].name.as_deref(), Some("first"));
    assert_eq!(scene.meshes[1].name.as_deref(), Some("second"));

    // First block has 2 triangles → 6 vertices, second has 1 triangle.
    assert_eq!(scene.meshes[0].primitives[0].positions.len(), 6);
    assert_eq!(scene.meshes[1].primitives[0].positions.len(), 3);
    assert_eq!(scene.triangle_count(), 3);

    // Each mesh wired to its own node attached to the root list.
    assert_eq!(scene.roots.len(), 2);
}

#[test]
fn three_block_mixed_named_and_anonymous() {
    let s = format!(
        "solid alpha\n{ONE_TRIANGLE_BLOCK}endsolid alpha\nsolid\n{ONE_TRIANGLE_BLOCK}endsolid\nsolid gamma\n{ONE_TRIANGLE_BLOCK}endsolid gamma\n"
    );
    let scene = StlDecoder::new().decode(s.as_bytes()).unwrap();
    assert_eq!(scene.meshes.len(), 3);
    assert_eq!(scene.meshes[0].name.as_deref(), Some("alpha"));
    assert!(
        scene.meshes[1].name.is_none(),
        "anonymous solid block should produce a name-less mesh"
    );
    assert_eq!(scene.meshes[2].name.as_deref(), Some("gamma"));
    assert_eq!(scene.triangle_count(), 3);
}

#[test]
fn multi_solid_survives_re_encode() {
    let s = format!(
        "solid a\n{ONE_TRIANGLE_BLOCK}endsolid a\nsolid b\n{TWO_TRIANGLE_BLOCK}endsolid b\n"
    );
    let scene = StlDecoder::new().decode(s.as_bytes()).unwrap();
    assert_eq!(scene.meshes.len(), 2);

    let encoded = StlEncoder::new_ascii().encode(&scene).unwrap();
    let scene2 = StlDecoder::new().decode(&encoded).unwrap();

    assert_eq!(scene2.meshes.len(), 2);
    assert_eq!(scene2.meshes[0].name.as_deref(), Some("a"));
    assert_eq!(scene2.meshes[1].name.as_deref(), Some("b"));
    assert_eq!(
        scene2.meshes[0].primitives[0].positions.len(),
        scene.meshes[0].primitives[0].positions.len()
    );
    assert_eq!(
        scene2.meshes[1].primitives[0].positions.len(),
        scene.meshes[1].primitives[0].positions.len()
    );
}

#[test]
fn single_solid_stays_single_mesh_unchanged() {
    // Regression — the single-block path used to be the only path.
    let s = format!("solid only\n{ONE_TRIANGLE_BLOCK}endsolid only\n");
    let scene = StlDecoder::new().decode(s.as_bytes()).unwrap();
    assert_eq!(scene.meshes.len(), 1);
    assert_eq!(scene.meshes[0].name.as_deref(), Some("only"));
    assert_eq!(scene.triangle_count(), 1);
}

#[test]
fn empty_input_rejected_with_clear_error() {
    let err = StlDecoder::new().decode(b"").unwrap_err();
    let msg = format!("{err:?}");
    // Detector calls binary path on empty input → InvalidData about
    // truncation. Either way the call must NOT panic.
    assert!(
        msg.contains("truncated") || msg.contains("solid"),
        "got error: {msg}"
    );
}

#[test]
fn whitespace_only_after_endsolid_is_clean_eof() {
    // Trailing newlines / blank lines after the last endsolid must
    // not trigger the "expected solid or eof" garbage detector.
    let s = format!("solid x\n{ONE_TRIANGLE_BLOCK}endsolid x\n\n\n   \n");
    let scene = StlDecoder::new().decode(s.as_bytes()).unwrap();
    assert_eq!(scene.meshes.len(), 1);
}

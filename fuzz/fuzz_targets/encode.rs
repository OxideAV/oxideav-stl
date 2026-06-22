#![no_main]

//! Drive the `StlEncoder` + dedup / stats surface on an arbitrary
//! fuzz-synthesised `Scene3D`.
//!
//! The existing targets never hand the **encoder** a hostile scene:
//!
//! * `roundtrip` only ever encodes a scene that first decoded cleanly
//!   from a synthesised binary STL, so the encoder always sees a
//!   well-formed single-mesh triangle soup with matched-length normals
//!   and no index buffer.
//! * `repair` builds arbitrary scenes but only runs the `validate` /
//!   `topology` analysis surface — it never calls `encode`, `stats`,
//!   or the unique-vertex dedup helpers.
//!
//! This target closes that gap: it builds a hostile `Scene3D` directly
//! from fuzz bytes (multiple meshes / primitives, every `Topology`,
//! indexed + unindexed, `U16`/`U32` index buffers with out-of-range
//! entries, `normals` arrays whose length disagrees with `positions`,
//! and NaN / ±Inf / subnormal coordinate bit-patterns) and then drives:
//!
//!   * `StlEncoder::new_binary().encode` — the fixed-50-byte-record
//!     packer, including the `stl:per_face_attributes` extras-recovery
//!     path and the face-normal fallback when the primitive's normal
//!     length disagrees with its vertex count;
//!   * `StlEncoder::new_ascii().encode` with **every** number-format
//!     knob — the default round-trip `{}` formatter,
//!     `with_float_precision`, and `with_spec_scientific` (the 1989
//!     `1.23456E+789` flavour) — so the ASCII float lexer's handling of
//!     non-finite coordinates (`NaN` / `inf` text) is exercised;
//!   * `StlEncoder::stats` + the bit-exact and spatial unique-vertex
//!     dedup helpers across a range of tolerances (including a negative
//!     / non-finite / zero `eps` that the spatial binner must clamp),
//!     driving the 27-cell neighbour scan + the `i32`-saturating bin
//!     index on absurd coordinates;
//!   * `apply_pre_encode_extras` with the auto-inject toggle on, then a
//!     re-encode of the mutated scene.
//!
//! Contract under test: *the call returns*. Every entry point takes a
//! caller-controlled `Scene3D` and must return a `Result` / report
//! rather than panic, index past a buffer, overflow on a bad index, or
//! divide by zero — no matter how internally inconsistent the scene is.
//! Return values are intentionally discarded. No external library is
//! consulted as an oracle (clean-room wall); panic-freedom is the whole
//! invariant.

use libfuzzer_sys::fuzz_target;
use oxideav_mesh3d::{Indices, Mesh, Mesh3DEncoder, Node, Primitive, Scene3D, Topology};
use oxideav_stl::{EncodeStats, StlEncoder};

/// A forward byte cursor that zero-pads past the end so a short input
/// still produces a (small) well-defined scene rather than an early
/// return — the encoder paths are the point.
struct Cursor<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Cursor<'a> {
    fn new(data: &'a [u8]) -> Self {
        Self { data, pos: 0 }
    }

    fn u8(&mut self) -> u8 {
        let b = self.data.get(self.pos).copied().unwrap_or(0);
        self.pos += 1;
        b
    }

    /// Bounded count read so a single nibble can't request a multi-
    /// million-element buffer (libfuzzer per-input memory budget).
    fn count(&mut self, max: usize) -> usize {
        (self.u8() as usize) % (max + 1)
    }

    /// One `f32` from four cursor bytes as a raw LE bit pattern.
    /// Deliberately includes NaN / Inf / subnormal so the non-finite
    /// branches in the ASCII float formatter + the spatial binner are
    /// hit.
    fn f32(&mut self) -> f32 {
        let b = [self.u8(), self.u8(), self.u8(), self.u8()];
        f32::from_bits(u32::from_le_bytes(b))
    }

    fn vec3(&mut self) -> [f32; 3] {
        [self.f32(), self.f32(), self.f32()]
    }
}

/// Caps so one fuzz iteration stays inside libfuzzer's memory budget.
const MAX_MESHES: usize = 4;
const MAX_PRIMS: usize = 4;
const MAX_VERTS: usize = 48;
const MAX_INDICES: usize = 64;

fn topology_of(b: u8) -> Topology {
    match b % 7 {
        0 => Topology::Triangles,
        1 => Topology::TriangleStrip,
        2 => Topology::TriangleFan,
        3 => Topology::Lines,
        4 => Topology::LineStrip,
        5 => Topology::LineLoop,
        _ => Topology::Points,
    }
}

fn build_primitive(cur: &mut Cursor) -> Primitive {
    let topology = topology_of(cur.u8());
    let mut prim = Primitive::new(topology);

    let n_pos = cur.count(MAX_VERTS);
    prim.positions = (0..n_pos).map(|_| cur.vec3()).collect();

    // Three normal-length regimes so the encoder's "normal length
    // matches vertex count?" branch is exercised both ways.
    match cur.u8() % 3 {
        0 => prim.normals = None,
        1 => prim.normals = Some((0..n_pos).map(|_| cur.vec3()).collect()),
        _ => {
            let n_norm = cur.count(MAX_VERTS);
            prim.normals = Some((0..n_norm).map(|_| cur.vec3()).collect());
        }
    }

    // Four index regimes — unindexed, in-range-ish U16, raw U16, raw
    // U32 (the last two deliberately dangle past the position slice so
    // the encoder's index-resolution must bounds-check).
    let n_idx = cur.count(MAX_INDICES);
    match cur.u8() % 4 {
        0 => prim.indices = None,
        1 => {
            let modulus = (n_pos + 1).max(1) as u16;
            let v: Vec<u16> = (0..n_idx).map(|_| (cur.u8() as u16) % modulus).collect();
            prim.indices = Some(Indices::U16(v));
        }
        2 => {
            let v: Vec<u16> = (0..n_idx)
                .map(|_| u16::from_le_bytes([cur.u8(), cur.u8()]))
                .collect();
            prim.indices = Some(Indices::U16(v));
        }
        _ => {
            let v: Vec<u32> = (0..n_idx)
                .map(|_| u32::from_le_bytes([cur.u8(), cur.u8(), cur.u8(), cur.u8()]))
                .collect();
            prim.indices = Some(Indices::U32(v));
        }
    }

    prim
}

fn build_scene(data: &[u8]) -> Scene3D {
    let mut cur = Cursor::new(data);
    let mut scene = Scene3D::new();

    let n_meshes = cur.count(MAX_MESHES);
    for _ in 0..n_meshes {
        let mut mesh = Mesh::new(None);
        let n_prims = cur.count(MAX_PRIMS);
        for _ in 0..n_prims {
            mesh.primitives.push(build_primitive(&mut cur));
        }
        let mesh_id = scene.add_mesh(mesh);
        let node = Node::new().with_mesh(mesh_id);
        let node_id = scene.add_node(node);
        scene.add_root(node_id);
    }

    scene
}

fuzz_target!(|data: &[u8]| {
    let scene = build_scene(data);

    // --- binary encode ---
    let _ = StlEncoder::new_binary().encode(&scene);

    // --- ASCII encode across every number-format knob ---
    let _ = StlEncoder::new_ascii().encode(&scene);
    let _ = StlEncoder::new_ascii()
        .with_float_precision(Some(6))
        .encode(&scene);
    let _ = StlEncoder::new_ascii()
        .with_float_precision(Some(0))
        .encode(&scene);
    let _ = StlEncoder::new_ascii()
        .with_spec_scientific(Some(6))
        .encode(&scene);
    let _ = StlEncoder::new_ascii()
        .with_spec_scientific(Some(0))
        .encode(&scene);

    // --- stats + dedup helpers across a tolerance spread ---
    let _ = StlEncoder::stats(&scene);
    for &eps in &[
        -1.0f32,
        f32::NAN,
        f32::INFINITY,
        0.0,
        1e-6,
        1e-3,
        1.0,
        1e30,
    ] {
        let _ = StlEncoder::unique_vertices_with_tolerance(&scene, eps);
        let _ = StlEncoder::unique_vertices_with_tolerance_spatial(&scene, eps);
        let _ = EncodeStats::with_tolerance(&scene, eps);
        let _ = EncodeStats::with_tolerance_spatial(&scene, eps);
    }

    // --- pre-encode extras auto-injection, then re-encode the mutant ---
    let mut mutated = scene;
    let mut enc = StlEncoder::new_binary().with_auto_inject_unique_count(true);
    enc.apply_pre_encode_extras(&mut mutated);
    let _ = enc.encode(&mutated);
    let _ = StlEncoder::new_ascii().encode(&mutated);
});

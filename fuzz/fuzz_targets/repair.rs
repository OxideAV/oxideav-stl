#![no_main]

//! Drive the `validate` + `topology` (repair) surface on an arbitrary
//! fuzz-synthesised `Scene3D`.
//!
//! The `decode` / `roundtrip` / `triage` targets all start from a byte
//! slice and exercise the *parsing* paths. None of them reach the
//! geometric-analysis surface — the bulk of the crate — because the
//! decoder only ever hands those passes a well-formed single-mesh
//! triangle soup. This target closes that gap: it builds a hostile
//! `Scene3D` directly from fuzz bytes (multiple meshes, multiple
//! primitives, every `Topology`, indexed and unindexed, `U16`/`U32`
//! index buffers carrying deliberately out-of-range entries,
//! `normals` arrays whose length disagrees with `positions`, and
//! coordinate bit-patterns that decode to NaN / ±Inf / subnormal) and
//! then runs:
//!
//!   * `validate` with **every** rule turned on, including the two
//!     opt-in brute-force sub-checks (`check_t_junctions`,
//!     `check_positive_octant`);
//!   * `bbox` / `bbox_of_mesh` / `bbox_of_primitive`;
//!   * `shells` connected-component analysis;
//!   * every mutating repair pass, in the documented pipeline order,
//!     each on its own clone so a pass that rewrites the index buffer
//!     can't mask a panic in a later pass that expected the original
//!     layout.
//!
//! Contract under test: *the call returns*. Every one of these entry
//! points takes a caller-controlled `Scene3D` and must never panic,
//! index past a buffer, divide by zero, overflow on a bad index, or
//! OOM, no matter how internally inconsistent the scene is. The return
//! values are intentionally discarded. No external library is consulted
//! as an oracle (clean-room wall); panic-freedom is the whole invariant.

use libfuzzer_sys::fuzz_target;
use oxideav_mesh3d::{Indices, Mesh, Node, Primitive, Scene3D, Topology};
use oxideav_stl::{
    bbox, bbox_of_mesh, bbox_of_primitive, boundary_loops, check_z_sorted, mesh_edge_length_stats,
    mesh_surface_area, mesh_volume, repair_cap_boundary_loops, repair_drop_degenerate_triangles,
    repair_make_winding_consistent, repair_normalize_unit_normals,
    repair_orient_normals_from_winding, repair_recompute_zero_normals, repair_sort_triangles_by_z,
    repair_split_t_junctions, repair_translate_to_positive_octant, repair_weld_vertices, shells,
    validate, ValidationOptions,
};

/// A forward byte cursor over the fuzz buffer. Reads zero-pad past the
/// end so a short input still produces a (small) well-defined scene
/// rather than an early return — the geometric passes are the point,
/// and a fully-zero scene still exercises the degenerate / collinear /
/// origin-corner branches.
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

    /// One `f32` from four cursor bytes, interpreted as a raw LE bit
    /// pattern. Deliberately includes NaN / Inf / subnormal so the
    /// non-finite-skip branches in `bbox` / the repair passes are hit.
    fn f32(&mut self) -> f32 {
        let b = [self.u8(), self.u8(), self.u8(), self.u8()];
        f32::from_bits(u32::from_le_bytes(b))
    }

    fn vec3(&mut self) -> [f32; 3] {
        [self.f32(), self.f32(), self.f32()]
    }
}

/// Cap so one fuzz iteration stays inside libfuzzer's memory budget.
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

    // Positions.
    let n_pos = cur.count(MAX_VERTS);
    prim.positions = (0..n_pos).map(|_| cur.vec3()).collect();

    // Normals — three independent length regimes selected by a control
    // byte, so the length-mismatch branches (`skipped_length_mismatch`
    // in several reports) are reachable alongside the matched-length
    // path:
    //   0 => no normals
    //   1 => matched length (the well-formed case)
    //   _ => a deliberately mismatched length
    match cur.u8() % 3 {
        0 => prim.normals = None,
        1 => {
            prim.normals = Some((0..n_pos).map(|_| cur.vec3()).collect());
        }
        _ => {
            let n_norm = cur.count(MAX_VERTS);
            prim.normals = Some((0..n_norm).map(|_| cur.vec3()).collect());
        }
    }

    // Index buffer — four regimes via a control byte:
    //   0 => unindexed (None)
    //   1 => U16 with entries reduced modulo (pos.len + 1) so most are
    //        in range but some can dangle one past the end
    //   2 => U16 with raw entries (deliberately out of range)
    //   _ => U32 with raw entries (deliberately out of range)
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
        // Attach the mesh to a root node so `validate` / `shells` /
        // `bbox` walk it the same way a decoded scene would.
        let node = Node::new().with_mesh(mesh_id);
        let node_id = scene.add_node(node);
        scene.add_root(node_id);
    }

    scene
}

/// All rules on, including the two opt-in brute-force sub-checks, so
/// the T-junction and positive-octant scanners are exercised too.
fn all_rules() -> ValidationOptions {
    ValidationOptions {
        check_positive_octant: true,
        check_facet_orientation: true,
        check_unit_normal: true,
        check_watertight: true,
        check_t_junctions: true,
        check_consistent_winding: true,
        check_degenerate_triangles: true,
        check_zero_area_triangles: true,
        ..ValidationOptions::default()
    }
}

fuzz_target!(|data: &[u8]| {
    let scene = build_scene(data);

    // --- non-mutating analysis surface ---
    let _ = validate(&scene, &all_rules());
    let _ = validate(&scene, &ValidationOptions::default());
    let _ = bbox(&scene);
    for m in 0..scene.meshes.len() {
        let _ = bbox_of_mesh(&scene, m);
        let prim_count = scene.meshes[m].primitives.len();
        for p in 0..prim_count {
            let _ = bbox_of_primitive(&scene, m, p);
        }
    }
    let _ = shells(&scene);
    let _ = boundary_loops(&scene);
    let _ = mesh_volume(&scene);
    let _ = mesh_surface_area(&scene);
    let _ = mesh_edge_length_stats(&scene);
    let _ = check_z_sorted(&scene);

    // --- mutating repair surface ---
    // Each pass runs on its own clone so a pass that rewrites the index
    // buffer / position soup cannot mask a panic in a later pass that
    // assumed the original layout. Every one of these takes a caller-
    // controlled scene and must return its report rather than panic.
    let _ = repair_weld_vertices(&mut scene.clone());
    let _ = repair_drop_degenerate_triangles(&mut scene.clone());
    let _ = repair_recompute_zero_normals(&mut scene.clone(), 0.0);
    let _ = repair_recompute_zero_normals(&mut scene.clone(), 1e-3);
    let _ = repair_orient_normals_from_winding(&mut scene.clone(), 0.0);
    let _ = repair_normalize_unit_normals(&mut scene.clone(), 1e-3);
    let _ = repair_sort_triangles_by_z(&mut scene.clone());
    let _ = repair_translate_to_positive_octant(&mut scene.clone(), 1e-6);
    let _ = repair_make_winding_consistent(&mut scene.clone());
    let _ = repair_split_t_junctions(&mut scene.clone(), 1e-5);
    let _ = repair_cap_boundary_loops(&mut scene.clone());

    // --- the documented full pipeline, in order, on a single scene ---
    // The README spells out a nine-step repair sequence; running it
    // end-to-end on one mutating scene exercises the interaction
    // between passes (e.g. weld surfacing degenerates the drop pass
    // then removes, the splitter re-walking a freshly welded soup).
    let mut piped = scene;
    let _ = repair_weld_vertices(&mut piped);
    let _ = repair_drop_degenerate_triangles(&mut piped);
    let _ = repair_recompute_zero_normals(&mut piped, 0.0);
    let _ = repair_orient_normals_from_winding(&mut piped, 0.0);
    let _ = repair_normalize_unit_normals(&mut piped, 1e-3);
    let _ = repair_sort_triangles_by_z(&mut piped);
    let _ = repair_translate_to_positive_octant(&mut piped, 1e-6);
    let _ = repair_make_winding_consistent(&mut piped);
    let _ = repair_split_t_junctions(&mut piped, 1e-5);
    let _ = repair_cap_boundary_loops(&mut piped);

    // Re-validate the fully-repaired scene — the repair pipeline must
    // leave a scene the validator can still walk without panicking.
    let _ = validate(&piped, &all_rules());
});

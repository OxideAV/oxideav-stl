# oxideav-stl

Pure-Rust STL (stereolithography) ASCII + binary 3D mesh codec.

STL is the *de-facto* mesh-exchange format for additive manufacturing,
defined by 3D Systems' *StereoLithography Interface Specification*
(October 1989, revised 1995). It encodes a single triangulated surface
as either:

- **ASCII** — `solid <name> … facet normal nx ny nz / outer loop /
  vertex x y z (×3) / endloop / endfacet … endsolid <name>` — token
  grammar, whitespace-separated, case-insensitive keywords.
- **Binary** — 80-byte header + `uint32` triangle count + `N × 50`
  bytes per triangle (`float[3]` normal, three `float[3]` vertex
  coordinates, `uint16` attribute byte count).

This crate plugs into `oxideav-mesh3d` as both a decoder and an
encoder, mapping STL ↔ `Scene3D`:

| STL                                    | `Scene3D`                                                    |
| -------------------------------------- | ------------------------------------------------------------ |
| One file → one mesh                    | Single root `Node` → single `Mesh` → single `Primitive`      |
| `solid <name>` / `endsolid <name>`     | `Mesh::name`                                                 |
| Per-face normal                        | Per-vertex normal (3 copies per face)                        |
| Three `vertex x y z` per facet         | Three `[f32; 3]` entries in `Primitive::positions`           |
| Implicit Z-up                          | `Scene3D::up_axis = Axis::PosZ`                              |
| No unit                                | `Scene3D::unit = Unit::Millimetres` (slicer-default)         |
| Non-zero attribute byte count (binary) | `Mesh::extras["stl:per_face_attributes"] = <hex string>`     |

## Format detection

A layered set of signals distinguishes ASCII from binary:

1. UTF-8 BOM and leading whitespace are stripped first.
2. Prefix must be `solid` followed by an ASCII whitespace byte —
   `solidWORKS` and other vendor-prefixed binary headers are rejected
   here.
3. A `\n`+optional-whitespace+`facet` token must appear in the first
   1 KiB.
4. Even when 1–3 all pass, the detector overrides to **binary** if
   the file length matches `84 + N*50` for the LE u32 count at offset
   80 (and `N > 0`). This catches CADKey-2002 / Microsoft / AutoCAD
   binary headers that embed a `\n facet` substring inside the
   80-byte vendor string.

## Materialise per-object default colour and material

Materialise Magics writes per-object default tokens into the 80-byte
binary STL header as ASCII lines:

```text
COLOR=R G B A
MATERIAL=Ar Ag Ab Sa Dr Dg Db Sd Sr Sg Sb Ss
```

Either or both are picked up on decode and surfaced on
`Primitive::extras["stl:default_color"]` (4-element `u8` JSON array)
and `Primitive::extras["stl:default_material"]` (12-element `u8` JSON
array). The encoder rebuilds a Materialise-compatible header on
re-emit when either key is set, so the round-trip is faithful. These
defaults are what per-face slots whose Materialise "valid" bit reports
"use default" refer back to.

## Multi-`solid` ASCII files

Older Pro/E + AutoCAD exporters concatenate multiple `solid NAME …
endsolid NAME` blocks into a single `.stl`. Each block becomes its
own `Mesh` in the resulting `Scene3D`, with one root `Node` per mesh
attached to the scene's `roots` list in source order. The encoder
mirrors this on output.

## ASCII number-formatting knobs

By default the ASCII encoder uses Rust's round-trip-safe `{}`
formatter for f32 values. Three other policies are available:

```rust
use oxideav_stl::StlEncoder;

// Fixed-decimal (compact / diff-friendly):
let _enc = StlEncoder::new_ascii().with_float_precision(Some(6));
// vertex 0.123457 0.000000 0.000000

// Spec-style scientific (matches the 1989 spec's `1.23456E+789`
// worked example verbatim — explicit `+`/`-` exponent sign):
let _enc = StlEncoder::new_ascii().with_spec_scientific(Some(5));
// vertex 1.23457E+0 0.00000E+0 0.00000E+0
```

The full policy enum is also exposed for callers that wire the knob
through a higher-level configuration plumbing:

```rust
use oxideav_stl::{AsciiNumberFormat, StlEncoder};

let _enc = StlEncoder::new_ascii()
    .with_number_format(AsciiNumberFormat::SpecScientific { precision: 4 });
```

All three policies have no effect on binary output (binary triangle
records are byte-identical regardless of the knob setting).

## Pre-encode statistics

`StlEncoder::stats(&scene)` returns an `EncodeStats { triangles,
emitted_vertices, unique_vertices }` summary without paying for the
full encode pass. Useful for tooling that wants to report
shared-index → STL expansion ratios:

```rust
let s = oxideav_stl::StlEncoder::stats(&scene);
println!("share factor = {:.2}", s.share_factor());
```

Unique-vertex matching uses exact `f32` bit-pattern comparison.

For meshes whose corners differ by floating-point noise (CAD/scanner
pipelines often emit positions that vary by ~1e-6 between "the same"
logical vertex), a tolerance-based view is also available:

```text
use oxideav_stl::{EncodeStats, StlEncoder};

// `with_tolerance` only changes `unique_vertices`; `triangles` and
// `emitted_vertices` come straight from the bit-exact path.
let s = EncodeStats::with_tolerance(&scene, 1.0e-5);

// `(unique_count, dedup_map)` — `dedup_map[i]` is the canonical-slot
// index assigned to the i-th *emitted* vertex, in encoder order.
let (unique, dedup_map) = StlEncoder::unique_vertices_with_tolerance(&scene, 1.0e-5);
```

`eps == 0.0` (or any negative / non-finite value) is clamped to the
bit-exact path. The scan is `O(N · K)` where `K` is the running
canonical count — fine for diagnostic use; large-mesh callers
should reach for the spatial-index variant:

```text
let s = EncodeStats::with_tolerance_spatial(&scene, 1.0e-5);
let (unique, dedup_map) =
    StlEncoder::unique_vertices_with_tolerance_spatial(&scene, 1.0e-5);
```

The spatial path bins each vertex into a uniform-grid cell of side
`eps × 2` and scans the 27 surrounding cells, amortising to `O(N)`
for typical geometry. With `eps == 0.0` it short-circuits to the
bit-exact branch and produces results identical to the brute-force
path; for `eps > 0` it is approximate by design — every two points
it merges are within `eps` on every axis under the Chebyshev
metric, but it may emit one additional canonical when borderline
points fall into non-adjacent cells.

## Auto-inject `stl:unique_vertex_count`

Opt-in encoder hook that stamps the bit-exact unique-vertex count onto
every triangle primitive's `Primitive::extras["stl:unique_vertex_count"]`
*before* emit, gated on the scene's `share_factor()` exceeding `1.5`
(the `AUTO_INJECT_SHARE_FACTOR_THRESHOLD` constant). The standard
`encode()` pass takes `&Scene3D` and stays pure-functional; callers
opt in via a separate one-line hook:

```text
let mut enc = StlEncoder::new_binary().with_auto_inject_unique_count(true);
enc.apply_pre_encode_extras(&mut scene);
let bytes = enc.encode(&scene)?;
```

The decoder leaves the key alone — STL has no native vertex sharing,
so the value is metadata for downstream tooling, not part of the byte
stream. Re-running the hook on a scene that already carries the key
overwrites with a freshly-recomputed count.

## 16-bit per-face colour

The binary STL `uint16` attribute slot (spec-defined as zero) is
repurposed by two vendor conventions for packed RGB:

- **VisCAM / SolidView** — `[valid:1][R:5][G:5][B:5]`, `valid=1`
  means the per-face triple is live.
- **Materialise Magics** — `[valid:1][B:5][G:5][R:5]`, `valid=0`
  means live (channel order AND valid-bit polarity inverted).

The decoder runs `oxideav_stl::color::detect()` against the per-face
attribute population and, when the bit-15 distribution is
unambiguous, records the convention on
`Primitive::extras["stl:color_convention"]` as `"viscam"` or
`"materialise"`. The raw bytes always round-trip through
`Primitive::extras["stl:per_face_attributes"]`.

```rust
use oxideav_mesh3d::Mesh3DDecoder;
use oxideav_stl::{ColorConvention, Stl16BitColor, StlDecoder};

# fn main() -> Result<(), Box<dyn std::error::Error>> {
let bytes = std::fs::read("colored.stl")?;
let scene = StlDecoder::new().decode(&bytes)?;
let prim = &scene.meshes[0].primitives[0];
if let Some(tag) = prim.extras.get("stl:color_convention").and_then(|v| v.as_str()) {
    let convention: ColorConvention = tag.parse().unwrap();
    let hex = prim.extras["stl:per_face_attributes"].as_str().unwrap();
    // Each 4 hex chars encode one 16-bit slot as `[lo_hi_byte,
    // hi_hi_byte]` — i.e. file-byte order. Pair the bytes back into
    // an LE u16.
    for chunk in hex.as_bytes().chunks_exact(4) {
        let lo = u8::from_str_radix(std::str::from_utf8(&chunk[..2])?, 16)?;
        let hi = u8::from_str_radix(std::str::from_utf8(&chunk[2..])?, 16)?;
        let slot = u16::from_le_bytes([lo, hi]);
        let _color = Stl16BitColor::from_word(convention, slot);
        // … render with `_color` …
    }
}
# Ok(())
# }
```

## Usage

```rust
use oxideav_mesh3d::{Mesh3DDecoder, Mesh3DEncoder};
use oxideav_stl::{StlDecoder, StlEncoder};

let bytes = std::fs::read("cube.stl")?;
let scene = StlDecoder::new().decode(&bytes)?;

// ... edit `scene` ...

let out = StlEncoder::new_binary().encode(&scene)?;
std::fs::write("cube-out.stl", out)?;
# Ok::<_, Box<dyn std::error::Error>>(())
```

## Standalone build

`oxideav-core` is gated behind the default-on `registry` cargo
feature. Drop the framework dependency entirely with:

```toml
oxideav-stl = { version = "0.0", default-features = false }
```

The encoder + decoder API stays available; the `register()` entry
point + `Mesh3DRegistry` plumbing disappear and the error type falls
back to `oxideav_mesh3d`'s crate-local enum.

## Validation + bounding box

Opt-in spec-aligned geometry validation lives in
`oxideav_stl::validate`. The standard rules — facet orientation
(right-hand rule), unit-length normal, vertex-to-vertex (watertight /
manifold), and the SLA-era all-positive-octant rule — are applied
to a `Scene3D` without mutating it, returning a `ValidationReport`
with per-rule counts plus up to `MAX_REPORTED_DEFECTS` (32)
illustrative `FaceLocator { mesh, primitive, face }` indices for
each rule.

```rust
use oxideav_stl::{validate, ValidationOptions};

# let scene = oxideav_mesh3d::Scene3D::new();
let report = validate(&scene, &ValidationOptions::default());
if !report.watertight {
    eprintln!("not watertight: {} boundary edges", report.boundary_edges);
}
```

The positive-octant rule (which no modern slicer enforces) is off by
default; toggle `ValidationOptions::check_positive_octant = true`
when interoperating with strict-spec consumers. All other rules
default on. `oxideav_stl::bbox(&scene)` returns an `Option<Bbox>`
covering every `Triangles` vertex (non-finite coordinates skipped);
the scene's node-graph transforms are not applied — STL produces
identity-transform single-mesh trees in practice.

## Topology utilities

Opt-in, non-mutating analysis of the triangle soup lives in
`oxideav_stl::topology`. Two utilities + one mutating repair:

```rust
use oxideav_stl::{repair_weld_vertices, shells};

# let mut scene = oxideav_mesh3d::Scene3D::new();
// Connected components (bit-exact shared-vertex BFS).
for shell in shells(&scene) {
    if shell.is_closed_manifold() {
        eprintln!(
            "closed shell: V={} E={} F={} chi={} genus={:?}",
            shell.vertices,
            shell.edges,
            shell.faces,
            shell.euler_characteristic(),
            shell.genus(),
        );
    } else {
        eprintln!(
            "open shell: {} boundary edges",
            shell.boundary_edges
        );
    }
}

// Trivial bit-exact weld — collapses an unindexed triangle soup
// into a shared `Indices::U32` buffer. Returns a `WeldReport` whose
// `positions_collapsed == 0` is the idempotency signal.
let report = repair_weld_vertices(&mut scene);
```

`shells` uses **shared-vertex** adjacency (any single corner
position match links two triangles into the same shell) — more
permissive than edge-adjacency to catch CSG-style corner-touching
geometry. Vertex equality uses bit-exact `f32` matching; corners
that differ by floating-point noise should be pre-deduplicated via
`StlEncoder::unique_vertices_with_tolerance_spatial`.

`repair_weld_vertices` only mutates `prim.positions` /
`prim.normals` / `prim.indices` on `Triangles` primitives;
`prim.extras`, `mesh.name`, and the scene-graph `nodes` / `roots`
structure are preserved. Degenerate triangles (post-weld, any two
indices coincide) are *reported* by `WeldReport.degenerate_triangles`
and *removed* in a separate pass via `repair_drop_degenerate_triangles`
— see the next section.

## Degenerate-triangle culling

```rust
use oxideav_stl::repair_drop_degenerate_triangles;

# let mut scene = oxideav_mesh3d::Scene3D::new();
// Drop zero-area triangles in-place. A triangle is degenerate when
// any two of its three corner *positions* coincide by bit-exact
// `f32` match. `DegenerateDropReport { triangles_inspected,
// dropped_triangles }` — `dropped_triangles == 0` is the
// idempotency signal.
let report = repair_drop_degenerate_triangles(&mut scene);
```

Operates on every `Triangles` primitive in the scene; non-`Triangles`
primitives are left untouched. Indexed primitives keep their
`Indices::U16` / `Indices::U32` discriminant and have their index
buffer rewritten with the surviving triangle slots. Unindexed
primitives have their `positions` (and matching-length `normals`)
compacted in place. The pass intentionally uses position-equality
rather than zero-cross-product to avoid culling hairline strips that
CAD pipelines deliberately emit; callers who want zero-cross culling
can pre-filter via `validate`'s orientation report.

The natural sequence is `repair_weld_vertices` →
`repair_drop_degenerate_triangles` — the weld pass surfaces hidden
degenerates by collapsing duplicated corners, and the drop pass
removes them.

## Zero-normal recompute (RHR sentinel)

```rust
use oxideav_stl::repair_recompute_zero_normals;

# let mut scene = oxideav_mesh3d::Scene3D::new();
// Walk every Triangles primitive and, for each face whose three
// stored per-vertex normals are all (within `eps`) zero, rewrite
// them with the right-hand-rule cross product of its positions.
// Triangles with even one non-zero stored normal slot are left
// alone. Missing `normals` arrays are freshly populated.
let report = repair_recompute_zero_normals(&mut scene, 0.0);
```

Implements the STL spec's documented "stored zero normal = consumer
should recompute from winding" sentinel. With `eps == 0.0` only an
exact-zero triple triggers the recompute; widening `eps` to
`1e-6`..`1e-3` lets producer-emitted floating-point-noise zeros pass
the gate. `NormalRecomputeReport { triangles_inspected,
recomputed_triangles, skipped_degenerate, primitives_populated,
skipped_length_mismatch }` — `recomputed_triangles == 0` is the
idempotency signal; a non-zero `skipped_degenerate` highlights faces
whose three corners are collinear / coincident and need to go
through `repair_drop_degenerate_triangles` instead.

## Orientation flip + unit-length rescale (spec consistency)

The 1989 spec says facet orientation is "specified redundantly in
two ways which must be consistent": the stored normal points outward
and the winding is CCW viewed from outside (right-hand rule). When a
producer emits a stored normal whose direction disagrees with the
winding (dot product < 0), `repair_orient_normals_from_winding`
rewrites the stored normal to match the winding — winding is the
authoritative source:

```rust
use oxideav_stl::repair_orient_normals_from_winding;

# let mut scene = oxideav_mesh3d::Scene3D::new();
let report = repair_orient_normals_from_winding(&mut scene, 0.0);
```

Per-face decision: zero-sentinel normals are skipped (use
`repair_recompute_zero_normals` first), cross-product-below-`eps`
triangles are counted under `OrientReport::skipped_degenerate`, and
non-`Triangles` primitives are silently skipped. `flipped_normals ==
0` is the idempotency signal.

The companion `repair_normalize_unit_normals(&mut scene, tol)`
rescales any non-unit stored normal to unit length, preserving
direction. The 1989 spec says each facet's normal is a *unit* vector;
the validate module flags non-unit stored normals under
`non_unit_normal_defects` with the same tolerance constant, and this
is the matching mutating fix-up:

```rust
use oxideav_stl::repair_normalize_unit_normals;

# let mut scene = oxideav_mesh3d::Scene3D::new();
// Default tolerance matches validate::DEFAULT_UNIT_NORMAL_TOLERANCE
// (1e-3). Negative or non-finite values clamp to that default.
let report = repair_normalize_unit_normals(&mut scene, 1e-3);
```

`rescaled_normals == 0` is the idempotency signal. The all-zero spec
sentinel is left alone (use `repair_recompute_zero_normals` instead).
The natural pipeline for a freshly-decoded STL scene is:

1. `repair_weld_vertices` — collapse the soup into a shared index buffer.
2. `repair_drop_degenerate_triangles` — cull zero-area triangles.
3. `repair_recompute_zero_normals` — fill in the spec's sentinel zeros.
4. `repair_orient_normals_from_winding` — align stored direction with winding.
5. `repair_normalize_unit_normals` — rescale any non-unit normal to length 1.

## ASCII comment-line tolerance

Whole-line `;`-introduced and `#`-introduced comments are silently
skipped at token boundaries, matching the dominant real-world
tolerance for hand-edited STL files and a handful of vintage CAD
exporters. The 1989 spec defines no comment syntax; we treat both
characters as line-comment introducers and discard the rest of the
line up to the next `\n`. Inline comments mid-token are NOT
tolerated — that would conflict with `vertex` / `facet` / numeral
recognition.

The ASCII-vs-binary sniffer applies the same rule, so a file
starting with `; …\nsolid …` classifies as ASCII end-to-end.

## Trace tape (cross-impl audit)

With the `trace` Cargo feature enabled and `OXIDEAV_STL_TRACE_FILE`
pointing at a writable path, the codec emits one JSON-Lines event per
state transition. **Decode** tapes carry header / triangle_count /
triangle / done; **encode** tapes additionally carry a
`share_stats` event between the final `triangle` and `done` so a
JSONL auditor can pick up the bit-exact `EncodeStats` summary
natively without re-walking the geometry. The field schema, ordering
invariants, and worked examples are documented in
`docs/trace-contract.md`; cross-implementation auditors can lockstep
against that schema without reading source.

## License

MIT — see [LICENSE](LICENSE).

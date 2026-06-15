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
identity-transform single-mesh trees in practice. `Bbox` carries
five geometry accessors for slicer / additive-manufacturing
pipelines that downstream the bbox to a pre-print sanity report:

```rust
use oxideav_stl::{bbox, bbox_of_mesh, bbox_of_primitive};

# let scene = oxideav_mesh3d::Scene3D::new();
if let Some(bb) = bbox(&scene) {
    let _vol = bb.volume();             // dx * dy * dz
    let _sa = bb.surface_area();        // 2*(xy + yz + xz)
    let _diag = bb.diagonal_length();   // sqrt(dx² + dy² + dz²)
    let _axis = bb.longest_axis();      // Some(0|1|2), None on degenerate
    let _inside = bb.contains_point([0.5, 0.5, 0.5]);
}

// Per-mesh / per-primitive scope-narrowed variants — same
// non-finite-skip behaviour as the scene-wide `bbox`. Returns
// `None` for an out-of-range index or a non-`Triangles` primitive.
let _per_mesh = bbox_of_mesh(&scene, 0);
let _per_prim = bbox_of_primitive(&scene, 0, 0);
```

`longest_axis()` returns `Some(0)` for X, `Some(1)` for Y,
`Some(2)` for Z — slicers sweep along the longest axis to
maximise per-layer fill ratio, matching the spec's "Sorting the
triangles in ascending z-value order is recommended … in order to
optimize performance of the slice program" guidance. Ties resolve
toward the lower index (`[1, 1, 0.5]` → `Some(0)`); degenerate
boxes return `None` because no single axis dominates a flat or
empty bbox. `contains_point` is inclusive on every face;
non-finite components reject, matching the silent-skip behaviour
the underlying `bbox` walker uses for non-finite vertex
coordinates. All five accessors are pure getters
(`Bbox` is `Copy`); the scope-narrowed variants are allocation-
free borrows that return `Option<Bbox>` by value.

Three composition helpers round out the bbox API for tooling that
builds a scene-wide bbox out of per-source pieces without juggling
an `Option<Bbox>` in the caller:

```rust
use oxideav_stl::Bbox;

// Degenerate single-point seed. Useful starting accumulator for an
// incremental `merge` chain.
let seed = Bbox::point([0.0, 0.0, 0.0]);

// Component-wise union — the smallest bbox containing every point in
// either input. Commutative + associative, so accumulation order
// doesn't matter.
let a = Bbox { min: [-1.0, 0.0, 0.0], max: [1.0, 2.0, 1.0] };
let b = Bbox { min: [ 0.0, 1.0, 2.0], max: [3.0, 3.0, 5.0] };
let union = a.merge(&b);
assert_eq!(union.min, [-1.0, 0.0, 0.0]);
assert_eq!(union.max, [ 3.0, 3.0, 5.0]);

// Slicer pre-flight safety margin — grow each face by `margin`. A
// negative margin shrinks (caller must re-check `is_degenerate` if
// the magnitude exceeds half an extent on any axis).
let envelope = union.expanded_by(0.5);
assert_eq!(envelope.centre(), union.centre());
```

`merge` is symmetric (`a.merge(&b) == b.merge(&a)`) and idempotent
on a self-merge (`a.merge(&a) == a`); `expanded_by(0.0)` is the
identity. The pattern `Bbox::point(first).merge(&Bbox::point(next))
.merge(&Bbox::point(...))` produces the same hull as the
brute-force walker for any finite vertex stream.

Three more methods round out the AABB lattice for slicer-pre-flight
collision / containment queries:

```rust
use oxideav_stl::Bbox;

let part = Bbox { min: [0.0, 0.0, 0.0], max: [50.0, 30.0, 20.0] };
let build_plate = Bbox { min: [0.0, 0.0, 0.0], max: [250.0, 210.0, 200.0] };

// "Does the part bbox fit inside the build-plate envelope?"
assert!(build_plate.contains_bbox(&part));

// "Does this part overlap with another already placed?"
let other = Bbox { min: [40.0, 0.0, 0.0], max: [90.0, 30.0, 20.0] };
assert!(part.intersects(&other));

// "Compute the overlap region of two clearance envelopes."
let overlap = part.intersect(&other).expect("non-empty overlap");
assert_eq!(overlap.min, [40.0, 0.0, 0.0]);
assert_eq!(overlap.max, [50.0, 30.0, 20.0]);
```

`intersects` is symmetric and self-true on any non-inverted box;
`intersect` is the component-wise dual of `merge` (returns `None`
when separated on any axis, a degenerate box when touching on
exactly one face); `contains_bbox` is reflexive, transitive, and
inclusive on touching faces. Together with `merge` these form the
AABB lattice (union via `merge`, intersection via `intersect`, the
`⊇` order via `contains_bbox`) so any two scene-derived bboxes
compose under the standard set-theoretic operations without
re-walking the geometry.

`Bbox::corners()` returns the eight corner vertices as `[[f32; 3]; 8]`
in a fixed canonical order — the three-bit Cartesian product of
`(min, max)` on each axis with X as the lowest-order bit, so
corner `0` is always `min`, corner `7` is always `max`, opposite
corners sit at indices `i` and `7 - i`, and the lowest-z face is
the first four slots:

```rust
use oxideav_stl::Bbox;

let bb = Bbox { min: [0.0, 0.0, 0.0], max: [10.0, 10.0, 10.0] };
let c = bb.corners();
assert_eq!(c[0], bb.min);
assert_eq!(c[7], bb.max);
// Every corner is inclusively inside the bbox.
for corner in c {
    assert!(bb.contains_point(corner));
}
```

Useful for pipelines that need to test the bbox against a
non-axis-aligned transform (e.g. "does the part still fit on the
build plate after a 30° Z-rotation?"), for visualising the bbox as
a wireframe, or for computing an oriented bbox by transforming
each corner and re-bounding the transformed set. A degenerate
bbox (one or more zero extents) collapses pairs of corners onto
each other but the eight-slot layout is preserved.

`Bbox::from_points` is the matching reduction constructor —
takes any `IntoIterator<Item = [f32; 3]>` and returns the smallest
axis-aligned hull, or `None` if no point contributes a finite
coordinate on any axis:

```rust
use oxideav_stl::Bbox;

// Compute a translated bbox without re-walking the scene: pair
// `corners()` with the per-corner transform and feed the stream
// straight into `from_points`.
let bb = Bbox { min: [0.0, 0.0, 0.0], max: [2.0, 3.0, 4.0] };
let shift = [10.0_f32, 20.0, 30.0];
let translated = Bbox::from_points(
    bb.corners().into_iter().map(|c| [c[0] + shift[0], c[1] + shift[1], c[2] + shift[2]])
).unwrap();
assert_eq!(translated.min, [10.0, 20.0, 30.0]);
assert_eq!(translated.max, [12.0, 23.0, 34.0]);
```

Non-finite components on individual points are silently skipped
per-axis (matching the silent-skip behaviour `bbox` applies to
non-finite vertex coordinates in a `Scene3D`), so a point with two
finite slots and one `NaN` still contributes on the two finite
axes. `from_points(bb.corners())` round-trips to a box equal to
`bb` for any non-degenerate input; `from_points([p])` on a
fully-finite `p` is identical to `Bbox::point(p)`. Single
allocation-free forward pass; `IntoIterator` lets `map`/`filter`
chains feed straight in without an intermediate `Vec`.

`Bbox::translated(delta)` is the typed pure-shift companion — adds
`delta[axis]` to both `min[axis]` and `max[axis]` on every axis,
preserving every shape invariant (extents, volume, surface area,
diagonal length, longest axis, degeneracy). Equivalent to (but
considerably cheaper than) the corner-rebuild pattern above:

```rust
use oxideav_stl::Bbox;

let bb = Bbox { min: [0.0, 0.0, 0.0], max: [2.0, 3.0, 4.0] };
let shift = [10.0_f32, 20.0, 30.0];

// Typed pure-shift — single per-axis add on min + max.
let t = bb.translated(shift);
assert_eq!(t.min, [10.0, 20.0, 30.0]);
assert_eq!(t.max, [12.0, 23.0, 34.0]);
// Centre shifts by the same vector; extents are preserved.
assert_eq!(t.extents(), bb.extents());
```

`translated([0.0; 3])` is the identity; `translated(-d).translated(d)`
round-trips to the original box for any finite `d`. Pairs with
`Bbox::contains_bbox` for slicer pre-flight "would this part still
fit inside the build envelope after the shift?" queries, and with
the `delta` field reported by `repair_translate_to_positive_octant`
to reproduce the post-repair scene bbox without re-walking the
geometry.

`ValidationReport` carries a yes/no `is_clean()` predicate plus two
quantitative summaries for tooling that wants to log or sort scenes
by overall defect count:

```rust
use oxideav_stl::{validate, ValidationOptions};

# let scene = oxideav_mesh3d::Scene3D::new();
let r = validate(&scene, &ValidationOptions::default());

// One scalar headline — sum of every per-rule defect counter.
// `defect_total() == 0` iff `is_clean() == true`.
let total = r.defect_total();

// Labeled per-rule breakdown — seven stable string keys safe to use
// as metric names. Sums exactly to `defect_total()`.
for (rule, count) in r.defects_by_rule() {
    if count > 0 {
        eprintln!("{rule}: {count}");
    }
}
```

Both accessors are pure getters and allocation-free
(`defects_by_rule` returns a fixed-size stack array, not a `Vec`).
Rules whose `ValidationOptions` toggle is off contribute zero to
the total by construction, so the headline is bounded by the rule
set actually run.

### T-junction sub-check

The spec's vertex-to-vertex rule says "a vertex of one triangle
cannot lie on the side (edge) of another triangle". The watertight
edge-use count misses this because the offending vertex is not an
endpoint of the edge it sits on — the geometric incidence has no
matching canonical edge key. `ValidationOptions::check_t_junctions`
(off by default; brute-force `O(E · V_unique)` scan) reports each
distinct `(offending-vertex, edge)` incidence under
`t_junction_defects` and surfaces the *owning* triangle of each
offending vertex on `t_junction_examples` (capped at
`MAX_REPORTED_DEFECTS`). Tolerance defaults to
`DEFAULT_T_JUNCTION_TOLERANCE` (`1e-5`): the vertex must sit within
`eps · |edge|` perpendicular distance of the line AND its projected
parameter must lie in `(eps, 1 - eps)`. Off-by-default because the
scan is expensive on large meshes; turn it on explicitly for
slicer-bound surfaces where producer pipelines might split edges.

### Consistent-winding sub-check

The spec's facet-orientation rule (§6.5) says the three vertices are
"listed in counterclockwise order when looking at the object from the
outside (right-hand rule)" and the two ways orientation is encoded
"must be consistent". `check_facet_orientation` enforces the *per-
facet* form (stored normal vs winding) and the watertight rule counts
*undirected* edge uses — but neither catches a triangle whose winding
is flipped relative to its neighbour. Such a surface can be perfectly
watertight (every edge used twice) while a shared edge is traversed in
the *same* direction by both adjacent triangles, which means one of
them is wound backwards. `ValidationOptions::check_consistent_winding`
(on by default) is the missing mesh-wide invariant: for each manifold
edge (exactly two incident triangles) it checks the two triangles walk
it in *opposite* directions. Same-direction edges land in
`inconsistent_winding_edges`, with both adjacent triangles surfaced on
`inconsistent_winding_examples` (de-duplicated, capped at
`MAX_REPORTED_DEFECTS`). Boundary and non-manifold edges are left to
the watertight rule. Uses bit-exact `f32` position equality, so weld
floating-point-noise corners via `repair_weld_vertices` first.

### Degenerate-triangle rule

A triangle whose three corner *positions* are not pairwise distinct
under bit-exact `f32` equality has no well-defined outward normal
direction — the right-hand-rule clause of §6.5 ("vertices listed in
counterclockwise order when looking at the object from the outside")
cannot be applied. `ValidationOptions::check_degenerate_triangles`
(on by default) surfaces every such face on
`degenerate_triangle_defects` plus a capped illustrative list on
`degenerate_triangle_examples`. The check is `O(N)` (three bit-equality
probes per face) and piggybacks on the main validate loop. Equality
model is identical to `repair_drop_degenerate_triangles`, so the
diagnostic count equals the number of triangles that repair would
drop on the same scene — useful as a pre-flight gauge of how much
geometry would survive the repair pass before running it. Distinct
from `check_unit_normal` (which inspects the *stored* normal vector,
not the geometry), and disjoint from the watertight rule (degenerate
triangles still contribute their three edges to the edge-use map and
can fool watertightness on their own).

### Zero-area-triangle rule

A face whose three corners are pairwise distinct under bit-equality
can still be collinear — three corners that sit on a single straight
line have a zero cross product, so the spec's right-hand-rule clause
cannot pick a unique outward direction either.
`ValidationOptions::check_zero_area_triangles` (on by default)
surfaces every such face on `zero_area_triangle_defects` plus a
capped illustrative list on `zero_area_triangle_examples`. The check
is `O(N)` (one cross-product magnitude probe per face) and piggybacks
on the main validate loop. Tolerance is configurable via
`ValidationOptions::zero_area_tolerance` (defaults to
`DEFAULT_ZERO_AREA_TOLERANCE` = `f32::EPSILON`); negative or
non-finite values clamp to the default so the rule is never silently
disabled. Faces that already trip the corner-coincidence rule
(`check_degenerate_triangles`) are silently skipped here so the two
counts describe disjoint populations and add cleanly into the report
totals. The canonical worked example `(0,0,0)`, `(1,1,1)`, `(2,2,2)`
shows the gap: those three corners are bit-distinct (so the
corner-coincidence rule does not fire) yet they sit on the body
diagonal of the unit cube, so the cross product is `(0, 0, 0)` and
no outward normal exists.

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

### Boundary-loop extraction (naked-edge holes)

`oxideav_stl::boundary_loops(&scene)` is the non-mutating companion to
the watertight diagnostic. `Shell::boundary_edges` and the validate
module's `boundary_edges` field only *count* edges used by exactly one
triangle; `boundary_loops` chains those naked edges into the ordered
cycles they form — each cycle is a hole in the surface that a slicer /
mesh-repair pipeline can cap by triangulating the loop:

```rust
use oxideav_stl::boundary_loops;

# let scene = oxideav_mesh3d::Scene3D::new();
for lp in boundary_loops(&scene) {
    if lp.closed {
        eprintln!("hole with {} boundary edges", lp.edge_count());
        // `lp.vertices` traces the loop in winding order; a cap
        // triangle fan rooted at `lp.vertices[0]` stays consistent
        // with the surrounding surface.
    } else {
        eprintln!("non-manifold open boundary chain ({} edges)", lp.edge_count());
    }
}
```

Each triangle contributes three **directed** edges in winding order;
the single directed instance of a boundary edge carries the surface's
orientation, so a loop walked tail-to-head keeps the surface on one
side — exactly the winding a cap triangle needs. For a closed loop the
first vertex is *not* repeated at the end (the closing edge is implied),
so `vertices.len() == edge_count()`; an open chain (a non-manifold
boundary where three-plus boundary edges meet at a point, e.g. a
bowtie) is emitted with `closed == false` rather than guessing, and
`edge_count() == vertices.len() - 1`. Every boundary edge appears in
exactly one returned loop, so the sum of `edge_count()` across all
loops equals the scene's total boundary-edge count. Loops are returned
sorted by their lexicographically-smallest vertex so the output is
stable across runs regardless of triangle-iteration order. A watertight
scene returns an empty vec. Vertex equality is bit-exact `f32` matching
(weld floating-point-noise corners via `repair_weld_vertices` first).

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
6. `repair_sort_triangles_by_z` — reorder facets into ascending z (see below).
7. `repair_translate_to_positive_octant` — shift the scene into the
   spec's `(+,+,+)` octant if it isn't already there (off by default
   in `ValidationOptions` because modern slicers ignore the rule;
   pair it with `check_positive_octant = true` when targeting a
   strict-1989-spec consumer).
8. `repair_make_winding_consistent` — propagate one canonical
   winding across every manifold-edge-connected component, flipping
   any neighbour whose winding disagrees (matches the validate
   module's `inconsistent_winding_edges` rule, on by default).
9. `repair_split_t_junctions` — split every triangle edge whose
   interior is shared with another triangle's corner (matches the
   validate module's `t_junction_defects` sub-check, off by default
   in `ValidationOptions` because the brute-force scan is expensive
   on large meshes; turn it on together with the opt-in
   `check_t_junctions` rule when interoperating with strict
   vertex-to-vertex consumers).

## Ascending-z facet sort (slicer optimisation)

The 1989 spec says: *"Sorting the triangles in ascending z-value order
is recommended, but not required, in order to optimize performance of
the slice program."* A slicer sweeps a cutting plane upward; presenting
facets in the order their lowest corner enters the sweep lets it stream
triangles instead of re-scanning the whole soup each layer.
`repair_sort_triangles_by_z` materialises that recommendation:

```rust
use oxideav_stl::repair_sort_triangles_by_z;

# let mut scene = oxideav_mesh3d::Scene3D::new();
let report = repair_sort_triangles_by_z(&mut scene);
```

Each triangle is keyed on its three corner z-values sorted ascending —
`(min_z, mid_z, max_z)` — so the primary key is the lowest corner (when
the slice plane first touches the facet), with `mid_z`/`max_z` as
deterministic tie-breakers. Comparison uses `f32::total_cmp`, a total
order over every `f32` (a facet with a NaN minimum z sorts last rather
than scrambling the finite facets). The sort is **stable** — equal-key
facets keep their emit order, so `triangles_reordered == 0` is the
idempotency signal on a re-run. Indexed primitives have only their index
buffer rewritten (the `Indices::U16`/`U32` discriminant and the shared
`positions`/`normals` are preserved); unindexed primitives have their
`positions` (and matched-length `normals`) re-laid-out three corners at
a time. The pass never adds, removes, or mutates a triangle's geometry —
it is a pure count-preserving reordering, and non-`Triangles` primitives
are skipped.

### Ascending-z diagnostic (non-mutating)

When you only need to *know* whether a scene already meets the
recommended order — for example to decide whether a re-sort is worth
paying for, or to *report* spec-recommended-order conformance —
`check_z_sorted` answers without mutating the scene:

```rust
use oxideav_stl::check_z_sorted;

# let scene = oxideav_mesh3d::Scene3D::new();
let report = check_z_sorted(&scene);
if !report.is_sorted() {
    // first descent is `report.first_out_of_order_triangle` (1-based)
}
```

It shares the repair's exact per-triangle z-key and lexicographic
ordering, so the two agree by construction:
`check_z_sorted(scene).is_sorted()` is `true` **iff**
`repair_sort_triangles_by_z` would report `triangles_reordered == 0` on
the same scene. The `ZSortReport` carries `triangles_inspected`,
`out_of_order_pairs` (adjacent within-primitive descents — boundary-
straddling pairs are never counted, matching the repair's per-primitive
scope), and `first_out_of_order_triangle` (1-based global triangle index
of the earliest descent, or `None` when sorted). It is one linear scan
with no permutation, buffer rewrite, or allocation — strictly cheaper
than clone-and-sort when only the yes/no answer is needed.

## Translate-to-positive-octant repair (spec all-positive-octant fix-up)

The 1989 spec says: *"The object represented must be located in the
all-positive octant. In other words, all vertex coordinates must be
positive-definite (nonnegative and nonzero) numbers."*
`ValidationOptions::check_positive_octant` (off by default; modern
slicers ignore the rule) surfaces facets that break this under
`positive_octant_defects`; `repair_translate_to_positive_octant` is the
matching mutating fix-up — it translates every `Triangles` vertex by a
single component-wise delta so the scene's axis-aligned bounding box
sits strictly inside the `(+,+,+)` octant:

```rust
use oxideav_stl::{repair_translate_to_positive_octant, DEFAULT_POSITIVE_OCTANT_MARGIN};

# let mut scene = oxideav_mesh3d::Scene3D::new();
let report = repair_translate_to_positive_octant(&mut scene, DEFAULT_POSITIVE_OCTANT_MARGIN);
```

Per-axis, the translation triggers only when the existing minimum
violates the spec rule (`min[i] <= 0`); axes already strictly positive
are left alone. The `margin` argument's job is to ensure the
post-translation minimum lands *strictly above zero* (not at exactly
zero, which would fail the spec's "nonzero" half); negative or
non-finite margins clamp to `DEFAULT_POSITIVE_OCTANT_MARGIN` (`1e-6`).
The pass is a pure translation — pairwise vertex distances, face
normals, edge connectivity, and the validate-module's facet-
orientation / unit-normal / watertight / consistent-winding rules
are all preserved. `prim.normals` (direction vectors) are left
untouched. Non-finite coordinate components pass through unchanged;
fully-non-finite vertex slots are reported under
`TranslateOctantReport::skipped_non_finite_vertices`. The
idempotency signal is `vertices_translated == 0` and
`delta == [0.0; 3]` on a scene already inside the +octant; a second
pass over the repaired scene is by construction a no-op.

## Make winding consistent (spec mesh-wide invariant)

The 1989 spec's facet-orientation rule (§6.5) says the three vertices
of every triangle are "listed in counterclockwise order when looking
at the object from the outside (right-hand rule)" and that the two
pieces of orientation information "must be consistent".
`repair_orient_normals_from_winding` handles the *per-facet* case
(stored normal vs winding); the *mesh-wide* case — every manifold edge
walked in opposite directions by its two incident triangles — is what
`repair_make_winding_consistent` is for. It is the matching mutating
fix-up for the validate module's `inconsistent_winding_edges` rule
(`ValidationOptions::check_consistent_winding`, on by default):

```rust
use oxideav_stl::repair_make_winding_consistent;

# let mut scene = oxideav_mesh3d::Scene3D::new();
let report = repair_make_winding_consistent(&mut scene);
```

Per-primitive isolation: each `Triangles` primitive's manifold-edge
adjacency is walked independently (no cross-primitive adjacency).
Within a primitive, BFS starts at the first unvisited face — its
winding is canonical, by definition — and propagates outward across
every manifold edge (exactly two incident triangles in the same
primitive). For each BFS edge `(seed_face, neighbour_face)`: if the
two traverse the shared edge in *opposite* directions the neighbour
is already consistent; if they traverse it in the *same* direction
the neighbour is wound backwards and gets flipped. A flip swaps the
second and third vertex slots (in the index buffer for indexed
primitives, in `positions` + matching-length `normals` for unindexed
primitives) — the only data transformation that reverses the
right-hand-rule cross-product direction.

`WindingConsistencyReport { triangles_inspected, triangles_flipped,
components_visited, conflicting_edges }` — `triangles_flipped == 0`
is the idempotency signal. `components_visited` rises with every BFS
seed even when no flip is needed (one increment per
manifold-edge-connected component, not an idempotency signal).
`conflicting_edges` increments when a flip decision would conflict
with one already propagated through a different BFS path — the
non-orientable Möbius-strip-like case where no single global winding
satisfies every edge constraint. Such offenders remain flagged by
`validate`; re-run it after this pass to surface them.

Stored *facet normals* are NOT recomputed here — flipping the
winding changes the cross-product direction, so
`repair_orient_normals_from_winding` is the natural follow-up when
the stored normal must agree with the new winding. The two passes
are independent: this one fixes the mesh-wide invariant, the orient
pass fixes the per-facet invariant.

## Split T-junctions (spec vertex-to-vertex invariant)

The 1989 spec's vertex-to-vertex rule (§6.5) says "every triangle must
share exactly two vertices with each of its adjacent triangles" — i.e.
no corner of one triangle may lie strictly *inside* an edge of another.
`ValidationOptions::check_t_junctions` (off by default; brute-force
`O(E · V_unique)` scan) flags every such incidence under
`t_junction_defects`; `repair_split_t_junctions` is the matching
mutating fix-up:

```rust
use oxideav_stl::{repair_split_t_junctions, DEFAULT_T_JUNCTION_SPLIT_TOLERANCE};

# let mut scene = oxideav_mesh3d::Scene3D::new();
let report = repair_split_t_junctions(&mut scene, DEFAULT_T_JUNCTION_SPLIT_TOLERANCE);
```

Per-`Triangles`-primitive isolation. For each face `(A, B, C)`, the
pass tests every other distinct corner key in the same primitive for
strict on-edge incidence against each of the three edges (under the
same tolerance the validate module uses, so a scene that detects-
and-repairs-and-re-detects at the matching defaults is consistent).
The edge with the most foreign splitters is picked (ties resolve in
cyclic `(A,B) → (B,C) → (C,A)` order); the original face is then
replaced by a fan rooted at the *opposite* corner — for an edge
`(P, Q)` with splitters `V₁ … Vₙ` sorted by their parameter `t ∈
(0, 1)`, the face `(P, Q, R)` becomes the sequence `(P, V₁, R),
(V₁, V₂, R), …, (Vₙ, Q, R)`. The fan preserves the original face's
plane (so its computed normal is identical) and walks every
sub-triangle in the same winding direction.

`TJunctionSplitReport { triangles_inspected, triangles_split,
triangles_emitted, split_vertices_inserted, triangles_unchanged,
skipped_length_mismatch }` — `triangles_split == 0` is the
idempotency signal. Faces with no splitters land in
`triangles_unchanged`; the pass is fully count-balanced
(`final_face_count == pre_face_count - triangles_split +
triangles_emitted`).

Indexed primitives have their splitter positions appended to
`prim.positions` (and `prim.normals` when length-matched) and a new
index buffer emitted. The `Indices::U16` / `Indices::U32`
discriminant is preserved as long as the resulting maximum index
still fits — `U16` auto-widens to `U32` only when a fresh splitter
slot would push the position count past `u16::MAX`. Unindexed
primitives have `prim.positions` (and matched-length `prim.normals`)
fully rewritten as the new flat triangle soup; the per-face normal
slot is replicated from the original face's apex normal because the
fan preserves the plane. Length-mismatched normals arrays — a sign of
a producer bug — count under `skipped_length_mismatch` and skip the
primitive entirely so the pass never invents nonsense face-normal
data.

One pass handles the common producer pattern of "every face carries
at most one T-junction"; nested splits where two new fan triangles
each carry their own splitter need re-runs. Re-running on a scene
that's already passed `validate`'s `check_t_junctions` rule at the
matching `eps` is a no-op. `eps` outside `[0, 0.5)` or non-finite
clamps to `DEFAULT_T_JUNCTION_SPLIT_TOLERANCE` (`1e-5`); cross-
primitive T-junctions are not detected (pre-merge with
`repair_weld_vertices`).

The repair surface now covers every diagnostic the validate module
exposes for the four spec rules AND the two spec sub-checks:
facet-orientation → `repair_orient_normals_from_winding`,
unit-normal → `repair_normalize_unit_normals`,
vertex-to-vertex → `repair_weld_vertices` +
`repair_drop_degenerate_triangles` + `repair_split_t_junctions`,
positive-octant → `repair_translate_to_positive_octant`,
consistent-winding → `repair_make_winding_consistent`.

## Binary-header inspector (pre-decode triage)

`oxideav_stl::inspect_binary_header(bytes)` is a typed,
allocation-free pass over a binary STL byte slice that returns a
`BinaryHeaderReport` *without* building a `Scene3D`. Useful for
pre-decode triage of files whose vendor extensions you may want to
reject (or specifically expect) before paying for the full decode
pass:

```rust
use oxideav_stl::inspect_binary_header;

# fn main() -> Result<(), Box<dyn std::error::Error>> {
let bytes = std::fs::read("part.stl")?;
let rep = inspect_binary_header(&bytes)?;

if !rep.spec_compliant_attributes {
    eprintln!(
        "non-zero attribute slots on {}/{} triangles — vendor extension in play",
        rep.non_zero_attribute_count,
        rep.triangles_walked,
    );
}
if !rep.length_matches_exactly {
    eprintln!(
        "file length {} ≠ expected {:?}",
        rep.actual_byte_length,
        rep.expected_byte_length,
    );
}
# Ok(())
# }
```

The 1989 spec says the per-triangle `uint16` attribute slot "should
be set to zero"; vendors (Materialise / VisCAM / SolidView)
repurpose it for per-face colour packing. The inspector surfaces
the raw header-level facts only — `triangle_count`,
`expected_byte_length` vs `actual_byte_length`,
`non_zero_attribute_count` + matching `_fraction`,
`spec_compliant_attributes`, `triangles_walked` — and never
classifies which convention. (For convention classification see
`oxideav_stl::detect_color_convention`.)

Truncated streams (slice shorter than the declared
`triangle_count * 50 + 84`) are NOT an error here; the inspector
walks as many records as the slice physically contains, sets
`length_matches_exactly = false`, and surfaces the walked count
under `triangles_walked` so a caller can decide whether to recover
the partial information or reject. Slices shorter than the 84-byte
header-plus-count prefix do return `Error::InvalidData`. The full
decode path (`StlDecoder::decode` / `binary::decode`) keeps
rejecting truncated bodies as before — the inspector is the
lightweight triage tool, not a replacement for the decoder.

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

## Strict-spec ASCII lint (pre-decode triage)

`oxideav_stl::lint_ascii(bytes)` walks an ASCII STL byte slice with
exactly the same grammar tolerance as the decoder — it succeeds iff
`ascii::decode` succeeds — but returns a typed `AsciiLintReport`
recording every place the file leans on a tolerance the strict 1989
spec letter does not grant. The ASCII counterpart of
`inspect_binary_header`, for pipelines that must emit (or demand)
letter-strict files while still reading the tolerant real-world
dialect. Seven rules, each grounded in the spec's §6.5.2 prose:

| Rule | Spec basis | Report field |
| ---- | ---------- | ------------ |
| Keyword case | "keywords … must appear in lower case" | `keyword_case_defects` (+ examples) |
| Tab indentation | "Indentation must be with spaces; tabs are not allowed" | `tab_indented_lines` |
| Vertex sign | "A facet normal coordinate may have a leading minus sign; a vertex coordinate may not" | `negative_vertex_coordinate_defects` (+ examples) |
| Comment lines | spec defines no comment syntax | `comment_lines` |
| Multi-`solid` | spec grammar describes one block per file | `extra_solid_blocks` |
| Leading BOM | the format is ASCII | `leading_bom` |
| Empty `solid` | grammar repeats the facet body with `{…}`+ (one or more) | `empty_solid_blocks` |

```rust
use oxideav_stl::lint_ascii;

# fn main() -> Result<(), Box<dyn std::error::Error>> {
# let bytes: &[u8] = b"solid p\nfacet normal 0 0 1\nouter loop\nvertex 1 1 1\nvertex 2 1 1\nvertex 1 2 1\nendloop\nendfacet\nendsolid p\n";
let rep = lint_ascii(bytes)?;
if !rep.is_strict_spec() {
    for (rule, count) in rep.findings_by_rule() {
        if count > 0 {
            eprintln!("{rule}: {count}");
        }
    }
    for f in &rep.keyword_case_examples {
        eprintln!("line {}: `{}` is not lower-case", f.line, f.token);
    }
}
# Ok(())
# }
```

Counts are always complete; the keyword-case and vertex-sign rules
additionally carry capped (`MAX_REPORTED_LINT_FINDINGS` = 32)
illustrative `AsciiLintFinding { line, token }` lists with 1-based
line numbers and verbatim tokens. `is_strict_spec()` /
`finding_total()` / `findings_by_rule()` mirror the validate module's
report ergonomics. The vertex-sign rule is the lexical face of the
geometric all-positive-octant rule: the crate's own ASCII encoder
lints fully strict on positive-octant single-mesh scenes, and running
`repair_translate_to_positive_octant` before re-emit clears the one
geometry-driven finding. Normal coordinates are never flagged — the
spec explicitly permits their leading minus.

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

## Fuzzing

`fuzz/` carries four cargo-fuzz targets driven by a daily GitHub
Actions schedule (`.github/workflows/fuzz.yml`, 1800-second budget):

- `decode` — feeds arbitrary attacker-controlled bytes through
  `StlDecoder::decode` and asserts the call always returns a `Result`
  rather than panicking / aborting / OOMing. Exercises the detector
  + both parsers (binary `uint32` triangle-count slot, ASCII keyword
  walk, multi-`solid` mesh proliferation, Materialise / VisCAM
  per-face colour distribution, `84 + N*50` size override, UTF-8 BOM
  + leading-whitespace skip).
- `roundtrip` — synthesises a small binary STL from fuzz-controlled
  bytes, decodes it, re-encodes it, and asserts the triangle-count
  slot + every per-triangle record survive byte-for-byte (the 80-byte
  header is allowed to differ — writers substitute their own
  signature). Mirrors the
  `binary_cube_triangle_records_roundtrip_byte_identical` integration
  test as a coverage-guided invariant.
- `triage` — drives the three pre-decode inspectors on arbitrary
  bytes: `lint_ascii` (strict-1989-spec ASCII conformance walk),
  `inspect_binary_header` (header + per-face attribute-slot walk with
  no `count * 50` pre-allocation), and `detect_color_convention`
  (VisCAM / Materialise 15-bit-RGB classification of an attribute-byte
  buffer). Each inspector hand-rolls its **own** byte scanner — none
  route through `StlDecoder` — so this target covers parser surfaces
  the other two never touch: the lint token loop + tab/sign/BOM scan +
  `MAX_REPORTED_LINT_FINDINGS` example cap, the inspector's
  hostile-count `min` cap + `triangle_count == 0` NaN-fraction branch,
  and the colour detector's `chunks_exact(2)` dangling-byte remainder.
  Panic-freedom is the whole invariant. A 60-second local sweep
  (≈14.8 M executions) found zero crashes; the corpus is checked in
  minimised to 363 coverage-preserving seeds.
- `repair` — builds a hostile `Scene3D` **directly** from fuzz bytes
  (no decode step) — multiple meshes, multiple primitives, every
  `Topology` variant, indexed and unindexed primitives, `U16` / `U32`
  index buffers with deliberately out-of-range entries, `normals`
  arrays whose length disagrees with `positions`, and coordinate
  bit-patterns that decode to NaN / ±Inf / subnormal — then drives the
  `validate` + `topology` (repair) surface that the byte-parsing
  targets never reach: `validate` with every rule on (including the
  opt-in `check_t_junctions` / `check_positive_octant` brute-force
  scans), `bbox` / `bbox_of_mesh` / `bbox_of_primitive`, `shells`, and
  every mutating repair pass — individually on per-pass clones and as
  the full documented nine-step pipeline on one scene, re-validated at
  the end. Each pass takes a caller-controlled scene and must return
  its report rather than panic / index past a buffer / overflow on a
  bad index / divide by zero. A 60-second local sweep (≈480 K
  executions) found zero crashes; the corpus is checked in minimised
  to coverage-preserving seeds.

Run locally with `cargo +nightly fuzz run decode` /
`cargo +nightly fuzz run roundtrip` / `cargo +nightly fuzz run
triage` / `cargo +nightly fuzz run repair` from `crates/oxideav-stl/`.

## Benchmarks

`benches/` carries four Criterion suites driven entirely by
deterministic xorshift32 fixtures (no committed binary corpora, no
`docs/` traffic — each generator is seeded from a fixed constant so
results are bit-stable across hosts and runs):

- `decode` — `decode_binary` (1 K / 10 K / 100 K triangles) and
  `decode_ascii` (1 K / 5 K / 10 K). Throughput reported in bytes/s
  so the binary and ASCII paths compare directly.
- `encode` — `encode_binary` (1 K / 10 K / 100 K) and `encode_ascii`
  (1 K / 5 K / 10 K) against pre-built `Scene3D` inputs. Binary
  throughput in bytes-out/s; ASCII in latency only because the
  output length depends on the formatter's per-coordinate width.
- `dedup` — `stats_bit_exact`, `dedup_brute_eps_1e-5`,
  `dedup_spatial_eps_1e-5` at matched element counts so the
  brute-force `O(N · K)` vs spatial-grid `O(N)` crossover is
  visible in a single comparison.
- `validate` — `validate_default_opts` at 1 K / 10 K / 100 K
  triangles (default-on rules: facet orientation, unit normal,
  watertight/manifold, consistent winding) and
  `validate_t_junctions_on` at 100 / 300 / 1 000 (the opt-in
  brute-force T-junction sub-check) so the diagnostic-only
  warning on the T-junction rule is empirically substantiated.

Run with `cargo bench -p oxideav-stl --bench <name>` or `--quick
--noplot` for a fast headline sweep. Indicative numbers
on an Apple M-series host (`cargo bench --quick`, optimised
profile): binary decode ~7.6 GiB/s at 100 K triangles; ASCII decode
~720 MiB/s; binary encode ~7.9 GiB/s at 10 K triangles; ASCII encode
9.2 ms/10 K triangles; default-rule
`validate` ~2.6–3.8 Melem/s; T-junction sub-check ~50–410 Kelem/s
(a 70× brake vs the default rules at matched N — the
diagnostic-only cost noted on that rule above).

## Profiling

`examples/` carries six deterministic, single-threaded long-running
drivers wrapping each hot path so a profiler can attribute cycles
line-by-line without Criterion's adaptive batching getting in the
way. Every driver runs against a fixed xorshift32-derived fixture
and prints a single sanity line at the end, so two runs against the
same release build are byte-identical inputs and the resulting
flame graphs diff cleanly across optimisation attempts.

```text
cargo flamegraph --release --example profile_encode_binary
samply record cargo run --release --example profile_decode_ascii
perf record -g cargo run --release --example profile_validate
# (Instruments' Time Profiler on macOS / Xcode equivalent on iOS)
```

The six targets:

- `profile_encode_binary` (2 000 × 10 K triangles) — pack-record /
  LE-float-byte emit hot loop in `binary::encode`.
- `profile_decode_binary` (2 000 × 10 K triangles) — 50-byte parse
  loop + `Scene3D` construction in `binary::decode`.
- `profile_encode_ascii` (200 × 5 K triangles) — per-coordinate
  float-to-string formatting + keyword emit.
- `profile_decode_ascii` (200 × 5 K triangles) — token walk +
  per-coordinate float lex.
- `profile_dedup_spatial` (50 × 50 K triangles, ε = 1e-5) —
  uniform-grid cell insertion + 27-cell scan in
  `EncodeStats::with_tolerance_spatial`.
- `profile_validate` (200 × 10 K triangles) — default-on rule set
  (facet orientation + unit normal + watertight/manifold +
  consistent winding).

Shared fixture builders live in `examples/profile_common/mod.rs`
and are pulled in by each driver via `#[path =
"profile_common/mod.rs"] mod profile_common;` — the same pattern
the bench suite uses for its `benches/common/mod.rs` helpers.

## License

MIT — see [LICENSE](LICENSE).

# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- `topology::boundary_loops` — non-mutating extraction of ordered
  naked-edge (boundary) loops. Where `Shell::boundary_edges` and the
  validate module merely *count* edges used by a single triangle, this
  chains them into the ordered cycles they form (each a hole in the
  surface), carrying the winding-consistent vertex order a cap triangle
  needs plus an `open`/`closed` flag for non-manifold boundaries. The
  total edge count across all loops equals the scene's boundary-edge
  count. Localises the spec's "closed boundary between interior and
  exterior" invariant breakage as discrete holes. Added to the `repair`
  fuzz target's panic-freedom surface.

## [0.0.4](https://github.com/OxideAV/oxideav-stl/compare/v0.0.3...v0.0.4) - 2026-06-15

### Other

- add check_z_sorted non-mutating ascending-z diagnostic
- Round 304 — lint_ascii rule 7: empty-solid-block strict-spec detection
- add `repair` target driving the validate + topology repair surface
- add `triage` target hardening the three pre-decode inspectors
- Round 280 — lint_ascii strict-spec ASCII conformance lint
- Round 273 — Bbox::scaled_about_centre per-axis in-place resize
- Round 266 — Bbox::translated pure-shift typed helper
- Round 257 — Bbox::from_points reduction constructor
- drop release-plz.toml — use release-plz defaults across the workspace
- Round 245 — check_zero_area_triangles validation rule
- Round 242 — check_degenerate_triangles validation rule
- Round 239 — Bbox::corners() canonical eight-vertex AABB enumeration
- Round 236 — inspect_binary_header: typed pre-decode header triage
- Round 231 — Bbox::intersects + Bbox::intersect + Bbox::contains_bbox AABB-lattice methods
- Round 225 — Bbox::point + Bbox::merge + Bbox::expanded_by composition helpers
- Round 219 — Bbox geometry accessors + per-mesh / per-primitive bbox
- Round 216 — ValidationReport::defect_total + defects_by_rule accessors
- Round 210 — repair_split_t_junctions (spec vertex-to-vertex fix-up)
- Round 205 — repair_make_winding_consistent (spec mesh-wide winding fix-up)
- Round 199 — repair_translate_to_positive_octant (spec all-positive-octant fix-up)

### Added

- Round 311 — `check_z_sorted(scene) -> ZSortReport`: non-mutating
  diagnostic counterpart of `repair_sort_triangles_by_z`. The 1989 spec
  notes "Sorting the triangles in ascending z-value order is
  recommended, but not required, in order to optimize performance of the
  slice program" (`docs/3d/stl/fabbers-stl-format.html` §6.5, Format
  Specifications); the repair *materialises* that recommendation, but
  pipelines that only need to *decide* whether to pay for a re-sort (or
  to *report* spec-recommended-order conformance) previously had to
  clone-and-sort to find out. The new diagnostic answers the yes/no
  question plus the first offending position in a single linear scan —
  no permutation, no buffer rewrite, no allocation. It shares the
  repair's exact per-triangle z-key (`(min_z, mid_z, max_z)` compared
  lexicographically with `f32::total_cmp`, malformed-face corners
  contributing the same `f32::NAN` high sentinel), so the two agree by
  construction: `check_z_sorted(scene).is_sorted()` is `true` iff
  `repair_sort_triangles_by_z` would report `triangles_reordered == 0`
  on the same scene (verified by an acceptance-parity sweep). The
  `ZSortReport` carries `triangles_inspected`, `out_of_order_pairs`
  (adjacent within-primitive descents — boundary-straddling pairs are
  never counted, mirroring the repair's per-primitive scope), and
  `first_out_of_order_triangle` (1-based global triangle index of the
  earliest descent, or `None` when sorted), with an `is_sorted()`
  convenience. Non-`Triangles` primitives are skipped; an empty scene is
  trivially sorted. Re-exported from the crate root alongside the repair
  family. 8 new unit tests + 6 integration tests
  (`tests/check_z_sorted.rs`, driven through the public decoder).

- Round 304 — seventh strict-spec ASCII lint rule: empty-`solid`-block
  detection. The spec's ASCII grammar repeats the facet body with the
  `{…}`+ notation, where the `+` means "one or more times", so a
  `solid … endsolid` block carrying zero facets violates the strict
  letter even though `ascii::decode` accepts it (yielding an empty
  mesh). `lint_ascii` now counts each such block under
  `AsciiLintReport::empty_solid_blocks`; the count folds into
  `finding_total()` / `is_strict_spec()` and adds a stable
  `"empty_solid_block"` key to `findings_by_rule()` (now 7 entries).
  Independent of the multi-`solid` rule (rule 5), so two empty blocks
  contribute one extra-block finding plus two empty-block findings.
  Spec basis: `docs/3d/stl/fabbers-stl-format.html` §6.5.2 ASCII
  grammar `{…}`+ repetition notation.

- Round 289 — third cargo-fuzz target `triage` (`fuzz/fuzz_targets/
  triage.rs`). The existing `decode` / `roundtrip` targets only drive
  `StlDecoder::decode`, but the three public pre-decode inspectors —
  `lint_ascii`, `inspect_binary_header`, and `detect_color_convention`
  — each hand-roll their own byte scanner and never route through the
  decoder, so they were an untested malformed-input surface. The new
  target feeds arbitrary attacker-controlled bytes through all three
  and asserts each call returns rather than panicking / indexing past
  the slice / dividing by a live zero, hitting the lint token loop +
  tab/sign/BOM scan + `MAX_REPORTED_LINT_FINDINGS` example cap, the
  inspector's hostile-`u32`-count `min` cap + `triangle_count == 0`
  NaN-fraction branch, and the colour detector's `chunks_exact(2)`
  dangling-byte remainder. A 60-second local sweep (≈14.8 M
  executions) found zero crashes; the checked-in corpus is minimised
  to 363 coverage-preserving seeds. Scheduled automatically by the
  daily `.github/workflows/fuzz.yml` reusable harness (now splitting
  its 1800-second budget across three targets). No `src/` change — the
  inspectors were already panic-free; this hardens the contract with
  coverage-guided assurance.

- Round 280 — `lint_ascii` strict-spec ASCII conformance lint (new
  `lint` module): walks an ASCII STL byte slice with exactly the same
  grammar tolerance as `ascii::decode` (acceptance parity by
  construction — the lint succeeds iff the decoder succeeds) and
  returns a typed `AsciiLintReport` recording every place the file
  leans on a tolerance the strict 1989 spec letter does not grant.
  The ASCII counterpart of `inspect_binary_header`: pre-decode triage
  for pipelines that must emit (or demand) letter-strict files while
  still reading the tolerant real-world dialect. Six rules, each
  grounded in the staged transcription's §6.5.2 prose
  (`docs/3d/stl/fabbers-stl-format.html`): keyword tokens not written
  entirely in lower case ("Bold face indicates a keyword; these must
  appear in lower case"); lines whose leading whitespace contains a
  tab ("Indentation must be with spaces; tabs are not allowed");
  `vertex` coordinate tokens with a lexical leading `-` ("A facet
  normal coordinate may have a leading minus sign; a vertex
  coordinate may not" — normal coordinates are never flagged);
  `;`/`#` whole-line comments (the spec defines no comment syntax);
  `solid` blocks beyond the first (the spec grammar describes one
  block per file); and a leading UTF-8 BOM (a non-ASCII prefix).
  Counts are always complete; the keyword-case and
  negative-vertex-coordinate rules also carry capped
  (`MAX_REPORTED_LINT_FINDINGS` = 32, matching
  `validate::MAX_REPORTED_DEFECTS`) illustrative example lists of
  `AsciiLintFinding { line, token }` with 1-based line numbers and
  verbatim tokens. `is_strict_spec()` / `finding_total()` /
  `findings_by_rule()` mirror the validate module's report
  ergonomics (six stable string labels, sums exactly to the total).
  The negative-vertex rule is the lexical face of the geometric
  all-positive-octant rule: the crate's own ASCII encoder lints
  fully strict on positive-octant geometry, and pairing with
  `repair_translate_to_positive_octant` before re-emit clears the
  one geometry-driven finding (covered by an integration test).
  10 new integration tests (`tests/ascii_lint.rs`) + 5 unit tests,
  including a 14-case accept/reject acceptance-parity corpus
  against `ascii::decode`.

- Round 273 — `Bbox::scaled_about_centre` per-axis in-place resize: each
  extent is multiplied by `factor[axis]` while `Bbox::centre` is held
  fixed. The multiplicative companion to `Bbox::translated` (a rigid
  shift) for the other in-place affine slicer pre-flight operation —
  "resize this part by `f` in place, does it still fit the build
  envelope / clear the already-placed parts?" Uniform factor for a
  proportional resize (`[1.1; 3]` → 110 %), distinct per-axis factors
  for an anisotropic stretch (axis-dependent shrink-compensation).
  Distinct from `expanded_by`, which adds a fixed *absolute* margin to
  every face regardless of extent: `scaled_about_centre` is
  multiplicative, so a part twice as long on an axis grows twice as much
  in absolute terms there. `scaled_about_centre([1.0; 3])` is the
  identity; `[0.0; 3]` collapses the box to its centre point (degenerate
  on every axis); a negative component mirrors that axis into an
  inverted (`min > max`) box for the caller to re-normalise. Composition
  is multiplicative per axis
  (`scaled_about_centre(a).scaled_about_centre(b)` ==
  `scaled_about_centre([a*b]...)`) and the reciprocal `1.0 / f`
  round-trips to the original box for any finite non-zero `f`. Scales
  the half-extent about the exact `(min + max) * 0.5` centre so any
  centre drift is bounded by the centre's own rounding error (zero for
  integer-bounded boxes). Equivalent to — but considerably cheaper than
  — scaling each `corners()` vertex about the centre and feeding the
  result back through `from_points`. Non-finite `factor` components
  propagate; finite inputs on a finite box stay finite.

- Round 257 — `Bbox::from_points` reduction constructor: takes any
  `IntoIterator<Item = [f32; 3]>` and returns
  `Option<Bbox>` — the smallest axis-aligned hull that contains every
  finite point in the stream, or `None` if no point contributes a
  finite coordinate on any axis. Non-finite components on individual
  points are silently skipped per-axis (matching the silent-skip
  behaviour `bbox` applies to non-finite vertex coordinates in a
  `Scene3D`), so a point with two finite slots and one `NaN` still
  contributes on the two finite axes. The missing primitive that lets
  callers build a `Bbox` without going through the `Scene3D` walker —
  pairs naturally with `Bbox::corners()` for "compute the bbox after a
  non-axis-aligned transform" (translated / rotated parts) by feeding
  the transformed corner stream straight into the constructor in a
  single forward pass. `from_points([p])` on a fully-finite `p`
  produces the same bbox as `Bbox::point(p)`;
  `from_points(bb.corners())` round-trips to a box equal to `bb` for
  any non-degenerate input. Allocation-free; `IntoIterator` lets
  `map`/`filter` chains feed straight in without an intermediate
  `Vec`. Internally a single forward pass over the same
  `BboxAccumulator` that powers the `bbox` / `bbox_of_mesh` /
  `bbox_of_primitive` scene walkers, so the silent-skip semantics
  cannot diverge.

- Round 245 — `ValidationOptions::check_zero_area_triangles` (on by
  default) + `ValidationOptions::zero_area_tolerance` knob, matching
  `ValidationReport::zero_area_triangle_defects` count and
  `zero_area_triangle_examples` capped illustrative list, and the
  new `DEFAULT_ZERO_AREA_TOLERANCE` constant (`f32::EPSILON`).
  Detects triangles whose three corners are pairwise distinct under
  bit-equality yet sit on a single straight line — the cross product
  of two edge vectors vanishes, so §6.5's right-hand-rule clause
  cannot pick a unique outward direction. Disjoint from
  `check_degenerate_triangles` (which fires on bit-equal corners): a
  face that already trips the corner-coincidence rule is silently
  skipped here so the two counts describe non-overlapping populations
  and add cleanly into the report totals. The check is `O(N)` (single
  forward pass, one cross-product magnitude probe per face) and
  piggybacks on the main validate loop. Brings the validation rule
  set from eight to nine; `is_clean()`, `defect_total()`, and
  `defects_by_rule()` pick up the new field, the latter now returning
  a nine-entry array with the new `"zero_area_triangle"` label pinned
  at the tail. Non-finite corners propagate through to a NaN cross
  product and are treated as zero-area (mirroring `recompute_normal`'s
  sentinel-return behaviour). Negative / non-finite tolerances clamp
  to the default so the rule cannot be silently disabled.

- Round 242 — `ValidationOptions::check_degenerate_triangles` (on by
  default) + matching `ValidationReport::degenerate_triangle_defects`
  count and `degenerate_triangle_examples` capped illustrative list.
  Detects triangles whose three corner *positions* are not pairwise
  distinct under bit-exact `f32` equality — a face whose corners
  collide has no defined outward normal direction and therefore cannot
  satisfy §6.5's right-hand-rule clause. The diagnostic counterpart to
  `repair_drop_degenerate_triangles`: same bit-equality model, non-
  mutating; the count produced by `validate` equals the number of
  triangles the repair pass would drop on the same scene. The check is
  `O(N)` (single forward pass, three bit-equality probes per face) and
  piggybacks on the main `validate` loop. Brings the validation rule
  set from seven to eight; `is_clean()`, `defect_total()`, and
  `defects_by_rule()` pick up the new field, the latter now returning
  an eight-entry array with the new `"degenerate_triangle"` label
  pinned at the tail. Distinct from `check_unit_normal` (which inspects
  the *stored* normal, not the geometry), and disjoint from the
  watertight rule (degenerate triangles still contribute their three
  edges to the undirected edge-use map).

- Round 239 — `Bbox::corners()` returns the eight corner vertices of
  the bounding box as `[[f32; 3]; 8]` in a fixed canonical order
  derived from the three-bit Cartesian product of `(min, max)` on each
  axis with X as the lowest-order bit. Corner `0` is always `Bbox::min`
  and corner `7` is always `Bbox::max`; opposite corners sit at indices
  `i` and `7 - i`; the lowest-z face is `[0, 1, 2, 3]` and the highest-z
  face is `[4, 5, 6, 7]`. Useful for pipelines that need to test the
  bbox against a non-axis-aligned transform (rotated build-plate fit),
  for visualising the bbox as a wireframe, or for computing an oriented
  bbox by transforming each corner and re-bounding the transformed set.
  Every returned corner satisfies `Bbox::contains_point` on the
  originating bbox (inclusive on every face); a degenerate bbox collapses
  pairs of corners onto each other but preserves the eight-slot layout.

- Round 236 — `oxideav_stl::inspect_binary_header(bytes)` typed
  byte-stream-level inspector that returns a
  `BinaryHeaderReport { triangle_count, expected_byte_length,
  actual_byte_length, length_matches_exactly,
  non_zero_attribute_count, non_zero_attribute_fraction,
  spec_compliant_attributes, triangles_walked }` *without* building
  a `Scene3D`. Pre-decode triage hook: the 1989 spec says the
  per-triangle `uint16` attribute slot "should be set to zero", and
  the inspector surfaces the raw header-level facts so consumers can
  decide pre-decode whether to expect vendor-extension payloads (or
  reject strict-spec inputs that carry them). Allocation-free single
  forward pass — never copies the triangle records and never
  allocates intermediate `Vec`s. Truncated streams (slice shorter
  than `84 + N * 50` for the declared `N`) are not an error here;
  the inspector walks every record the slice physically contains,
  sets `length_matches_exactly = false`, and surfaces the walked
  count under `triangles_walked`. Slices shorter than the 84-byte
  header-plus-count prefix return `Error::InvalidData` since no
  valid binary STL is shorter. Distinct from
  `oxideav_stl::detect_color_convention` (which classifies a *bit
  distribution* across vendor conventions) and from `validate`
  (which operates on a decoded `Scene3D`); this inspector reports
  the raw header-level facts only. `BinaryHeaderReport` is `Copy`
  so the caller can stash it in a log/scoreboard without lifetime
  juggling.

- Round 231 — `Bbox::intersects` / `Bbox::intersect` /
  `Bbox::contains_bbox` AABB-lattice methods on
  `oxideav_stl::validate`. The round-225 composition helpers
  (`point`/`merge`/`expanded_by`) covered the union half of the
  lattice; the three new methods cover the intersection +
  containment half so any two scene-derived bboxes compose under
  the standard set-theoretic operations without re-walking the
  geometry. Slicer-pre-flight use cases pick up three queries:
  "does this part fit inside the build-plate envelope" →
  `build_plate.contains_bbox(&part)`; "does this part overlap with
  another already placed" → `a.intersects(&b)`; "compute the
  overlap region of two clearance envelopes" → `a.intersect(&b)`.
  `intersects(other)` returns `bool` — inclusive on every face
  (boxes touching on exactly one face share that face and count as
  intersecting), symmetric, and self-true on any non-inverted box.
  `intersect(other) -> Option<Bbox>` returns the largest box
  contained in both inputs; `None` when the inputs do not overlap
  (`!self.intersects(other)`); otherwise `min ==
  max(self.min, other.min)` and `max == min(self.max, other.max)`
  component-wise. The result may be degenerate on any axis whose
  `self.min == other.max` (or the dual) — touching on exactly one
  face produces a flat (zero-extent) intersection. Symmetric and
  idempotent (`a.intersect(&a) == Some(a)` for any non-inverted
  box); component-wise dual of `merge`.
  `contains_bbox(other) -> bool` returns whether `other` lies
  entirely inside `self` (inclusive on every face). Reflexive
  (`a.contains_bbox(&a) == true`) and transitive
  (`a ⊇ b && b ⊇ c → a ⊇ c`); a degenerate `other` (zero extents
  on some axis) is still "contained" as long as its single-point
  face lies within `self`'s closed range. Pure getters,
  allocation-free (`Bbox` stays `Copy`; the methods take `&Bbox`
  borrows and return `bool` / `Option<Bbox>` by value). Lattice
  invariants pinned as a unit test: for any `a`, `b`,
  `a.merge(&b)` contains both inputs; `a.intersect(&b)`, when
  present, is contained by both inputs. 13 new unit tests
  (intersects symmetry + self-true; axis-by-axis separation
  rejection; inclusive-on-touching-face; intersect overlap region
  + symmetry + dual-of-merge invariants; separated returns None;
  self-idempotence; touching-face produces a flat degenerate box;
  fully-contained returns the inner box; contains_bbox reflexive
  + inclusive; rejects overhanging / separated / partial overlap;
  transitive a ⊇ b ⊇ c; accepts degenerate inner point on a face
  but rejects outside-point; lattice cross-check tying merge,
  intersect, contains_bbox together) and 3 new integration tests
  (build-plate envelope contains decoded brick under binary
  roundtrip; per-mesh bboxes report intersect/non-intersect when
  meshes overlap or are separated in space; intersect of per-mesh
  bbox with scene-wide bbox collapses to the per-mesh bbox).
  Lib-test count rises by 13 (189 → 202); integration-test count
  rises by 3 (9 → 12 in `bbox_geometry.rs`). No new public type;
  no behavioural change to `bbox` / `bbox_of_mesh` /
  `bbox_of_primitive` / any other existing accessor. Standalone
  (`--no-default-features --lib`) build unchanged.

- Round 225 — `Bbox::point` / `Bbox::merge` / `Bbox::expanded_by`
  composition helpers on `oxideav_stl::validate`. The round-219
  geometry accessors (`volume` / `surface_area` / `diagonal_length`
  / `longest_axis` / `contains_point`) cover one-bbox queries; the
  three new methods cover the two-bbox + transform cases tooling
  reaches for when assembling a scene-wide envelope out of per-mesh
  / per-source pieces (multi-source slicer pre-flight where each
  input reports its own bbox; clearance-aware build-plate checks
  that need a kerf / raft margin around the print volume). The
  three helpers: `Bbox::point(p)` returns a degenerate single-point
  bbox seed (`min == max == p`); `Bbox::merge(&other)` returns the
  component-wise union (the smallest box containing every point in
  either input — commutative, associative, self-merge identity);
  `Bbox::expanded_by(margin)` returns a box grown by `margin` on
  every face (each axis: `min - margin`, `max + margin`).
  Symmetric expansion preserves the centre; a `0.0` margin is the
  identity; negative margins shrink and may produce a degenerate
  or inverted result on any axis whose magnitude is exceeded — the
  caller is responsible for re-checking `is_degenerate` afterwards
  (documented contract). Pure getters, allocation-free (`Bbox`
  stays `Copy`; the helpers take/return `Bbox` by value). The
  composition pattern `Bbox::point(first).merge(&Bbox::point(next))
  .merge(...)` produces the same hull as the brute-force
  `bbox(&scene)` walker for any finite vertex stream; the merge of
  per-mesh `bbox_of_mesh` reports equals the scene-wide `bbox` —
  pinned as integration tests. 7 new unit tests (point-seed
  degeneracy + zero-everything accessors; merge commutativity +
  associativity + self-identity; point-merge accumulation matches
  a vertex swarm; expanded-by margin sum on every axis +
  centre-preservation + identity on zero; negative margin shrinks
  but stays non-degenerate; negative excess inverts the box) and
  3 new integration tests (per-mesh merge equals scene-wide bbox
  on a multi-mesh scene; expanded envelope contains every emitted
  vertex of a binary-roundtrip-decoded scene; point-merge
  accumulator matches the brute-force walker on the brick
  fixture). Lib-test count rises by 7 (182 → 189);
  integration-test count rises by 3 (6 → 9 in `bbox_geometry.rs`).
  No new public type; no behavioural change to `bbox` /
  `bbox_of_mesh` / `bbox_of_primitive` / any other existing
  accessor. Standalone (`--no-default-features --lib`) build
  unchanged.

- Round 219 — `Bbox` geometry accessors + per-mesh / per-primitive
  bbox variants on `oxideav_stl::validate`. The existing `Bbox`
  type (`min`/`max` plus `extents`/`centre`/`is_degenerate`)
  surfaces a scene-wide axis-aligned bounding box but lacks the
  derived scalars slicer / additive-manufacturing pipelines reach
  for: total volume estimate, bounding-surface area, space-
  diagonal length, and which axis dominates (sweeping the longest
  axis maximises per-layer fill ratio, matching the spec's
  "Sorting the triangles in ascending z-value order is recommended
  ... in order to optimize performance of the slice program"
  guidance). Five new pure-getter methods land on `Bbox`:
  `volume()` (product of the three extents; `0.0` on a degenerate
  box, matching `is_degenerate`), `surface_area()`
  (`2 * (xy + yz + xz)`; partially-degenerate boxes still report a
  positive area drawn from the two non-degenerate axes),
  `diagonal_length()` (`sqrt(dx² + dy² + dz²)` — one scalar "scene
  size" headline), `longest_axis()` (returns `Some(0)` for X,
  `Some(1)` for Y, `Some(2)` for Z; ties resolve toward the lower
  index; `None` for a degenerate box because no single axis
  dominates a flat or empty bbox), and `contains_point(p)` (each
  axis tested inclusive against `min`/`max`; non-finite
  components reject, matching the spec-style silent-skip
  behaviour `bbox` uses for non-finite vertex coordinates). Two
  new scope-narrowed entry points sit alongside the existing
  scene-wide `bbox`: `bbox_of_mesh(scene, mesh_idx) -> Option<Bbox>`
  (only that mesh's `Triangles` primitives contribute) and
  `bbox_of_primitive(scene, mesh_idx, prim_idx) -> Option<Bbox>`
  (only that one primitive, returning `None` for non-`Triangles`
  topology or any out-of-range index). Both share a single
  `BboxAccumulator` walker with the scene-wide path so the
  min/max accumulation logic stays unified. The whole suite is
  allocation-free (`Bbox` is `Copy`, the accessors take `&self`,
  the per-scope variants borrow `&Scene3D` and return `Option<Bbox>`
  by value); no `Mesh3DRegistry` plumbing changes; no public-
  type breakage. 15 new tests cover the geometry math (unit cube
  volume + surface area + diagonal; 2x3x4 brick survives a
  binary roundtrip with bit-identical extents), the longest-axis
  tie-break + degenerate-box short-circuit, `contains_point`
  inclusive-boundary + non-finite rejection, and the per-mesh /
  per-primitive isolation against a hand-built multi-mesh and
  multi-`solid` ASCII flavour. 188 lib tests pass (was 173).

- Round 216 — `ValidationReport::defect_total()` +
  `ValidationReport::defects_by_rule()` quantitative summary
  accessors on `oxideav_stl::validate`. The existing `is_clean()`
  predicate answers a yes/no question about scene validity; the
  new pair answer the matching quantitative questions. The 1989
  spec's facet-orientation / unit-normal / positive-octant /
  watertight (boundary-edge + non-manifold-edge) rules plus the
  two sub-checks (T-junction + consistent-winding) populate seven
  separate counter fields on `ValidationReport`. Tooling that
  wants to log or sort scenes by overall defect count had to sum
  the fields by hand; the same tooling for per-rule labeled
  reporting had to hand-roll seven `if count > 0` arms. Both
  paths are now one method call. `defect_total() -> usize`
  returns the arithmetic sum across all seven counters (rules
  whose `ValidationOptions` toggle is off contribute zero, so the
  number is bounded by the rule set actually run);
  `defect_total() == 0` iff `is_clean() == true`, by construction.
  `defects_by_rule() -> [(&'static str, usize); 7]` returns the
  per-rule labeled breakdown in `validate` scan order
  (`"facet_orientation"`, `"non_unit_normal"`, `"positive_octant"`,
  `"boundary_edges"`, `"non_manifold_edges"`, `"t_junction"`,
  `"inconsistent_winding"`) — the labels are stable strings safe
  to use as metric names or log keys, and the seven counts sum
  exactly to `defect_total()`. Pure-getter, allocation-free
  (`defect_total` is constant-time; `defects_by_rule` returns a
  fixed-size stack-resident array — no `Vec`, no `String`, no
  heap traffic). No `Display` impl on the report; callers compose
  their own formatting against the labeled rows. 7 new unit tests
  (empty-scene zero, open-triangle boundary-edge count, mixed
  orientation+boundary sum, every-rule-disabled vacuous total,
  scan-order label list + sum-matches-total invariant,
  clean-scene all-zero rows, positive-octant row picks up
  opt-in rule) cover idempotency between `is_clean()` and
  `defect_total() == 0`. Lib-test count rises by 7 (166 → 173);
  surface is purely additive on `ValidationReport` — no rule
  toggle, no new tolerance constant, no behavioural change to
  `validate`.

- Round 210 — `repair_split_t_junctions(&mut scene, eps)` +
  `DEFAULT_T_JUNCTION_SPLIT_TOLERANCE` constant +
  `TJunctionSplitReport` carrier in `oxideav_stl::topology`.
  Mutating fix-up for the validate module's T-junction sub-check
  (`ValidationOptions::check_t_junctions`, off by default; brute-
  force `O(E · V_unique)` scan) — the spec's vertex-to-vertex rule
  (§6.5) says "every triangle must share exactly two vertices with
  each of its adjacent triangles" and the watertight edge-use
  check alone misses the case where one triangle's corner sits
  strictly inside another triangle's edge (the offending vertex
  is not an endpoint of the edge it sits on, so canonical-edge
  keys don't collide). The repair walks every `Triangles`
  primitive in isolation, collects the bit-exact-key set of
  every distinct corner position, then for each face tests its
  three edges for foreign on-edge incidence under the same
  geometric predicate the validate module uses (perpendicular
  distance `≤ eps · |PQ|` + projected parameter `t ∈ (eps, 1 -
  eps)`). The edge with the most splitters is picked (ties resolve
  in cyclic `(A,B) → (B,C) → (C,A)` order), the splitters are
  sorted along the edge by `t`, and the original face `(P, Q, R)`
  is replaced by a fan rooted at the *opposite* corner: `(P, V₁,
  R), (V₁, V₂, R), …, (Vₙ, Q, R)`. The fan preserves the original
  face's plane (so the computed normal is unchanged) and walks
  every sub-triangle in the same winding direction. Indexed
  primitives append the splitter positions (plus matched-length
  normals replicated from the apex slot) and rewrite the index
  buffer; `Indices::U16` auto-widens to `U32` only when a fresh
  splitter slot would push the position count past `u16::MAX`.
  Unindexed primitives have `positions` + matched-length
  `normals` fully rewritten as the new flat triangle soup; the
  per-face normal is replicated from the original face's apex
  normal slot because the fan preserves the plane.
  `TJunctionSplitReport { triangles_inspected, triangles_split,
  triangles_emitted, split_vertices_inserted, triangles_unchanged,
  skipped_length_mismatch }`: `triangles_split == 0` is the
  idempotency signal; the pass is fully count-balanced
  (`final_face_count == pre_face_count - triangles_split +
  triangles_emitted`); length-mismatched normals arrays skip the
  primitive entirely so the pass never invents nonsense face-
  normal data; `eps` outside `[0, 0.5)` or non-finite clamps to
  `DEFAULT_T_JUNCTION_SPLIT_TOLERANCE` (`1e-5`, matching
  `validate::DEFAULT_T_JUNCTION_TOLERANCE` exactly so the
  diagnostic↔repair pairing is consistent at the matching
  defaults). One pass handles the common producer pattern of
  "every face carries at most one T-junction"; nested splits where
  two new fan triangles each carry their own splitter need
  re-runs. Cross-primitive T-junctions are not detected
  (adjacency is per-primitive, matching the validate module's
  per-primitive edge accounting); pre-merge with
  `repair_weld_vertices` for cross-primitive coverage.
  Discriminant-preserving on indexed primitives (`U16` stays
  `U16` unless auto-widened); `prim.extras`, `mesh.name`, the
  scene-graph `nodes` / `roots`, and every non-affected vertex
  attribute (tangents, uvs, colours, joints, weights, morph
  targets) are preserved. The repair surface now covers every
  diagnostic the validate module exposes for the four spec rules
  AND the two spec sub-checks: facet-orientation →
  `repair_orient_normals_from_winding`, unit-normal →
  `repair_normalize_unit_normals`, vertex-to-vertex →
  `repair_weld_vertices` + `repair_drop_degenerate_triangles` +
  `repair_split_t_junctions`, positive-octant →
  `repair_translate_to_positive_octant`, consistent-winding →
  `repair_make_winding_consistent`. 11 unit tests (empty-scene
  no-op, non-`Triangles` skip, clean-pair no-op, classic
  T-junction unindexed + U32-indexed + U16-indexed with
  auto-widening, idempotency on already-clean scene, NaN-/inf-eps
  clamp to default, length-mismatch skip, multi-splitter fan
  emit, extras + mesh-name preservation) and 4 integration tests
  (decode → repair → re-validate drives `t_junction_defects` to
  zero; idempotency on the now-clean scene; face-count balance
  invariant `3 → 4`; constant-equality between
  `DEFAULT_T_JUNCTION_SPLIT_TOLERANCE` and
  `DEFAULT_T_JUNCTION_TOLERANCE`). Lib-test count rises by 11
  (155 → 166); integration-test file count rises by one.

- Round 205 — `repair_make_winding_consistent(&mut scene)` +
  `WindingConsistencyReport` carrier in `oxideav_stl::topology`.
  Mutating fix-up for the validate module's mesh-wide
  `inconsistent_winding_edges` rule (on by default in
  `ValidationOptions::check_consistent_winding`, landed in round
  100). Walks every `Triangles` primitive in isolation, builds the
  manifold-edge adjacency map (canonical undirected edge → list of
  incident face indices; exactly-two-incidences edges only —
  boundary and non-manifold edges left to the watertight rule),
  then BFS from each unvisited face. The seed face's winding is
  canonical by definition; for each manifold-edge neighbour, if
  the two faces walk the shared edge in opposite directions the
  neighbour is already consistent, if they walk it in the same
  direction the neighbour is flipped. A flip swaps the second and
  third vertex slots (in the index buffer for indexed primitives,
  in `positions` + matching-length `normals` for unindexed
  primitives) — the only data transformation that reverses the
  right-hand-rule cross-product direction of a face. The
  in-progress flip state is incrementally maintained during the
  BFS so neighbours-of-neighbours see the post-flip orientation;
  flips are batched and applied to the primitive's buffers at the
  end so the BFS sees a stable snapshot. Discriminant-preserving:
  `Indices::U16` stays `U16`, `Indices::U32` stays `U32`; the
  shared `positions` buffer is left untouched on indexed
  primitives (only the index entries for the flipped faces are
  swapped). `WindingConsistencyReport { triangles_inspected,
  triangles_flipped, components_visited, conflicting_edges }`:
  `triangles_flipped == 0` is the idempotency signal;
  `components_visited` rises one per BFS seed regardless of flips
  (not an idempotency signal); `conflicting_edges` increments when
  a flip decision would conflict with one already propagated
  through a different BFS path (the non-orientable
  Möbius-strip-like case). Non-`Triangles` primitives are silently
  skipped; `prim.extras`, `mesh.name`, the scene-graph `nodes` /
  `roots`, the `prim.positions` buffer of indexed primitives, and
  every non-affected vertex attribute (tangents, uvs, colours,
  joints, weights, morph targets) are preserved. Stored facet
  *normals* are NOT recomputed — flipping the winding inverts the
  cross-product direction, so
  `repair_orient_normals_from_winding` is the natural follow-up
  when the stored normal must agree with the new winding (the two
  passes are independent: this one fixes the mesh-wide invariant,
  the orient pass fixes the per-facet invariant). 11 unit tests
  (empty-scene no-op, non-`Triangles` skip, already-consistent
  quad idempotency, flipped-neighbour flip on unindexed quad +
  U32-indexed quad + U16-indexed quad with discriminant
  preservation, normals-swapped-in-lockstep for unindexed,
  per-component seed counting on disconnected triangles,
  face-count preservation, extras + mesh-name preservation) and 5
  integration tests (binary decode → repair → re-validate on a
  flipped-neighbour quad: 1 inconsistent edge pre-repair → 0
  post-repair; binary encoder round-trip; face-count preservation
  through the pipeline; second-pass no-op; clean-quad input is a
  no-op). Lib-test count rises by 11 (144 → 155); integration-test
  file count rises by one. The repair surface now covers every
  diagnostic the validate module exposes for the four spec rules
  AND the spec's mesh-wide winding invariant:
  facet-orientation → `repair_orient_normals_from_winding`,
  unit-normal → `repair_normalize_unit_normals`,
  vertex-to-vertex → `repair_weld_vertices` +
  `repair_drop_degenerate_triangles`,
  positive-octant → `repair_translate_to_positive_octant`,
  consistent-winding → `repair_make_winding_consistent`.

- Round 199 — `repair_translate_to_positive_octant(&mut scene, margin)`
  + `DEFAULT_POSITIVE_OCTANT_MARGIN` constant + `TranslateOctantReport`
  carrier in `oxideav_stl::topology`. Mutating fix-up for the
  validate-module's all-positive-octant rule (the 1989 spec says
  every vertex coordinate must be "positive-definite (nonnegative
  AND nonzero)"). Computes the per-axis bbox minimum across every
  `Triangles` primitive, then translates the scene by a single
  component-wise delta so every minimum lands strictly above zero.
  Per-axis-independent: an axis whose minimum is already `> 0` is
  left alone (delta on that axis = 0); only axes that violate the
  spec rule get shifted. The `margin` argument's sole purpose is to
  push the post-shift minimum strictly past zero (not at exactly
  zero, which would fail the spec's nonzero half); negative or
  non-finite margins clamp to `DEFAULT_POSITIVE_OCTANT_MARGIN`
  (`1e-6`). Non-finite vertex components pass through unchanged
  (`NaN + delta` stays `NaN`); fully-non-finite vertex slots are
  passed through bit-for-bit and reported under
  `TranslateOctantReport::skipped_non_finite_vertices` instead of
  `vertices_translated`. `prim.normals` are direction vectors and
  are not touched; `prim.extras`, `mesh.name`, the scene-graph
  `nodes` / `roots`, and every non-position vertex attribute
  (tangents, uvs, colours, joints, weights, morph targets) are
  preserved. Non-`Triangles` primitives are silently skipped, in
  keeping with the rest of the topology repair family. The pass is
  by construction idempotent: a second run sees a strictly-positive
  minimum on every axis and reports `delta == [0.0; 3]` +
  `vertices_translated == 0`. 11 unit tests (idempotency, negative-
  margin clamp, per-axis independence, mixed-finite vertex slots,
  non-`Triangles` skip, normal-preservation) and 5 integration
  tests (full decoder→repair→re-validate cycle on a binary STL
  whose `bbox.min` sits at `(-2, -3, -4)`: validate flags 1
  positive-octant defect pre-repair and 0 post-repair; binary
  encoder round-trip; triangle-shape preservation under pairwise
  edge distances; second-pass no-op; positive-scene no-op).
  Lib-test count rises by 11; integration-test file count rises by
  one. The repair surface now mirrors every diagnostic the
  validate module exposes for the four spec rules:
  facet-orientation → `repair_orient_normals_from_winding`,
  unit-normal → `repair_normalize_unit_normals`,
  vertex-to-vertex → `repair_weld_vertices` +
  `repair_drop_degenerate_triangles`, and now positive-octant →
  `repair_translate_to_positive_octant`.

## [0.0.3](https://github.com/OxideAV/oxideav-stl/compare/v0.0.2...v0.0.3) - 2026-05-29

### Other

- Round 189 — chunks_exact-driven unpack_triangle_record on binary decode
- Round 175 — profile arc: pack-record binary encode + flamegraph drivers
- Round 161 — Criterion bench suite (decode / encode / dedup / validate)
- Round 155 — cargo-fuzz harness + nightly fuzz workflow

### Changed

- Round 189 — `binary::decode` now walks the triangle body via
  `chunks_exact(TRIANGLE_BYTES)`, converts each 50-byte chunk to an
  `&[u8; 50]` reference via `try_into`, and unpacks it with a new
  `unpack_triangle_record` helper. The compiler folds the four
  per-field `f32::from_le_bytes` slice indices plus the two
  attribute-byte reads into a single chunk-length proof, replacing
  the previous `cursor += 50` arithmetic and four
  `read_vec3(&bytes[cursor..cursor + 12])` invocations (each
  carrying its own implicit bounds check). Symmetric counterpart of
  the round-175 `pack_triangle_record` optimisation on the encoder
  side; the round-trip wire bytes are unchanged and pinned by the
  `binary_cube_triangle_records_roundtrip_byte_identical`
  integration test. Measured speedup on an Apple M-series host
  (`cargo bench --bench decode --quick`, release-profile):
  `decode_binary/100000` 7.57 GiB/s -> 7.71 GiB/s;
  `decode_binary/10000` 7.51 GiB/s -> 7.69 GiB/s;
  `decode_binary/1000` 6.30 GiB/s -> 6.42 GiB/s. ASCII decode is
  unaffected (it does not flow through the binary record unpacker).

### Added

- Round 175 — profile arc: six deterministic single-threaded
  `examples/profile_*` drivers wrapping each hot path
  (`encode_binary`, `decode_binary`, `encode_ascii`, `decode_ascii`,
  `dedup_spatial`, `validate`) so a profiler (`cargo flamegraph`,
  `samply record`, `perf record`, Instruments Time Profiler) can
  attribute cycles line-by-line without Criterion's adaptive
  batching getting in the way. Shared fixture builders live in
  `examples/profile_common/mod.rs` and use the same xorshift32 seed
  scheme as `benches/common/mod.rs`, so a profile driver's input is
  byte-identical to the matching bench at the same triangle count.

### Changed

- Round 175 — `binary::encode` now packs each STL binary triangle
  record into a stack-resident `[u8; 50]` via a new
  `pack_triangle_record` helper and emits it with a single
  `Vec::extend_from_slice` call, replacing the previous
  14-call-per-triangle pattern (12 `write_vec3` four-byte writes +
  two single-byte `push`es). The output is byte-identical to the
  previous code; the
  `binary_cube_triangle_records_roundtrip_byte_identical`
  integration test pins the invariant. Measured speedup on an Apple
  M-series host (`cargo bench --bench encode --quick`,
  release-profile): `encode_binary/10000` 2.70 GiB/s → 7.94 GiB/s
  (~2.9× throughput); `encode_binary/100000` 2.28 GiB/s →
  5.55 GiB/s; `encode_binary/1000` 2.37 GiB/s → 5.83 GiB/s. The
  ASCII output path is unaffected (it does not flow through the
  binary record packer); ASCII encode benches stay at their
  round-161 numbers.

- Round 161 — Criterion bench suite (`benches/decode`, `benches/encode`,
  `benches/dedup`, `benches/validate`).
  - All inputs synthesised on the fly from deterministic xorshift32
    PRNG seeds (no committed binary corpora, no `docs/` traffic) so
    sweeps stay bit-stable across hosts and runs. Shared fixture
    builders live in `benches/common/mod.rs` and are pulled in by
    each bench via `#[path = "common/mod.rs"] mod common;`.
  - `decode` covers the binary path at 1 K / 10 K / 100 K triangles
    and the ASCII path at 1 K / 5 K / 10 K. Throughput in bytes/s
    so the two formats compare directly at the same triangle count.
  - `encode` covers binary at 1 K / 10 K / 100 K (throughput in
    bytes-out/s = `84 + n * 50`) and ASCII at 1 K / 5 K / 10 K
    (latency only — ASCII output width depends on the per-coordinate
    formatter).
  - `dedup` measures the three vertex-deduplication code paths at
    matched element counts: `StlEncoder::stats` (bit-exact
    `HashMap`-keyed baseline), `EncodeStats::with_tolerance` (the
    `O(N · K)` brute force), and
    `EncodeStats::with_tolerance_spatial` (the `O(N)` spatial-grid
    path). Both tolerance paths use `eps == 1.0e-5` to stay on
    their general branches (`eps == 0.0` short-circuits to the
    bit-exact branch documented in the trace contract). The bench
    materialises the crossover the README has documented since
    round 5: brute force wins at small N on constant factors,
    spatial pulls ahead before 10 K vertices, and by 100 K the gap
    is the whole reason `_spatial` exists.
  - `validate` measures the default-on rule set (facet orientation
    + unit-length normal + watertight/manifold + consistent winding)
    at 1 K / 10 K / 100 K triangles, and the opt-in T-junction
    sub-check at 100 / 300 / 1 K triangles. The 70× ratio between
    the two confirms the round-10 `check_t_junctions = false`
    default — turning it on is a diagnostic-only investment, not a
    default-pipeline option.
  - `criterion = "0.5"` added under `[dev-dependencies]` only; the
    `[[bench]] harness = false` lines route to our own
    `criterion_main!` instead of libtest. The standalone
    (`--no-default-features --lib`) build path is unchanged — the
    bench suite uses the registry-enabled feature set in line with
    the rest of the development dependency tree.

## [0.0.2](https://github.com/OxideAV/oxideav-stl/compare/v0.0.1...v0.0.2) - 2026-05-24

### Other

- Round 115 — ascending-z facet sort repair (slicer optimisation)
- Round 100 — consistent-winding (directed-edge) validation check
- Round 10 — opt-in T-junction sub-check in oxideav_stl::validate
- Round 9 — orient-from-winding + unit-length normal repairs + spec-style scientific ASCII formatter
- Round 8 — degenerate-triangle culling + zero-normal recompute repairs
- Round 7 — topology utilities + ASCII comment-line tolerance

### Added

- Round 155 — cargo-fuzz harness + nightly fuzz workflow.
  - New `fuzz/` subcrate with two libfuzzer targets:
    - `decode` drives arbitrary attacker-controlled bytes through
      `StlDecoder::decode` and asserts the call always returns a
      `Result` rather than panicking / aborting / OOMing. The
      detector + both parsers (binary `uint32` triangle-count slot,
      ASCII `solid` / `facet` / `outer loop` / `vertex` keyword
      walk, multi-`solid` mesh proliferation, Materialise / VisCAM
      per-face colour distribution, `84 + N*50` size override,
      UTF-8 BOM + leading-whitespace skip) are all exercised by
      libfuzzer's coverage-guided byte mutation in a single target.
    - `roundtrip` synthesises a small binary STL from fuzz-
      controlled bytes (`data[0] % 65` triangles, body bytes
      cycled from `data[1..]`), decodes it through `StlDecoder`,
      re-encodes through `StlEncoder::new_binary`, and asserts the
      triangle-count slot + every per-triangle record survive
      byte-for-byte. The 80-byte header is allowed to differ — both
      writers substitute their own signature, matching the
      `binary_cube_triangle_records_roundtrip_byte_identical`
      integration-test invariant. No external library worth
      dlopen-ing as a cross-decode oracle (clean-room wall + STL
      flavours don't agree on header semantics), so this is a
      self-roundtrip target.
  - The fuzz subcrate pulls `oxideav-stl` with `default-features =
    false`, exercising the standalone (no `oxideav-core`) build
    path end-to-end. Its own `[workspace]` block keeps it out of
    the umbrella `crates/*` glob.
  - New `.github/workflows/fuzz.yml` schedules a daily 1800-second
    fuzz run via the shared `OxideAV/.github/.github/workflows/
    crate-fuzz.yml@master` reusable workflow (cron `37 7 * * *`,
    jittered off the hour to space against sibling-crate fuzz
    jobs).
  - Seed corpus: one ASCII single-facet stl, one binary single-
    triangle stl for the `decode` target; one synthetic 51-byte
    seed for the `roundtrip` target. Local smoke runs of 15-20s
    per target exercised ~3M (decode) + ~880K (roundtrip)
    iterations with zero crashes.

- Round 115 — ascending-z facet sort repair (`oxideav_stl::topology`).
  - New `repair_sort_triangles_by_z(&mut scene) -> SortByZReport`
    reorders every `Triangles` primitive's facets into ascending
    z-value order in place, materialising the 1989 spec's
    recommendation that "sorting the triangles in ascending z-value
    order is recommended, but not required, in order to optimize
    performance of the slice program". A slicer sweeps a cutting plane
    upward; emitting facets in the order their lowest corner enters the
    sweep lets the slicer stream triangles instead of re-scanning the
    soup at each layer.
  - Sort key is the triangle's three corner z-values sorted ascending —
    `(min_z, mid_z, max_z)`. The primary key is the lowest corner (when
    the slice plane first touches the facet); `mid_z` then `max_z` are
    deterministic tie-breakers. Comparison uses `f32::total_cmp`, giving
    a total order over all `f32` values — a facet whose minimum z is
    non-finite (all three corners NaN) sorts last rather than scrambling
    the finite facets around it; a facet with a single NaN corner still
    keys on its finite minimum. The sort is **stable**, so equal-key
    facets keep their emit order and a re-run reports
    `triangles_reordered == 0` (the idempotency signal).
  - Indexed primitives have only their index buffer rewritten in the
    sorted face order — the `Indices::U16`/`U32` discriminant and the
    shared `positions`/`normals` arrays are preserved. Unindexed
    primitives have their `positions` (and `normals`, when present and
    length-matched 1:1) re-laid-out three corners at a time. The pass
    never adds, removes, or alters a triangle's geometry — it is a pure
    count-preserving reordering. Non-`Triangles` primitives are skipped;
    `prim.extras`, `mesh.name`, and the scene-graph are untouched. A
    face whose index references an out-of-range position is kept (sort
    never drops geometry — that is `repair_drop_degenerate_triangles`'
    job) and sorts to the end via the NaN-high sentinel.
  - New `SortByZReport { triangles_inspected, triangles_reordered }`.
    Re-exported at the crate root as `repair_sort_triangles_by_z`,
    `SortByZReport`. Added as step 6 of the README repair pipeline.
  - 13 new unit tests (empty-scene no-op, unindexed ascending order,
    already-sorted idempotency, second-pass-reorders-nothing,
    keys-on-min-corner-not-max, stable-for-equal-keys, U16 + U32
    discriminant preservation, normals-carried-along, non-`Triangles`
    skip, all-NaN-face-sorts-last, single-NaN-corner-keys-on-finite-min,
    count-preservation) + 5 integration tests (binary decode → sort
    ascending, idempotent through the decoder, binary-encoder round-trip
    in order, count + geometry-set preservation, ASCII decode → sort →
    binary re-emit).

- Round 100 — consistent-winding (directed-edge) check in
  `oxideav_stl::validate`.
  - The 1989 spec's facet-orientation rule (§6.5) says the three
    vertices are "listed in counterclockwise order when looking at the
    object from the outside (right-hand rule)" and that the two pieces
    of orientation information "must be consistent". The existing
    `check_facet_orientation` enforces the *per-facet* consistency
    (stored normal vs winding); the watertight rule counts *undirected*
    edge uses. Neither catches a triangle whose winding is flipped
    relative to its neighbour: such a surface can be perfectly
    watertight (every edge used twice) yet have a shared edge that both
    adjacent triangles traverse in the *same* direction. The new check
    is that missing mesh-wide invariant.
  - New `ValidationOptions::check_consistent_winding: bool` (default
    `true`). For each canonical undirected edge with exactly two
    incident triangles, the check records each triangle's traversal
    direction; a correctly oriented manifold edge is walked in
    *opposite* directions (`A→B` and `B→A`). Same-direction traversal
    flags one of the two neighbours as flipped. Boundary edges (one
    incidence) and non-manifold edges (3+) are left to the watertight
    rule — direction consistency is only well-defined for the clean
    two-triangle case. Degenerate edges (coincident endpoints) are
    skipped.
  - New `ValidationReport::inconsistent_winding_edges: usize` and
    `inconsistent_winding_examples: Vec<FaceLocator>` (each offending
    edge contributes both adjacent triangles' locators, de-duplicated,
    capped at `MAX_REPORTED_DEFECTS`). `ValidationReport::is_clean()`
    now includes `inconsistent_winding_edges == 0` in its conjunction.
  - `FaceLocator` now derives `Hash` (used for example de-duplication).
  - Uses bit-exact `f32` position equality like the watertight check;
    meshes whose duplicate corners differ by floating-point noise
    should be pre-welded via `repair_weld_vertices`.
  - 6 new unit tests (default-on, flipped-neighbour detection,
    clean-cube stays clean, opt-off zeroes the fields,
    non-two-incidence edges ignored, empty scene vacuous) + 2
    integration tests (clean binary cube has consistent winding,
    ASCII flipped-neighbour flagged through the decoder).

- Round 10 — opt-in T-junction sub-check in `oxideav_stl::validate`.
  - The 1989 spec's vertex-to-vertex rule ("a vertex of one
    triangle cannot lie on the side (edge) of another triangle")
    is not covered by the watertight edge-use count alone — the
    offending vertex is not an endpoint of the edge it sits on,
    so canonical edge keys never collide. The new sub-check is
    the missing piece.
  - New `ValidationOptions::check_t_junctions: bool` (default
    `false`) and `ValidationOptions::t_junction_tolerance: f32`
    (default `1e-5`, exposed as the new public
    `DEFAULT_T_JUNCTION_TOLERANCE` constant). Off by default
    because the scan is `O(E · V_unique)` brute-force and is
    intended for diagnostic use, not the default report.
  - New `ValidationReport::t_junction_defects: usize` and
    `t_junction_examples: Vec<FaceLocator>` (capped at
    `MAX_REPORTED_DEFECTS` like the other rules). Each distinct
    `(offending-vertex, edge)` incidence counts once; the owning
    triangles of each offending vertex are recorded as examples
    (a triangle whose corner sits on someone else's edge is the
    spec violation, not the edge-owner).
  - Geometric predicate: vertex V lies strictly between segment
    `P-Q` when the perpendicular distance from V to the infinite
    line through `P, Q` is at most `eps · |PQ|` AND the projected
    parameter `t = ((V - P) · (Q - P)) / |Q - P|²` lies in
    `(eps, 1 - eps)`. Endpoint-matching vertices (bit-exact
    equality on either side) are excluded — that's the well-
    formed edge-sharing case.
  - Empty scene + check on → vacuously clean. Negative /
    non-finite tolerance clamps to the default. Non-finite
    coordinates and degenerate edges (`|PQ|² == 0`) return false
    from the predicate.
  - `ValidationReport::is_clean()` now includes
    `t_junction_defects == 0` in its conjunction.
  - 12 new tests: 8 unit tests on the geometric predicate +
    integration tests (default-off behaviour, opt-in detection of
    a 3-triangle classic split-edge layout, clean two-triangle
    strip stays clean, public-API tolerance constant pinned,
    100-strip cap test verifies `MAX_REPORTED_DEFECTS` cap holds
    while the count keeps climbing).

- Round 9 — orientation-flip + unit-length normal repairs
  (`oxideav_stl::topology`).
  - New `repair_orient_normals_from_winding(&mut scene, eps) ->
    OrientReport` rewrites every stored facet normal whose direction
    disagrees with the right-hand-rule cross product of its winding
    (`dot(stored, recomputed) < 0`) to the recomputed unit-normalised
    direction. The 1989 spec says facet orientation is "specified
    redundantly in two ways which must be consistent"; this repair
    makes winding the authoritative source. Per-face: zero-sentinel
    normals are skipped (deferred to
    `repair_recompute_zero_normals`); below-`eps` cross-product
    triangles count under `skipped_degenerate`; primitives with a
    missing or mismatched-length `normals` field are skipped without
    modification. Non-`Triangles` primitives are silently skipped.
    `flipped_normals == 0` is the idempotency signal.
  - New `repair_normalize_unit_normals(&mut scene, unit_tolerance) ->
    NormalizeReport` rescales any non-unit stored normal to unit
    length, preserving direction. Matches the 1989 spec's "unit
    normal" rule; uses the same tolerance constant
    (`validate::DEFAULT_UNIT_NORMAL_TOLERANCE`, 1e-3) the validate
    module's `non_unit_normal_defects` check uses, so the repair and
    the diagnostic stay in lockstep. Zero-sentinel normals are
    skipped; primitives with missing / length-mismatched `normals`
    are reported. `rescaled_normals == 0` is the idempotency signal.
  - Re-exported at the crate root as
    `repair_orient_normals_from_winding`, `OrientReport`,
    `repair_normalize_unit_normals`, `NormalizeReport`.
  - 20 new unit tests (9 for orient, 11 for normalize) + 6
    integration tests (3 binary-decode-then-repair workflows for
    each repair) exercise pre/post validate state and through-the-
    binary-encoder round-trips.

- Round 9 — spec-style scientific ASCII number formatter.
  - New `AsciiNumberFormat` enum (`RoundTrip` | `FixedDecimal { precision }`
    | `SpecScientific { precision }`) on `oxideav_stl::AsciiEncodeOptions`
    selects the float-formatting policy for ASCII output.
  - `SpecScientific` matches the 1989 spec's `1.23456E+789` worked
    example verbatim — mantissa + literal `E` + explicit `+`/`-`
    exponent sign. Distinguished from Rust's `{:E}` (which emits
    `1.23456E789` with no sign) and from the existing
    `with_float_precision` fixed-decimal flavour.
  - New `StlEncoder::with_spec_scientific(Option<usize>)` convenience
    setter, plus `StlEncoder::with_number_format(AsciiNumberFormat)`
    for full-control callers wiring the knob through higher-level
    plumbing. The historical `with_float_precision` keeps its
    semantics: `RoundTrip + Some(n)` → `FixedDecimal { precision: n }`
    so existing tests are untouched.
  - Re-exported at the crate root as `AsciiNumberFormat`.
  - 6 new integration tests cover explicit-exponent-sign emission,
    negative-exponent minus-sign, parser round-trip at 7-digit
    precision, revert-to-default via `None`, binary-format
    unaffected, and `with_number_format` ↔ `with_spec_scientific`
    parity.

- Round 8 — degenerate-triangle culling + zero-normal recompute
  repairs (`oxideav_stl::topology`).
  - New `repair_drop_degenerate_triangles(&mut scene) ->
    DegenerateDropReport` removes zero-area triangles in-place from
    every `Triangles` primitive. A triangle is considered degenerate
    when any two of its three corner *positions* coincide by
    bit-exact `f32` match (the same equality model the rest of the
    crate uses). Indexed primitives keep their `Indices::U16` /
    `Indices::U32` discriminant and have their index buffer rewritten
    with the surviving triangle slots; unindexed primitives have
    their `positions` (and matching-length `normals`) compacted in
    place. Non-`Triangles` primitives are silently skipped. The
    routine intentionally uses position-equality rather than
    zero-cross-product to avoid culling hairline strips that CAD
    pipelines deliberately emit. `DegenerateDropReport {
    triangles_inspected, dropped_triangles }` — `dropped_triangles
    == 0` is the idempotency signal.
  - New `repair_recompute_zero_normals(&mut scene, eps) ->
    NormalRecomputeReport` implements the STL spec's "consumer should
    recompute from winding" sentinel. For each triangle whose three
    *current* per-vertex normals are all (within `eps`) zero, the
    routine rewrites them with the right-hand-rule cross product of
    that triangle's three positions. Triangles where some corners
    carry non-zero normals and others do not are left alone — a tell
    that the producer mixed face-normal and vertex-normal data.
    Primitives whose `normals` field is `None` get one freshly
    populated; primitives whose `normals` length disagrees with
    `positions.len()` are skipped and counted under
    `skipped_length_mismatch`. `eps == 0.0` matches the strict
    spec rule (exact-zero only); positive `eps` widens it to catch
    float-noise zeros. Mathematically-degenerate faces (zero cross-
    product magnitude) are reported under `skipped_degenerate`
    rather than rewritten to a sentinel.
  - Re-exported at the crate root as `repair_drop_degenerate_triangles`,
    `DegenerateDropReport`, `repair_recompute_zero_normals`,
    `NormalRecomputeReport`.
  - 17 new unit tests (8 for degenerate drop, 9 for normal recompute)
    + 6 integration tests (3 binary-decode-then-repair workflows for
    each repair).

- Round 7 — mesh topology utilities (`oxideav_stl::topology`).
  - New `shells(&scene) -> Vec<Shell>` splits the triangle soup into
    its connected components via BFS over bit-exact shared vertex
    positions. Each `Shell { face_indices, vertices, edges, faces,
    boundary_edges, non_manifold_edges }` carries the per-shell
    V/E/F counts plus the in-shell edge-use breakdown.
  - `Shell::euler_characteristic() -> i64` returns χ = V − E + F;
    `Shell::is_closed_manifold()` reports whether every edge appears
    in exactly two triangles within the shell; `Shell::genus()`
    estimates the genus for closed orientable shells via
    `g = (2 − χ) / 2`. Returns `None` when the shell is not closed-
    manifold or the formula does not apply (odd numerator).
  - `repair_weld_vertices(&mut scene) -> WeldReport` rewrites every
    `Triangles` primitive to use a shared `Indices::U32` buffer
    keyed on bit-exact `f32` positions; non-`Triangles` primitives
    are left untouched. `WeldReport { triangles_inspected,
    slots_collapsed, positions_collapsed, degenerate_triangles }` —
    `positions_collapsed == 0` is the idempotency signal (the pass
    actually changed something iff it's > 0); `slots_collapsed`
    remains the gross emit-vs-canonical ratio for tooling.
  - Re-exported at the crate root as `shells`,
    `repair_weld_vertices`, `Shell`, `TopologyFaceLocator`, and
    `WeldReport`. The locator type is module-local rather than
    sharing `validate::FaceLocator` so `topology` is usable
    standalone.

- Round 7 — ASCII comment-line tolerance (vendor quirk).
  - The ASCII parser and the ASCII-vs-binary sniffer now treat
    whole-line `;`-introduced and `#`-introduced comments as
    whitespace. Hand-edited STL files and a handful of vintage CAD
    exporters annotate their output with these; the 1989 spec
    defines no comment syntax, but the dominant real-world tolerance
    is to skip them silently.
  - Comments are recognised only at token boundaries (i.e. after
    whitespace has been consumed); inline comments mid-token are
    NOT tolerated so the syntax cannot conflate with `vertex` /
    `facet` / numeral tokens.
  - Both the prefix sniff (`is_ascii_stl`) and the parser's
    `skip_ws` share the same comment-aware whitespace skipper, so a
    file starting with `; …\n solid …` classifies as ASCII and
    parses cleanly end-to-end.

## [0.0.1](https://github.com/OxideAV/oxideav-stl/compare/v0.0.0...v0.0.1) - 2026-05-10

### Other

- Round 6 — opt-in geometry validation + bbox + non_exhaustive cascade
- add Primitive.targets + Mesh.weights to all literal sites
- Add Primitive.targets + Mesh.weights to literal struct sites
- Round 5 — README: spatial dedup + share_stats trace event
- Round 5 — spatial-grid variant of tolerance-based vertex dedup
- Round 5 — ASCII-mode parity test for apply_pre_encode_extras
- Round 5 — share_stats JSONL trace event (encoder-only)
- Round 4 — docs/trace-contract.md companion document
- Round 4 — opt-in auto-inject of stl:unique_vertex_count extras
- Round 4 — tolerance-based vertex dedup helpers
- Round 3 — multi-solid ASCII + float-precision knob + EncodeStats
- Round 3 — Materialise binary-header default colour + material round-trip
- Round 2 — 16-bit per-face colour extension (VisCAM + Materialise)
- Round 2 — JSONL trace emitter (`trace` feature)
- Round 2 — fuzz-resistant ASCII-vs-binary detection

### Added

- Round 6 — opt-in spec-aligned geometry validation.
  - New `oxideav_stl::validate` module with `validate(&scene, &opts)
    -> ValidationReport` covering the four spec rules from §6.5 of
    Marshall Burns' *Automated Fabrication* transcription:
    facet orientation (stored normal vs recomputed-from-winding,
    component-wise tolerance), unit-length normal, vertex-to-vertex
    (watertight / manifold via per-edge bit-exact use counts), and
    the SLA-era all-positive-octant rule (off by default — modern
    slicers ignore it).
  - `ValidationReport { triangles_total, facet_orientation_defects,
    non_unit_normal_defects, positive_octant_defects, boundary_edges,
    non_manifold_edges, watertight, * _examples }` — counts are
    unbounded; per-rule `_examples` lists are capped at
    `MAX_REPORTED_DEFECTS` (32) so the report stays cheap to log
    even on million-triangle inputs. Each `FaceLocator { mesh,
    primitive, face }` indexes back to the originating triangle
    in scene-graph order + post-index-buffer-resolution face index.
  - `ValidationOptions` lets callers toggle each rule independently
    and override the per-rule tolerances (`DEFAULT_NORMAL_TOLERANCE`
    + `DEFAULT_UNIT_NORMAL_TOLERANCE`, both `1e-3`). Zero-length
    stored normals are accepted as the spec'd "consumer should
    recompute from winding" sentinel.
  - New `bbox(&scene) -> Option<Bbox>` returns the axis-aligned
    bounding box of every `Triangles` vertex in the scene
    (non-finite coordinates skipped; non-`Triangles` primitives
    silently skipped). `Bbox::extents` / `Bbox::centre` /
    `Bbox::is_degenerate` round out the API.
  - Validation is **opt-in and non-mutating** — neither the encoder
    nor decoder invokes it. Intended for pipeline tooling, bug
    bisection, and format-conversion adapters that need to know
    whether the source surface is watertight before exporting.
  - 17 unit + 8 integration tests (round-trip-via-decoder unit cube
    is clean + watertight; two-triangle ASCII strip surfaces 4
    boundary edges; three-triangle "fin" surfaces a non-manifold
    edge; example caps; positive-octant on/off behaviour).

- Round 6 — forward-compatible `Mesh` + `Primitive` construction.
  - Migrated every literal `Primitive { … }` site to
    `Primitive::new(Topology::*) + per-field assignment` and every
    literal `Mesh { … }` site to
    `Mesh::new(name).with_primitive(prim)`. mesh3d round 7 marks
    both structs `#[non_exhaustive]`; the new construction style
    works against today's published mesh3d 0.0.1 AND the upcoming
    non_exhaustive 0.0.2 without further churn.

- Round 5 — README refresh.
  - Documents `with_tolerance_spatial` /
    `unique_vertices_with_tolerance_spatial` alongside the
    brute-force tolerance section so the spatial path is the
    obvious choice for large-mesh callers.
  - Refreshes the trace-tape paragraph to mention the encoder-only
    `share_stats` event and the decode-vs-encode tape distinction.
  - Drops the round-4 "Round 5 candidates" section now that those
    items have landed.

- Round 5 — spatial-grid variant of the tolerance dedup helper.
  - New `EncodeStats::with_tolerance_spatial(scene, eps)` +
    `StlEncoder::unique_vertices_with_tolerance_spatial(scene, eps)`.
    Bins each emitted vertex into a uniform-grid cell of side
    `eps × 2`, then scans the 27 surrounding cells for an existing
    canonical within tolerance. Amortises to `O(N)` for typical
    geometry (the brute-force `O(N · K)` path remains the
    reference).
  - Cross-tested against the bit-exact path for `eps == 0.0` (must
    produce **identical** counts and dedup-map shapes — both paths
    delegate to the same `f32::to_bits`-keyed HashMap on the fast
    path) and against the brute-force path on noisy fixtures
    (collapses 9 perturbed copies of a single triangle to 3
    canonicals at `eps = 1e-5`).
  - Approximate by design — see `docs/trace-contract.md` §
    "Spatial-dedup notes" for the exact contract: every two points
    the spatial path merges are within `eps` on every axis under
    the Chebyshev metric, but the spatial path may emit one
    additional canonical when borderline points fall into
    non-adjacent cells.
  - NaN coordinates land in a sentinel cell so each NaN takes its
    own canonical slot, matching the well-defined NaN handling of
    the brute-force + bit-exact paths.
  - Negative / non-finite `eps` clamps to zero; `eps == 0.0` short-
    circuits to the bit-exact branch.

- Round 5 — ASCII-mode parity test for `apply_pre_encode_extras`.
  - New `tests/ascii_apply_pre_encode_extras.rs` mirrors the
    round-4 binary suite against `StlEncoder::new_ascii`. The hook
    is format-agnostic by design (`apply_pre_encode_extras` mutates
    the scene independently of the eventual emit format), but the
    pre-existing tests only exercised it through the binary
    encoder; the parity suite locks in the format-agnostic
    contract so a future refactor that accidentally makes the
    hook format-aware fails loudest.
  - Includes a direct same-scene-different-encoder-format
    parity assertion (the byte-for-byte equivalent of "the hook
    leaves the scene in identical states regardless of which
    StlEncoder produced it").

- Round 5 — `share_stats` JSONL trace event (encoder-only).
  - With `--features trace` ON and a trace path configured, both the
    binary and ASCII encoders now emit a single `share_stats` event
    between the final `triangle` (binary: between `triangle_count`
    and `done`; ASCII: between `triangle_count` and `done` after the
    triangles) carrying the same vertex-share summary the
    synchronous `EncodeStats` API surfaces. Fields:
    `{ "kind": "share_stats", "triangles", "emitted_vertices",
       "unique_vertices", "share_factor", "tolerance_eps" }`.
  - The encoder always reports the bit-exact summary
    (`tolerance_eps == null`); tolerance variants stay on the
    synchronous side because the trace tape is the ε-free
    audit-handoff channel.
  - Decoder tapes do not emit `share_stats` (no `&Scene3D` summary
    is available at decode time). Documented in
    `docs/trace-contract.md` alongside the updated
    decode-vs-encode ordering invariants and the new
    spatial-dedup notes.

- Round 4 — `docs/trace-contract.md` companion document.
  - One-page reference for the JSON-Lines event vocabulary the
    `trace` Cargo feature emits — header / triangle_count /
    triangle / done event shapes, field types, and ordering
    invariants.
  - Documents the multi-`solid` ASCII trace behaviour (only the
    first block's name fires a `header` event; the tape is a flat
    triangle stream across all blocks).
  - Includes a worked four-line example for a single-triangle
    binary STL so cross-impl auditors can sanity-check their tape
    against a known-good reference without running the codec.

- Round 4 — opt-in auto-injection of `stl:unique_vertex_count` extras.
  - New `StlEncoder::with_auto_inject_unique_count(bool)` setter +
    `auto_inject_unique_count()` accessor.
  - New `StlEncoder::apply_pre_encode_extras(&mut scene)` hook that,
    when the toggle is on AND `EncodeStats::share_factor() > 1.5`
    (the public `AUTO_INJECT_SHARE_FACTOR_THRESHOLD` constant),
    stamps every Triangles primitive's
    `Primitive::extras["stl:unique_vertex_count"]` (also re-exported
    as `UNIQUE_VERTEX_COUNT_EXTRAS_KEY`) with the bit-exact count
    from `StlEncoder::stats`. Idempotent; non-Triangles primitives
    are skipped.
  - The `Mesh3DEncoder::encode` pass remains pure-functional on
    `&Scene3D`; callers invoke the hook explicitly between
    configure-and-emit because the trait signature cannot mutate
    the scene during the emit pass.
  - Decoder leaves the key alone — STL has no native vertex sharing,
    so the value is metadata only.

- Round 4 — tolerance-based vertex dedup helpers.
  - New `EncodeStats::with_tolerance(&scene, eps)` builds an
    `EncodeStats` whose `unique_vertices` field reflects ε-equality
    instead of bit-exact `f32` matching. `triangles` and
    `emitted_vertices` come straight from the bit-exact path so the
    return shape is interchangeable.
  - New `StlEncoder::unique_vertices_with_tolerance(&scene, eps)`
    returns `(unique_count, dedup_map)`; `dedup_map[i]` is the
    canonical-slot index assigned to the `i`-th emitted vertex (in
    encoder order, post-index-buffer resolution).
  - Two emitted vertices are merged when each component-wise absolute
    distance is `≤ eps`. `eps == 0.0` (and any negative / non-finite
    value, clamped to zero) reduces the scan to bit-exact f32 equality
    on finite values, matching `StlEncoder::stats`.
  - The bit-exact path (`StlEncoder::stats`) is unchanged. Tolerance
    dedup is `O(N · K)` and intended for diagnostic / pipeline-stats
    use; large-mesh callers should spatial-index upstream.

- Round 3 — multi-`solid` ASCII parsing + per-mesh emit.
  - The decoder now accepts back-to-back `solid NAME … endsolid NAME`
    blocks (older Pro/E + AutoCAD ASCII exporters concatenate
    multiple). Each block becomes its own `Mesh` in the resulting
    `Scene3D`, with one root `Node` per mesh in source order. The
    historical single-block path is unchanged.
  - The encoder mirrors the decoder: multi-mesh scenes round-trip as
    one `solid NAME … endsolid NAME` block per mesh; single-mesh
    scenes still produce a single block. Empty scenes emit
    `solid\nendsolid\n`.
  - Stray non-`solid` content between blocks is rejected with a clear
    `InvalidData` error rather than producing a partial scene.

- Round 3 — ASCII pretty-printer with configurable float precision.
  - New `oxideav_stl::AsciiEncodeOptions` struct with optional
    `float_precision` field; `StlEncoder::with_float_precision(Some(n))`
    switches the ASCII encoder to fixed-decimal `{:.n}` output.
  - The default (`None`) preserves the historical round-trip-safe
    `{}` formatting — every existing test and consumer is untouched.
  - The knob has no effect on binary output; binary triangle records
    remain byte-identical regardless of the precision setting.

- Round 3 — pre-encode statistics (`StlEncoder::stats`).
  - New `EncodeStats { triangles, emitted_vertices, unique_vertices }`
    summary computed without paying for the full encode pass.
  - `unique_vertices` deduplicates by exact `f32` bit pattern (the
    only definition that is well-defined for floats given that
    `0.1 + 0.2 != 0.3`); a fully-shared cube (8 corners, 12 triangles)
    reports `unique_vertices == 8` even though `emitted_vertices == 36`.
  - `EncodeStats::share_factor()` returns `emitted_vertices /
    unique_vertices` (4.5 for the cube above), useful for tooling that
    reports compression ratios.

- Round 3 — Materialise binary-header per-object default colour and
  material round-trip.
  - The 80-byte binary STL header is scanned on decode for
    Materialise Magics' textual `COLOR=R G B A` and
    `MATERIAL=Ar Ag Ab Sa Dr Dg Db Sd Sr Sg Sb Ss` lines (each
    optional, either order). Either or both populate
    `Primitive::extras["stl:default_color"]` (4-element `u8` JSON
    array) and `Primitive::extras["stl:default_material"]` (12-element
    `u8` JSON array).
  - On encode, if either extras key is present and the right shape,
    the encoder rebuilds a Materialise-compatible header (NUL-padded to
    80 bytes) and emits it in place of the writer-signature default.
    Out-of-shape values silently fall back to the default header.
  - Token parsing is strict on count + range (`0..=255`) but tolerant
    on order, padding, and unknown adjacent lines (vendor signatures
    carry through). Tokens lift the per-object default that
    Materialise's per-face `valid=0` slots refer back to.

- Round 2 — 16-bit per-face colour extension support.
  - New `oxideav_stl::color` module with `ColorConvention`
    (`ViscamSolidView` / `Materialise`) + `Stl16BitColor` (5+5+5+1
    valid bit) + `detect()` heuristic.
  - VisCAM/SolidView layout: `[valid:1][R:5][G:5][B:5]` with
    `valid=1` meaning live; Materialise layout:
    `[valid:1][B:5][G:5][R:5]` with `valid=0` meaning live (channel
    order AND valid-bit polarity differ).
  - Decoder calls `color::detect` against the per-face attribute
    population and surfaces the result on
    `Primitive::extras["stl:color_convention"]` as `"viscam"` /
    `"materialise"` whenever the bit-15 distribution is unambiguous;
    raw bytes still round-trip via `stl:per_face_attributes`.
  - `Stl16BitColor::r8/g8/b8` upscale 5-bit channels to 8-bit using
    bit-replication (`(c << 3) | (c >> 2)`) so 0x1F → 0xFF.
  - `ColorConvention` implements `std::str::FromStr` for round-trip
    through the extras tag.

- Round 2 — JSON-Lines trace emitter (`trace` Cargo feature).
  - With `--features trace` AND `OXIDEAV_STL_TRACE_FILE=<path>`, the
    parser/serialiser writes one JSONL event per state transition:
    `header` (one), `triangle_count` (one), `triangle` (one per
    triangle, in input order), `done` (one). ASCII vs binary trace
    tapes carry distinct fields (`format`, `name`, `attribute_bytes`,
    `header_hex`) so a `jq -c .` line-diff against another impl is
    character-equal where the underlying bytes are.
  - With the feature OFF the module compiles to nothing; release
    build pays zero cost. With the feature ON but the env var unset,
    every emit becomes a single `Option::is_some` check.
  - Test-only `trace::set_thread_trace_path` lets parallel tests pin
    a per-thread tape rather than racing on the env var.

- Round 2 — fuzz-resistant ASCII-vs-binary detection.
  - Optional UTF-8 BOM is stripped before sniffing the prefix.
  - Leading ASCII whitespace (spaces, tabs, CR, LF) is also stripped
    so AutoCAD-style ASCII files with a stray leading newline are
    classified correctly.
  - Prefix check now requires `b"solid"` followed by an ASCII
    whitespace byte; `b"solidWORKS"` and similar SolidWorks-style
    binary headers are rejected at the prefix stage.
  - **Binary-size cross-check** — even when the prefix + `\nfacet`
    token sniff both pass, the detector overrides to binary if the
    file length matches `84 + N*50` for the LE `u32` count at offset
    80 (and `N > 0`). This catches CADKey-2002 / Microsoft / AutoCAD
    binary headers that happen to embed a `\n facet` substring inside
    the 80-byte vendor string.
  - ASCII parser additionally strips a UTF-8 BOM at the byte-slice
    level so callers that bypass the detector still get a clean
    parse.

- Round 1 — initial bootstrap.
  - `StlDecoder` + `StlEncoder` implementing
    `oxideav_mesh3d::Mesh3DDecoder` / `Mesh3DEncoder` for both ASCII
    and binary STL.
  - Format auto-detection: ASCII vs binary distinguished by the first
    six bytes (`b"solid "`) cross-checked against a `\nfacet` /
    `\n  facet` token search in the first 1 KiB. Defends against the
    real-world trap of binary STL files whose 80-byte header begins
    with the literal `solid` ASCII string (some Microsoft toolchains).
  - One mesh per file → single root `Node` carrying a single
    `Triangles` primitive. Per-face normals expanded to per-vertex by
    duplication (3 copies per face), so downstream renderers don't
    need to reconstruct normals from winding.
  - Coordinate metadata: `Scene3D::up_axis = Axis::PosZ` (STL is Z-up
    per CAD/3D-printing convention) + `Scene3D::unit = Unit::Millimetres`
    (STL has no unit field, so the most-common slicer convention is
    chosen — downstream renderers that want metres scale by
    `Unit::Millimetres.to_metres() == 0.001`).
  - Per-face attribute byte preservation: when a binary STL's
    `uint16` attribute byte count is non-zero (Materialise / VisCAM
    colour conventions stash data here), the raw bytes are surfaced
    on the parsed `Mesh::extras["stl:per_face_attributes"]` as a hex
    string and round-tripped on encode.
  - Topology rejection: encoding a `Scene3D` whose mesh primitives
    use anything other than `Triangles` returns
    `Error::Unsupported("STL only supports Triangles topology; got <other>")`.
  - Standalone build path (`--no-default-features`) drops the
    `oxideav-core` dependency entirely; trait impls / registry
    helpers are re-exported through `oxideav-mesh3d`'s own standalone
    feature set so the surface stays consistent.
  - `register(&mut Mesh3DRegistry)` entry point under the default
    `registry` feature wires the decoder + encoder into the framework
    registry under format id `"stl"` with extension `"stl"`.

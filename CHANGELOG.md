# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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

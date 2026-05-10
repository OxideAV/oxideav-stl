# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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

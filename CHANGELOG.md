# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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

# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

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

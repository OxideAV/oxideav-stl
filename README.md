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

## Round 3 candidates

- Materialise binary-header `COLOR=R G B A` / `MATERIAL=…` line →
  `Mesh::extras["stl:default_color"]` so per-object default lights
  up the slots that the per-face Materialise convention encodes as
  "use default" (`valid=0`).
- ASCII tolerance: empty `solid` / `endsolid` lines without the
  trailing name; multiple `solid` blocks in a single file (rare but
  seen from CAD exports).
- Vertex deduplication on encode (currently emits 3 unique vertices
  per triangle even when the input mesh had a shared index buffer).
- ASCII pretty-printer with configurable float precision (currently
  defaults to single-precision `{}` formatting).
- Trace-tape companion docs (`docs/3d/stl/trace-contract.md`) so
  cross-impl auditors can lockstep against the schema without
  reading source.

## License

MIT — see [LICENSE](LICENSE).

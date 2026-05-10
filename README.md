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

## ASCII float-precision knob

By default the ASCII encoder uses Rust's round-trip-safe `{}`
formatter for f32 values. Switch to fixed-decimal output for
human-readable diffs:

```rust
use oxideav_stl::StlEncoder;

let _enc = StlEncoder::new_ascii().with_float_precision(Some(6));
// vertex 0.123457 0.000000 0.000000
```

Has no effect on binary output.

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
canonical count — fine for diagnostic use; large-mesh callers should
spatially index upstream.

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

## Trace tape (cross-impl audit)

With the `trace` Cargo feature enabled and `OXIDEAV_STL_TRACE_FILE`
pointing at a writable path, the codec emits one JSON-Lines event per
state transition (header / triangle_count / triangle / done). The
field schema, ordering invariants, and worked example are documented
in `docs/trace-contract.md`; cross-implementation auditors can
lockstep against that schema without reading source.

## Round 5 candidates

- Vertex-share-stats trace events (so the `trace` tape carries the
  same diagnostic signal that `EncodeStats` does).
- ASCII-mode auto-inject hook parity (currently `apply_pre_encode_extras`
  is format-agnostic — exercise it through the ASCII path explicitly
  in tests).

## License

MIT — see [LICENSE](LICENSE).

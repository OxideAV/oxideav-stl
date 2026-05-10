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

The first six bytes of the file are sniffed:

- Starts with `b"solid "` and a `\nfacet` / `\n  facet` token appears
  in the first 1 KiB → **ASCII**.
- Otherwise → **binary**, including the real-world trap of binary
  files whose 80-byte vendor header begins with the literal `solid`
  ASCII string.

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

## Round 2 candidates

- ASCII tolerance: tabs allowed in indentation (the spec disallows
  them but real-world files use them everywhere); empty `solid` /
  `endsolid` lines without the trailing name; multiple `solid`
  blocks in a single file (rare but seen from CAD exports).
- Materialise vendor extension: `COLOR=…` comment in the 80-byte
  binary header → `Material` with base-colour swatch.
- VisCAM / SolidView vendor extension: 15-bit packed RGB in the
  per-face `uint16` attribute count → per-face `Material` array.
- Vertex deduplication on encode (currently emits 3 unique vertices
  per triangle even when the input mesh had a shared index buffer).
- ASCII pretty-printer with configurable float precision (currently
  defaults to single-precision `{}` formatting).

## License

MIT — see [LICENSE](LICENSE).

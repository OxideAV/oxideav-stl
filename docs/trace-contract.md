# oxideav-stl trace-tape contract

JSON-Lines event vocabulary emitted by the `oxideav-stl` codec when
built with the `trace` Cargo feature and the
`OXIDEAV_STL_TRACE_FILE` env var (or the per-thread override
`oxideav_stl::trace::set_thread_trace_path`) names a writable path.
Each decode (or encode) invocation truncates the file on open and
writes one JSON object per state transition, terminated by a single
`\n`. Field order in the serialised JSON mirrors the order documented
below so a `jq -c .` line-diff against another implementation's tape
is character-equal where the underlying byte stream is.

This document exists so cross-implementation auditors can lockstep
against the schema without reading source. The companion
source-of-truth lives in `src/trace.rs` (the doc-comment at the top
of that file enumerates the same event vocabulary inline).

## Event vocabulary

Every event is a single JSON object on its own line. The `kind` field
discriminates; remaining fields appear in the order documented below.

### `header`

Emitted exactly once per decode/encode call, before any
`triangle_count` or `triangle` event. The fields differ slightly
between ASCII and binary because the two encodings carry different
metadata in the header position.

| field | type | binary | ascii | meaning |
|-------|------|--------|-------|---------|
| `kind` | string | `"header"` | `"header"` | always present, always first |
| `format` | string | `"binary"` | `"ascii"` | encoding tag |
| `byte_len` | integer | total file length | total file length | input/output bytes |
| `header_hex` | string (lowercase hex) | 80-byte vendor header bytes | omitted | binary STL header padding |
| `name` | string | omitted | `<name>` after `solid` | ASCII STL `solid` block name |

`header_hex` is exactly 160 lowercase hex characters (80 bytes × 2)
when present. `name` is omitted (rather than empty-string) when the
ASCII `solid` keyword is not followed by a name.

### `triangle_count`

Emitted exactly once per decode/encode call, after `header` and
before any `triangle` event.

| field | type | meaning |
|-------|------|---------|
| `kind` | string | `"triangle_count"` |
| `count` | integer | total triangle count |

For binary STL on decode, this is read directly from the LE u32 at
offset 80. For ASCII STL on decode, the count is computed by walking
all `facet` blocks and emitted at the same logical position so the
event ordering matches binary.

### `triangle`

Emitted once per triangle, in input order (`index = 0` for the first
triangle, monotonically increasing by one).

| field | type | binary | ascii | meaning |
|-------|------|--------|-------|---------|
| `kind` | string | `"triangle"` | `"triangle"` | |
| `index` | integer | 0..N-1 | 0..N-1 | triangle ordinal |
| `normal` | `[f32; 3]` | per-record normal | per-record normal | facet normal |
| `v0` | `[f32; 3]` | first vertex | first vertex | |
| `v1` | `[f32; 3]` | second vertex | second vertex | |
| `v2` | `[f32; 3]` | third vertex | third vertex | |
| `attribute_bytes` | string (lowercase hex, exactly 4 chars) | the two attribute-slot bytes (lo, hi) | omitted | ASCII has no attribute slot |

`f32` array elements use Rust's default `Display` formatter (round-
trip-safe). Non-finite components are serialised as JSON `null`.

### `done`

Emitted exactly once per decode/encode call, after the final
`triangle` event.

| field | type | meaning |
|-------|------|---------|
| `kind` | string | `"done"` |
| `source` | string | `"binary"` or `"ascii"` (matches the `header` event's `format`) |
| `triangles_emitted` | integer | total triangles processed (matches the `triangle_count` event's `count`) |

## Ordering invariants

The complete event sequence for any successful decode/encode call is:

```text
header
triangle_count
triangle (×N, with index 0..N-1 in monotonically-increasing order)
done
```

A failed (truncated, malformed) decode may stop emitting at any point;
auditors should treat a missing `done` event as "incomplete tape" and
not assume the partial data is consistent.

For multi-`solid` ASCII files, only one `header` event fires (with
the FIRST `solid` block's name). Per-block names beyond the first
are reflected via `Mesh::name` on the parsed `Scene3D`, not via
additional trace events — the trace tape is a flat triangle stream
across all blocks, indexed monotonically from 0.

## Field encoding details

- All event objects are valid JSON (RFC 8259) with no leading or
  trailing whitespace inside the line.
- Lines are terminated by `\n` only (no CR).
- Hex strings (`header_hex`, `attribute_bytes`) are lowercase only.
- String fields with non-ASCII control chars are escaped with the
  RFC 8259 §7 minimal-escape convention (`\"`, `\\`, `\n`, `\r`,
  `\t`, `\u00XX` for everything below 0x20).
- `f32` array elements rely on Rust's `Display`-formatted shortest-
  round-trip representation: `0.5_f32` serialises as `0.5`, not
  `0.500000`. Non-finite (`NaN`, `±Inf`) becomes JSON `null`.

## Cost discipline

With the `trace` Cargo feature OFF the entire trace module compiles
to nothing; release builds pay zero cost. With the feature ON but no
trace path configured, every emit site is a single
`Option::is_some` check on the cold path. The trace tape is
best-effort observability — I/O errors during emit are silently
dropped so the trace tape can never block a real-world decode.

## Configuration

Two ways to point the codec at a writable trace file (the per-thread
override takes precedence, so parallel tests can pin per-test tapes
without racing on a process-global env var):

- **Env var** — `OXIDEAV_STL_TRACE_FILE=/path/to/tape.jsonl` set
  before the decoder/encoder is invoked.
- **Per-thread override** — `oxideav_stl::trace::set_thread_trace_path(Some(path))`
  (test-only convenience; the function is feature-gated behind
  `trace` and not part of the production-stable surface).

Each invocation truncates the named file on open, so the trace tape
always reflects the most recent operation only. To preserve traces
across multiple operations, the caller must rotate the file path
externally between calls.

## Worked example — binary STL, single triangle

A 134-byte binary STL (80-byte header + 4-byte count + 50-byte
triangle) with header bytes all-zero, count = 1, normal `(0,0,1)`,
vertices `(0,0,0)`, `(1,0,0)`, `(0,1,0)`, attribute bytes `0x12 0x34`
emits four lines:

```jsonl
{"kind":"header","format":"binary","byte_len":134,"header_hex":"0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"}
{"kind":"triangle_count","count":1}
{"kind":"triangle","index":0,"normal":[0,0,1],"v0":[0,0,0],"v1":[1,0,0],"v2":[0,1,0],"attribute_bytes":"1234"}
{"kind":"done","source":"binary","triangles_emitted":1}
```

(Wrapped to 80 columns above for readability; the actual tape has
no soft-wrapping inside any single event line.)

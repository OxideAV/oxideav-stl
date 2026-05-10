//! JSON-Lines trace emitter for cross-implementation lockstep audit.
//!
//! Compiled in only when the `trace` Cargo feature is enabled. When
//! the feature is on AND the env var `OXIDEAV_STL_TRACE_FILE` is set
//! to a writable path, the parser/serialiser emits one JSONL event per
//! state transition to that path.
//!
//! ## Event vocabulary
//!
//! Every event line is a single JSON object with a `"kind"` field:
//!
//! | `kind`           | Fields                                                                                          | Meaning                                                  |
//! |------------------|-------------------------------------------------------------------------------------------------|----------------------------------------------------------|
//! | `header`         | `format`, `byte_len`, `header_hex` (binary), `name` (ascii)                                     | One per decode/encode invocation, before any triangles   |
//! | `triangle_count` | `count`                                                                                         | Binary STL: from offset 80 u32. ASCII: emitted at end    |
//! | `triangle`       | `index`, `normal`, `v0`, `v1`, `v2`, `attribute_bytes` (binary)                                 | One per triangle, in input order                          |
//! | `share_stats`    | `triangles`, `emitted_vertices`, `unique_vertices`, `share_factor`, `tolerance_eps`             | Encoder-only: emitted just before `done` when a `Scene3D` summary is available |
//! | `done`           | `source`, `triangles_emitted`                                                                   | One at the end of the operation                          |
//!
//! Field order in the serialised JSON mirrors the table above so that
//! a `jq -c .` line-diff against another implementation's tape is
//! character-equal where the underlying byte stream is.
//!
//! ## Cost discipline
//!
//! With the feature OFF, this module is empty and every call site is
//! gated behind `#[cfg(feature = "trace")]`, so the release build
//! pays zero cost. With the feature ON but the env var unset,
//! [`Tracer::from_env`] returns `None` and every emit becomes a single
//! `Option::is_some` check on the cold path.

#![cfg(feature = "trace")]

use std::cell::RefCell;
use std::env;
use std::fmt::Write as _;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::PathBuf;

thread_local! {
    /// Per-thread override for the trace path. When set, takes
    /// precedence over `OXIDEAV_STL_TRACE_FILE`. Pin from
    /// [`set_thread_trace_path`] inside parallel tests so each
    /// test's tape goes to its own file rather than racing on a
    /// process-global env var.
    static TRACE_PATH_OVERRIDE: RefCell<Option<PathBuf>> = const { RefCell::new(None) };
}

/// Pin a per-thread override for the trace path. Pass `None` to
/// clear.
///
/// Test-only convenience — production users should use
/// `OXIDEAV_STL_TRACE_FILE`.
#[allow(dead_code)] // referenced from tests behind the same feature gate
pub fn set_thread_trace_path(path: Option<PathBuf>) {
    TRACE_PATH_OVERRIDE.with(|cell| *cell.borrow_mut() = path);
}

/// Per-decode (or per-encode) tape. Constructed by the caller, threaded
/// through the parser/serialiser, dropped at the end. Each method
/// flushes the underlying writer before returning so a partial tape
/// after a panic is still usable for diagnosis up to the last
/// completed event.
#[derive(Debug)]
pub struct Tracer {
    writer: RefCell<BufWriter<File>>,
}

impl Tracer {
    /// Open the trace file named by the per-thread override (if set)
    /// or the `OXIDEAV_STL_TRACE_FILE` env var. The file is truncated
    /// on open so each decode/encode produces a fresh tape.
    ///
    /// Returns `None` if no path is set, the path is empty, or the
    /// file cannot be created. All errors are best-effort (a failed
    /// open silently disables tracing) so the trace tape can never
    /// block a real-world decode.
    pub fn from_env() -> Option<Self> {
        let path: Option<PathBuf> = TRACE_PATH_OVERRIDE
            .with(|cell| cell.borrow().clone())
            .or_else(|| env::var_os("OXIDEAV_STL_TRACE_FILE").map(PathBuf::from));
        let path = path?;
        if path.as_os_str().is_empty() {
            return None;
        }
        let file = File::create(&path).ok()?;
        Some(Self {
            writer: RefCell::new(BufWriter::new(file)),
        })
    }

    /// Write one JSONL event line. I/O errors are silently dropped —
    /// the trace tape is best-effort observability.
    pub fn emit(&self, ev: Event<'_>) {
        let mut s = String::with_capacity(192);
        ev.write_to(&mut s);
        s.push('\n');
        let mut w = self.writer.borrow_mut();
        let _ = w.write_all(s.as_bytes());
    }
}

impl Drop for Tracer {
    fn drop(&mut self) {
        let _ = self.writer.borrow_mut().flush();
    }
}

/// Format tag for the `header` event.
#[derive(Clone, Copy, Debug)]
pub enum Format {
    Ascii,
    Binary,
}

impl Format {
    fn as_str(&self) -> &'static str {
        match self {
            Format::Ascii => "ascii",
            Format::Binary => "binary",
        }
    }
}

/// One JSONL event — exactly one JSON object on its own line.
#[derive(Debug)]
pub enum Event<'a> {
    Header {
        format: Format,
        byte_len: usize,
        /// Binary STL: 80-byte vendor header as lowercase hex.
        /// ASCII STL: leave as `None`.
        header_hex: Option<&'a [u8]>,
        /// ASCII STL: optional `<name>` after `solid`.
        /// Binary STL: leave as `None`.
        name: Option<&'a str>,
    },
    TriangleCount {
        count: usize,
    },
    Triangle {
        index: usize,
        normal: [f32; 3],
        v0: [f32; 3],
        v1: [f32; 3],
        v2: [f32; 3],
        /// Binary STL: the two attribute-slot bytes (lo, hi).
        /// ASCII STL: leave as `None`.
        attribute_bytes: Option<[u8; 2]>,
    },
    /// Encoder-only summary of the vertex-share statistics computed
    /// over the about-to-be-emitted [`oxideav_mesh3d::Scene3D`].
    ///
    /// Emitted between the final `triangle` event and the `done` event
    /// so a downstream auditor processing the JSONL tape sequentially
    /// can pick up the summary without re-scanning the geometry.
    /// `tolerance_eps` is `None` when the unique-vertex count was
    /// computed under the bit-exact `f32` rule (the encoder's own
    /// pre-emit summary always uses this), and `Some(eps)` for the
    /// tolerance-based variant.
    ShareStats {
        triangles: usize,
        emitted_vertices: usize,
        unique_vertices: usize,
        share_factor: f32,
        tolerance_eps: Option<f32>,
    },
    Done {
        source: Format,
        triangles_emitted: usize,
    },
}

impl<'a> Event<'a> {
    fn write_to(&self, out: &mut String) {
        match self {
            Event::Header {
                format,
                byte_len,
                header_hex,
                name,
            } => {
                out.push_str("{\"kind\":\"header\"");
                push_str_field(out, "format", format.as_str());
                push_int_field(out, "byte_len", *byte_len as u64);
                if let Some(bytes) = header_hex {
                    push_hex_field(out, "header_hex", bytes);
                }
                if let Some(n) = name {
                    push_str_field(out, "name", n);
                }
                out.push('}');
            }
            Event::TriangleCount { count } => {
                out.push_str("{\"kind\":\"triangle_count\"");
                push_int_field(out, "count", *count as u64);
                out.push('}');
            }
            Event::Triangle {
                index,
                normal,
                v0,
                v1,
                v2,
                attribute_bytes,
            } => {
                out.push_str("{\"kind\":\"triangle\"");
                push_int_field(out, "index", *index as u64);
                push_vec3_field(out, "normal", *normal);
                push_vec3_field(out, "v0", *v0);
                push_vec3_field(out, "v1", *v1);
                push_vec3_field(out, "v2", *v2);
                if let Some(ab) = attribute_bytes {
                    push_hex_field(out, "attribute_bytes", ab);
                }
                out.push('}');
            }
            Event::ShareStats {
                triangles,
                emitted_vertices,
                unique_vertices,
                share_factor,
                tolerance_eps,
            } => {
                out.push_str("{\"kind\":\"share_stats\"");
                push_int_field(out, "triangles", *triangles as u64);
                push_int_field(out, "emitted_vertices", *emitted_vertices as u64);
                push_int_field(out, "unique_vertices", *unique_vertices as u64);
                // share_factor follows the same finite/non-finite contract
                // as the per-vertex coordinates — non-finite ⇒ JSON null.
                out.push(',');
                out.push_str("\"share_factor\":");
                push_f32(out, *share_factor);
                out.push(',');
                out.push_str("\"tolerance_eps\":");
                match tolerance_eps {
                    Some(eps) => push_f32(out, *eps),
                    None => out.push_str("null"),
                }
                out.push('}');
            }
            Event::Done {
                source,
                triangles_emitted,
            } => {
                out.push_str("{\"kind\":\"done\"");
                push_str_field(out, "source", source.as_str());
                push_int_field(out, "triangles_emitted", *triangles_emitted as u64);
                out.push('}');
            }
        }
    }
}

fn push_str_field(out: &mut String, key: &str, val: &str) {
    out.push(',');
    out.push('"');
    out.push_str(key);
    out.push_str("\":");
    push_json_string(out, val);
}

fn push_int_field(out: &mut String, key: &str, val: u64) {
    out.push(',');
    out.push('"');
    out.push_str(key);
    out.push_str("\":");
    let _ = write!(out, "{}", val);
}

fn push_hex_field(out: &mut String, key: &str, bytes: &[u8]) {
    out.push(',');
    out.push('"');
    out.push_str(key);
    out.push_str("\":\"");
    for b in bytes {
        let _ = write!(out, "{:02x}", b);
    }
    out.push('"');
}

fn push_vec3_field(out: &mut String, key: &str, v: [f32; 3]) {
    out.push(',');
    out.push('"');
    out.push_str(key);
    out.push_str("\":[");
    push_f32(out, v[0]);
    out.push(',');
    push_f32(out, v[1]);
    out.push(',');
    push_f32(out, v[2]);
    out.push(']');
}

/// Emit an f32 with Rust's default `Display` (round-trip-safe). Non-
/// finite values become `null` so downstream JSON parsers don't gag.
fn push_f32(out: &mut String, v: f32) {
    if v.is_finite() {
        let _ = write!(out, "{}", v);
    } else {
        out.push_str("null");
    }
}

/// Append a JSON-quoted string literal. Escapes `\"`, `\\`, control
/// characters per RFC 8259 §7. ASCII printables pass through; non-
/// printables become `\u00XX`.
fn push_json_string(out: &mut String, s: &str) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn header_event_serialises_with_hex() {
        let mut s = String::new();
        let header = [0xab, 0xcd];
        Event::Header {
            format: Format::Binary,
            byte_len: 134,
            header_hex: Some(&header),
            name: None,
        }
        .write_to(&mut s);
        assert_eq!(
            s,
            r#"{"kind":"header","format":"binary","byte_len":134,"header_hex":"abcd"}"#
        );
    }

    #[test]
    fn header_event_serialises_ascii_with_name() {
        let mut s = String::new();
        Event::Header {
            format: Format::Ascii,
            byte_len: 200,
            header_hex: None,
            name: Some("cube"),
        }
        .write_to(&mut s);
        assert_eq!(
            s,
            r#"{"kind":"header","format":"ascii","byte_len":200,"name":"cube"}"#
        );
    }

    #[test]
    fn triangle_event_serialises_in_field_order() {
        let mut s = String::new();
        Event::Triangle {
            index: 0,
            normal: [0.0, 0.0, 1.0],
            v0: [0.0, 0.0, 0.0],
            v1: [1.0, 0.0, 0.0],
            v2: [0.0, 1.0, 0.0],
            attribute_bytes: Some([0x12, 0x34]),
        }
        .write_to(&mut s);
        assert_eq!(
            s,
            r#"{"kind":"triangle","index":0,"normal":[0,0,1],"v0":[0,0,0],"v1":[1,0,0],"v2":[0,1,0],"attribute_bytes":"1234"}"#
        );
    }

    #[test]
    fn triangle_count_event_minimal() {
        let mut s = String::new();
        Event::TriangleCount { count: 12 }.write_to(&mut s);
        assert_eq!(s, r#"{"kind":"triangle_count","count":12}"#);
    }

    #[test]
    fn share_stats_event_serialises_with_null_tolerance() {
        let mut s = String::new();
        Event::ShareStats {
            triangles: 12,
            emitted_vertices: 36,
            unique_vertices: 8,
            share_factor: 4.5,
            tolerance_eps: None,
        }
        .write_to(&mut s);
        assert_eq!(
            s,
            r#"{"kind":"share_stats","triangles":12,"emitted_vertices":36,"unique_vertices":8,"share_factor":4.5,"tolerance_eps":null}"#
        );
    }

    #[test]
    fn share_stats_event_serialises_with_tolerance() {
        let mut s = String::new();
        Event::ShareStats {
            triangles: 3,
            emitted_vertices: 9,
            unique_vertices: 3,
            share_factor: 3.0,
            tolerance_eps: Some(1.0e-5),
        }
        .write_to(&mut s);
        // 1e-5 is exactly representable as the Display form `0.00001`.
        assert_eq!(
            s,
            r#"{"kind":"share_stats","triangles":3,"emitted_vertices":9,"unique_vertices":3,"share_factor":3,"tolerance_eps":0.00001}"#
        );
    }

    #[test]
    fn done_event_carries_source_tag() {
        let mut s = String::new();
        Event::Done {
            source: Format::Binary,
            triangles_emitted: 12,
        }
        .write_to(&mut s);
        assert_eq!(
            s,
            r#"{"kind":"done","source":"binary","triangles_emitted":12}"#
        );
    }

    #[test]
    fn json_string_escapes_control_chars_and_quotes() {
        let mut s = String::new();
        push_json_string(&mut s, "ab\"\n\\c");
        assert_eq!(s, r#""ab\"\n\\c""#);
    }

    #[test]
    fn f32_non_finite_becomes_null() {
        let mut s = String::new();
        push_f32(&mut s, f32::NAN);
        assert_eq!(s, "null");
        let mut s = String::new();
        push_f32(&mut s, f32::INFINITY);
        assert_eq!(s, "null");
    }
}

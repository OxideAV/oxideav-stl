//! ASCII STL parser + serializer.
//!
//! Grammar (per Marshall Burns' transcription, §6.5.2):
//!
//! ```text
//! solid <name>?
//!   { facet normal nx ny nz
//!       outer loop
//!         vertex x y z
//!         vertex x y z
//!         vertex x y z
//!       endloop
//!     endfacet }+
//! endsolid <name>?
//! ```
//!
//! Bold-face keywords (`solid`, `endsolid`, `facet`, `normal`,
//! `outer`, `loop`, `vertex`, `endloop`, `endfacet`) MUST appear in
//! lower case per the spec; we accept any-case to match the
//! prevailing real-world tolerance. Indentation is "with spaces; tabs
//! are not allowed" per spec; we accept tabs too because nearly every
//! authoring tool emits them.
//!
//! The `<name>` after `solid` and `endsolid` is optional; we
//! preserve it on the parsed [`Mesh::name`] when present.
//!
//! ## Comment-line tolerance (vendor quirk, r7)
//!
//! Some hand-edited ASCII STL files (and a handful of vintage CAD
//! exporters) prepend a `;`-introduced line comment to the file
//! header or interleave comments between `endfacet` and the next
//! `facet`. The 1989 spec defines no comment syntax, but the dominant
//! real-world tolerance is to skip whole-line comments starting with
//! `;` (sometimes `#`). We accept both characters as line-comment
//! introducers; the rest of the line up to the next `\n` is silently
//! discarded. Inline comments mid-token are NOT tolerated — that
//! would conflict with token-recognition for `solid` / `facet` /
//! `vertex` and would silently change the meaning of legitimate
//! numerals when an editor inserted a `;` at the wrong column.

use std::collections::HashMap;
use std::fmt::Write as _;

use oxideav_mesh3d::{Axis, Error, Mesh, Node, Primitive, Result, Scene3D, Topology, Unit};

/// Parse an ASCII STL byte slice into a [`Scene3D`].
///
/// Some CAD exporters (older Pro/E, AutoCAD, hand-edited files)
/// concatenate multiple `solid NAME … endsolid NAME` blocks into a
/// single `.stl` file. The strict 1989 spec defines exactly one
/// `solid` block per file but the de-facto tolerance across modern
/// readers is to accept additional blocks back-to-back. We follow that
/// tolerance: each `solid` block becomes its own [`Mesh`] in the
/// resulting [`Scene3D`], with one [`Node`] per mesh attached to the
/// scene root in source order.
pub fn decode(bytes: &[u8]) -> Result<Scene3D> {
    // ASCII STL is restricted to printable ASCII + standard whitespace
    // by the spec; we tolerate UTF-8 in the optional `<name>` field via
    // a lossy decode, since real-world files do ship non-ASCII names.
    // Strip an optional UTF-8 BOM first — Windows-side text editors
    // sometimes prepend one and the decoder shouldn't trip on it.
    let bytes = bytes.strip_prefix(&[0xEF, 0xBB, 0xBF]).unwrap_or(bytes);
    let text = std::str::from_utf8(bytes)
        .map_err(|e| Error::InvalidData(format!("ASCII STL is not valid UTF-8: {e}")))?;

    let mut p = Parser::new(text);

    #[cfg(feature = "trace")]
    let tracer = crate::trace::Tracer::from_env();
    #[cfg(feature = "trace")]
    let mut tri_index: usize = 0;
    // For the trace `header` event we keep the name of the *first*
    // solid block (consistent with single-solid behaviour). Multi-solid
    // names beyond the first are reflected via per-mesh `Mesh::name`.
    #[cfg(feature = "trace")]
    let mut emitted_header = false;

    let mut scene = Scene3D::new();
    scene.up_axis = Axis::PosZ;
    scene.unit = Unit::Millimetres;

    let mut block_count = 0usize;
    loop {
        p.skip_ws();
        if p.is_eof() {
            // No further `solid` block — clean end.
            break;
        }
        if !p.peek_keyword_eq("solid") {
            // Stray garbage between blocks — strict-mode reject.
            // (Trailing newlines etc. were already consumed by skip_ws.)
            let snippet: String = p.src[p.pos..]
                .chars()
                .take(16)
                .filter(|c| !c.is_control())
                .collect();
            return Err(Error::InvalidData(format!(
                "ASCII STL: expected `solid` or end-of-file, got `{snippet}`"
            )));
        }
        p.expect_keyword("solid")?;
        let name = p.read_optional_line_remainder();

        #[cfg(feature = "trace")]
        if !emitted_header {
            if let Some(t) = tracer.as_ref() {
                t.emit(crate::trace::Event::Header {
                    format: crate::trace::Format::Ascii,
                    byte_len: bytes.len(),
                    header_hex: None,
                    name: name.as_deref(),
                });
            }
            emitted_header = true;
        }

        let mut positions: Vec<[f32; 3]> = Vec::new();
        let mut normals: Vec<[f32; 3]> = Vec::new();

        loop {
            p.skip_ws();
            if p.peek_keyword_eq("endsolid") {
                p.expect_keyword("endsolid")?;
                // Consume optional trailing name on `endsolid`.
                let _ = p.read_optional_line_remainder();
                break;
            }
            // Otherwise expect a `facet normal nx ny nz` block.
            p.expect_keyword("facet")?;
            p.skip_ws();
            p.expect_keyword("normal")?;
            let n = [p.read_float()?, p.read_float()?, p.read_float()?];

            p.skip_ws();
            p.expect_keyword("outer")?;
            p.skip_ws();
            p.expect_keyword("loop")?;

            // Read three vertices in source order. We capture them into
            // local bindings (rather than only `positions.push`) so the
            // trace emitter can see them.
            p.skip_ws();
            p.expect_keyword("vertex")?;
            let v0 = [p.read_float()?, p.read_float()?, p.read_float()?];
            p.skip_ws();
            p.expect_keyword("vertex")?;
            let v1 = [p.read_float()?, p.read_float()?, p.read_float()?];
            p.skip_ws();
            p.expect_keyword("vertex")?;
            let v2 = [p.read_float()?, p.read_float()?, p.read_float()?];
            positions.push(v0);
            normals.push(n);
            positions.push(v1);
            normals.push(n);
            positions.push(v2);
            normals.push(n);

            #[cfg(feature = "trace")]
            if let Some(t) = tracer.as_ref() {
                t.emit(crate::trace::Event::Triangle {
                    index: tri_index,
                    normal: n,
                    v0,
                    v1,
                    v2,
                    attribute_bytes: None,
                });
            }
            #[cfg(feature = "trace")]
            {
                tri_index += 1;
            }

            p.skip_ws();
            p.expect_keyword("endloop")?;
            p.skip_ws();
            p.expect_keyword("endfacet")?;
        }

        let mut prim_extras: HashMap<String, serde_json::Value> = HashMap::new();
        prim_extras.insert(
            "stl:source".to_string(),
            serde_json::Value::String("ascii".to_string()),
        );

        // Forward-compatible construction: oxideav-mesh3d's `Primitive`
        // is `#[non_exhaustive]`, so external crates must build via
        // `Primitive::new(Topology::*)` + per-field assignment rather
        // than struct-literal syntax.
        let mut primitive = Primitive::new(Topology::Triangles);
        primitive.positions = positions;
        primitive.normals = Some(normals);
        primitive.extras = prim_extras;

        // `Mesh` is `#[non_exhaustive]`; build via `Mesh::new` + the
        // `with_primitive` builder so we don't break when mesh3d adds
        // further fields in future minor releases.
        let mesh = Mesh::new(name.filter(|s| !s.is_empty())).with_primitive(primitive);
        let mesh_id = scene.add_mesh(mesh);
        let mut node = Node::new();
        node.mesh = Some(mesh_id);
        let node_id = scene.add_node(node);
        scene.add_root(node_id);
        block_count += 1;
    }

    if block_count == 0 {
        return Err(Error::InvalidData(
            "ASCII STL: no `solid` block found".into(),
        ));
    }

    #[cfg(feature = "trace")]
    if let Some(t) = tracer.as_ref() {
        t.emit(crate::trace::Event::TriangleCount { count: tri_index });
        t.emit(crate::trace::Event::Done {
            source: crate::trace::Format::Ascii,
            triangles_emitted: tri_index,
        });
    }
    Ok(scene)
}

/// Serialise a [`Scene3D`] as ASCII STL.
///
/// Multi-mesh scenes round-trip as multiple `solid NAME … endsolid
/// NAME` blocks back-to-back, mirroring the multi-solid tolerance of
/// [`decode`]. Single-mesh scenes still produce a single block. The
/// per-mesh `Mesh::name` (when set) drives the `solid` / `endsolid`
/// trailing name; meshes with no name emit a bare `solid` line.
pub fn encode(scene: &Scene3D) -> Result<Vec<u8>> {
    encode_with(scene, &EncodeOptions::default())
}

/// Float formatting policy for [`encode_with`].
///
/// Three flavours, in order of strictness:
///
/// 1. [`AsciiNumberFormat::RoundTrip`] (the default) — Rust's `{}`
///    formatter. Bit-exact round-trip on re-parse; not always the
///    most human-readable choice (`0.1_f32` prints as `0.1` but
///    `0.1_f32 + 0.2_f32` prints as `0.3` despite carrying a
///    representation-error LSB).
/// 2. [`AsciiNumberFormat::FixedDecimal { precision }`] — `{:.n}`
///    fixed-decimal output. Pretty for diffs, can lose precision
///    on the round-trip if `n` is small.
/// 3. [`AsciiNumberFormat::SpecScientific { precision }`] — emits the
///    `1.23456E+789` scientific-notation form the 1989 spec uses as
///    its worked example (mantissa + literal `E` + explicit
///    `+`/`-` sign + base-10 exponent). The strictest match to the
///    spec letter; least common in the real-world ecosystem.
///
/// `SpecScientific` is the only flavour that emits an explicit `+`
/// before non-negative exponents. Rust's native `{:E}` produces e.g.
/// `1.23E2` with no sign; the spec example has `1.23456E+789` and the
/// vast majority of 1989-era StL-writing tools followed that
/// convention. Round-trip-equivalent for any consumer obeying the
/// spec's "single precision floats, for example, 1.23456E+789"
/// guidance; non-conformant parsers that reject `E+nnn` should not be
/// asked to read this output.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum AsciiNumberFormat {
    /// Rust's round-trip-safe `{}` formatter. Default.
    #[default]
    RoundTrip,
    /// Fixed-decimal `{:.precision}` formatter — `precision` digits
    /// after the point. Compact, diff-friendly, lossy if `precision`
    /// is small.
    FixedDecimal {
        /// Decimal digits after the point.
        precision: usize,
    },
    /// Spec-style scientific `mantissa.E[+-]exponent` formatter —
    /// matches the 1989 spec's `1.23456E+789` worked example
    /// verbatim. `precision` digits after the point in the mantissa.
    SpecScientific {
        /// Mantissa digits after the point.
        precision: usize,
    },
}

/// Float formatting precision for [`encode_with`]. Defaults to
/// Rust's `{}` round-trip formatting; pass `Some(n)` to get
/// `{:.n}` fixed-decimal output (e.g. for human-readable diffs).
///
/// The historical `float_precision` knob composes with the newer
/// [`number_format`](Self::number_format) field as follows:
///
/// - `number_format == RoundTrip` + `float_precision == Some(n)` →
///   `FixedDecimal { precision: n }` (backward-compatible: the
///   round-1 callers built `EncodeOptions { float_precision: Some(6),
///   .. }` and got fixed-decimal output, which we preserve).
/// - `number_format != RoundTrip` → `number_format` wins, ignoring
///   `float_precision`.
#[derive(Clone, Copy, Debug, Default)]
pub struct EncodeOptions {
    /// Number of decimal digits after the point. `None` =
    /// round-trip-safe `{}`. `Some(n)` = `{:.n}`. Composes with
    /// [`Self::number_format`] (see the type doc).
    pub float_precision: Option<usize>,
    /// New (round-9) policy switch — [`AsciiNumberFormat`] selects
    /// between the round-trip, fixed-decimal, and spec-style
    /// scientific output forms. Defaults to round-trip.
    pub number_format: AsciiNumberFormat,
}

impl EncodeOptions {
    /// Convenience: build with a fixed decimal precision.
    pub fn with_float_precision(precision: usize) -> Self {
        Self {
            float_precision: Some(precision),
            number_format: AsciiNumberFormat::default(),
        }
    }

    /// Convenience: build with the spec-style scientific number format.
    pub fn with_spec_scientific(precision: usize) -> Self {
        Self {
            float_precision: None,
            number_format: AsciiNumberFormat::SpecScientific { precision },
        }
    }

    /// Resolve the effective number-formatting policy by combining
    /// `float_precision` and `number_format` per the composition rule
    /// documented on the struct.
    pub(crate) fn effective_format(&self) -> AsciiNumberFormat {
        match self.number_format {
            AsciiNumberFormat::RoundTrip => match self.float_precision {
                Some(n) => AsciiNumberFormat::FixedDecimal { precision: n },
                None => AsciiNumberFormat::RoundTrip,
            },
            other => other,
        }
    }
}

/// Serialise a [`Scene3D`] as ASCII STL using `opts`.
///
/// See [`EncodeOptions`] for the knobs. Behaviour is identical to
/// [`encode`] when `opts` is the default.
pub fn encode_with(scene: &Scene3D, opts: &EncodeOptions) -> Result<Vec<u8>> {
    let mut out = String::new();

    #[cfg(feature = "trace")]
    let tracer = crate::trace::Tracer::from_env();
    #[cfg(feature = "trace")]
    let mut tri_index: usize = 0;
    // The trace `header` event captures the *first* mesh's name —
    // multi-solid output reflects further names via the per-mesh
    // emitted bytes (visible to a downstream re-decode).
    #[cfg(feature = "trace")]
    let first_name: Option<&str> = scene
        .meshes
        .iter()
        .find_map(|m| m.name.as_deref())
        .filter(|s| !s.is_empty());
    #[cfg(feature = "trace")]
    if let Some(t) = tracer.as_ref() {
        t.emit(crate::trace::Event::Header {
            format: crate::trace::Format::Ascii,
            byte_len: 0, // unknown until done — emitter reads `out.len()` at end
            header_hex: None,
            name: first_name,
        });
    }

    // Multi-mesh scenes emit one `solid NAME … endsolid NAME` block
    // per mesh (preserving order); single-mesh / empty scenes still
    // produce a single block (the historical contract). When there are
    // zero meshes we still emit one empty block so the file is a valid
    // ASCII STL ("solid\nendsolid\n").
    let blocks: Vec<&Mesh> = if scene.meshes.is_empty() {
        Vec::new()
    } else {
        scene.meshes.iter().collect()
    };

    if blocks.is_empty() {
        out.push_str("solid\nendsolid\n");
    } else {
        for mesh in blocks {
            let name = mesh.name.as_deref().unwrap_or("");
            if name.is_empty() {
                out.push_str("solid\n");
            } else {
                let _ = writeln!(out, "solid {name}");
            }
            for prim in &mesh.primitives {
                if prim.topology != Topology::Triangles {
                    return Err(Error::Unsupported(format!(
                        "STL only supports Triangles topology; got {:?}",
                        prim.topology
                    )));
                }
                let face_count = match &prim.indices {
                    Some(idx) => idx.len() / 3,
                    None => prim.positions.len() / 3,
                };
                for face_idx in 0..face_count {
                    let (vi0, vi1, vi2) = match &prim.indices {
                        Some(oxideav_mesh3d::Indices::U16(v)) => {
                            let b = face_idx * 3;
                            (v[b] as usize, v[b + 1] as usize, v[b + 2] as usize)
                        }
                        Some(oxideav_mesh3d::Indices::U32(v)) => {
                            let b = face_idx * 3;
                            (v[b] as usize, v[b + 1] as usize, v[b + 2] as usize)
                        }
                        None => {
                            let b = face_idx * 3;
                            (b, b + 1, b + 2)
                        }
                    };
                    let v0 = prim.positions[vi0];
                    let v1 = prim.positions[vi1];
                    let v2 = prim.positions[vi2];
                    let n = match prim.normals.as_ref() {
                        Some(ns) if ns.len() == prim.positions.len() => ns[vi0],
                        _ => face_normal(v0, v1, v2),
                    };

                    #[cfg(feature = "trace")]
                    if let Some(t) = tracer.as_ref() {
                        t.emit(crate::trace::Event::Triangle {
                            index: tri_index,
                            normal: n,
                            v0,
                            v1,
                            v2,
                            attribute_bytes: None,
                        });
                    }
                    #[cfg(feature = "trace")]
                    {
                        tri_index += 1;
                    }

                    let _ = writeln!(
                        out,
                        "  facet normal {} {} {}",
                        fmt_f32_with(n[0], opts),
                        fmt_f32_with(n[1], opts),
                        fmt_f32_with(n[2], opts)
                    );
                    out.push_str("    outer loop\n");
                    let _ = writeln!(
                        out,
                        "      vertex {} {} {}",
                        fmt_f32_with(v0[0], opts),
                        fmt_f32_with(v0[1], opts),
                        fmt_f32_with(v0[2], opts)
                    );
                    let _ = writeln!(
                        out,
                        "      vertex {} {} {}",
                        fmt_f32_with(v1[0], opts),
                        fmt_f32_with(v1[1], opts),
                        fmt_f32_with(v1[2], opts)
                    );
                    let _ = writeln!(
                        out,
                        "      vertex {} {} {}",
                        fmt_f32_with(v2[0], opts),
                        fmt_f32_with(v2[1], opts),
                        fmt_f32_with(v2[2], opts)
                    );
                    out.push_str("    endloop\n");
                    out.push_str("  endfacet\n");
                }
            }
            if name.is_empty() {
                out.push_str("endsolid\n");
            } else {
                let _ = writeln!(out, "endsolid {name}");
            }
        }
    }

    #[cfg(feature = "trace")]
    if let Some(t) = tracer.as_ref() {
        t.emit(crate::trace::Event::TriangleCount { count: tri_index });
        // Bit-exact share-stats summary — emitted between
        // `triangle_count` and `done` so the JSONL tape carries the
        // EncodeStats signal natively without forcing a re-walk on
        // the auditor side. Bit-exact only (`tolerance_eps == None`);
        // tolerance variants live behind
        // `EncodeStats::with_tolerance` on the caller.
        let stats = crate::encoder::compute_stats(scene);
        t.emit(crate::trace::Event::ShareStats {
            triangles: stats.triangles,
            emitted_vertices: stats.emitted_vertices,
            unique_vertices: stats.unique_vertices,
            share_factor: stats.share_factor(),
            tolerance_eps: None,
        });
        t.emit(crate::trace::Event::Done {
            source: crate::trace::Format::Ascii,
            triangles_emitted: tri_index,
        });
    }

    Ok(out.into_bytes())
}

/// f32 formatter parameterised by [`EncodeOptions`].
///
/// Dispatches on [`EncodeOptions::effective_format`]:
/// - `RoundTrip` → Rust's `{}` (default).
/// - `FixedDecimal { precision }` → `{:.precision}`.
/// - `SpecScientific { precision }` → mantissa + `E` + explicit
///   `+`/`-` sign + base-10 exponent, matching the 1989 spec's
///   `1.23456E+789` worked example.
///
/// Non-finite values become `0` since STL has no representation for
/// NaN or Inf.
fn fmt_f32_with(v: f32, opts: &EncodeOptions) -> String {
    if !v.is_finite() {
        return "0".to_string();
    }
    match opts.effective_format() {
        AsciiNumberFormat::RoundTrip => format!("{v}"),
        AsciiNumberFormat::FixedDecimal { precision } => format!("{v:.precision$}"),
        AsciiNumberFormat::SpecScientific { precision } => fmt_spec_scientific(v, precision),
    }
}

/// Format `v` in the 1989 spec's `1.23456E+789` scientific-notation
/// flavour. `precision` digits after the point in the mantissa.
///
/// Rules:
/// - The literal letter is uppercase `E` (matching the spec example).
/// - The exponent always carries an explicit sign — `+` for
///   non-negative, `-` for negative — never bare digits. This is
///   what distinguishes the spec example (`1.23456E+789`) from
///   Rust's default `{:E}` formatter (`1.23456E789`).
/// - Zero is rendered as `0.000…E+0` with `precision` zero-digits
///   after the point. Rust's `{:E}` agrees on that form, save for
///   the sign.
///
/// This is the strict-spec output flavour; most consumers in the
/// wild also accept Rust's `{}` round-trip representation.
fn fmt_spec_scientific(v: f32, precision: usize) -> String {
    // We could rely on `{:E}` and patch the `Ennn` → `E+nnn` mapping
    // by hand, but Rust's `{:.p$E}` only adds the `+` sign in the
    // explicit-sign forms (`{:+E}` flips the *mantissa* sign, not the
    // exponent). Easiest is to lean on `{:.p$E}` and rewrite the
    // exponent token, which is everything after the trailing `E`.
    let basic = format!("{:.*E}", precision, v);
    // Split at the (single) `E`. Both halves are guaranteed because
    // `{:E}` always emits an `E`.
    let (mantissa, exp) = match basic.split_once('E') {
        Some(p) => p,
        // Defensive fall-through — should never happen given Rust's
        // {:E} contract, but if a future stdlib change breaks the
        // assumption we still emit a well-formed result.
        None => return format!("{}E+0", basic),
    };
    // Rust emits the exponent with a leading `-` for negative values
    // and no sign for non-negative. Normalise to always carry a sign.
    if let Some(rest) = exp.strip_prefix('-') {
        format!("{mantissa}E-{rest}")
    } else {
        // Includes the case where `exp` already starts with `+`
        // (no current Rust stdlib version does so, but be tolerant).
        let exp = exp.strip_prefix('+').unwrap_or(exp);
        format!("{mantissa}E+{exp}")
    }
}

type Vec3 = [f32; 3];

fn face_normal(a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
    let cx = u[1] * v[2] - u[2] * v[1];
    let cy = u[2] * v[0] - u[0] * v[2];
    let cz = u[0] * v[1] - u[1] * v[0];
    let len = (cx * cx + cy * cy + cz * cz).sqrt();
    if len > f32::EPSILON {
        [cx / len, cy / len, cz / len]
    } else {
        [0.0, 0.0, 0.0]
    }
}

/// Hand-rolled ASCII-STL tokeniser — the grammar is small enough that
/// pulling in `nom`/`logos` would be overkill, and we want zero
/// non-essential dependencies in this crate.
struct Parser<'a> {
    src: &'a str,
    pos: usize,
}

impl<'a> Parser<'a> {
    fn new(src: &'a str) -> Self {
        Self { src, pos: 0 }
    }

    /// Whether the parser has consumed all input.
    fn is_eof(&self) -> bool {
        self.pos >= self.src.len()
    }

    /// Skip ASCII whitespace including newlines + tabs, AND
    /// whole-line `;`/`#`-introduced comments. A comment runs from
    /// the introducer up to the next `\n` (or EOF); the introducer
    /// itself is only recognised at a position where the parser
    /// would otherwise expect a fresh token (i.e. after whitespace
    /// has been consumed), so the syntax has no chance of conflating
    /// with `vertex` / `facet` / coordinate-numeral tokens.
    fn skip_ws(&mut self) {
        let bytes = self.src.as_bytes();
        loop {
            // First: regular whitespace.
            while self.pos < bytes.len() {
                let b = bytes[self.pos];
                if b == b' ' || b == b'\t' || b == b'\r' || b == b'\n' {
                    self.pos += 1;
                } else {
                    break;
                }
            }
            // Then: if the next byte introduces a line comment, eat
            // the rest of the line and loop back for any trailing
            // whitespace + further comments. Otherwise we're at a
            // real token and bail.
            if self.pos < bytes.len() && (bytes[self.pos] == b';' || bytes[self.pos] == b'#') {
                while self.pos < bytes.len() && bytes[self.pos] != b'\n' {
                    self.pos += 1;
                }
            } else {
                break;
            }
        }
    }

    /// Read the next whitespace-delimited token (no leading-ws skip).
    fn read_token(&mut self) -> Option<&'a str> {
        let bytes = self.src.as_bytes();
        let start = self.pos;
        while self.pos < bytes.len() {
            let b = bytes[self.pos];
            if b == b' ' || b == b'\t' || b == b'\r' || b == b'\n' {
                break;
            }
            self.pos += 1;
        }
        if self.pos == start {
            None
        } else {
            Some(&self.src[start..self.pos])
        }
    }

    fn expect_keyword(&mut self, kw: &str) -> Result<()> {
        self.skip_ws();
        let tok = self.read_token().ok_or_else(|| {
            Error::InvalidData(format!("ASCII STL: expected `{kw}`, got end-of-file"))
        })?;
        if tok.eq_ignore_ascii_case(kw) {
            Ok(())
        } else {
            Err(Error::InvalidData(format!(
                "ASCII STL: expected `{kw}`, got `{tok}`"
            )))
        }
    }

    /// Lookahead for a keyword without consuming it.
    fn peek_keyword_eq(&self, kw: &str) -> bool {
        let mut p = Parser {
            src: self.src,
            pos: self.pos,
        };
        p.skip_ws();
        p.read_token()
            .map(|t| t.eq_ignore_ascii_case(kw))
            .unwrap_or(false)
    }

    fn read_float(&mut self) -> Result<f32> {
        self.skip_ws();
        let tok = self.read_token().ok_or_else(|| {
            Error::InvalidData("ASCII STL: expected float, got end-of-file".into())
        })?;
        tok.parse::<f32>()
            .map_err(|e| Error::InvalidData(format!("ASCII STL: `{tok}` is not a valid f32: {e}")))
    }

    /// Read the rest of the current line into a trimmed string,
    /// returning `None` if the line was empty after trimming. Used
    /// for the optional `<name>` token after `solid` / `endsolid`.
    fn read_optional_line_remainder(&mut self) -> Option<String> {
        let bytes = self.src.as_bytes();
        // Skip horizontal whitespace only (spaces / tabs); preserve
        // any newline so the caller's outer loop can detect facet vs
        // endsolid on the next iteration.
        while self.pos < bytes.len() {
            let b = bytes[self.pos];
            if b == b' ' || b == b'\t' {
                self.pos += 1;
            } else {
                break;
            }
        }
        let start = self.pos;
        while self.pos < bytes.len() {
            let b = bytes[self.pos];
            if b == b'\n' || b == b'\r' {
                break;
            }
            self.pos += 1;
        }
        let raw = &self.src[start..self.pos];
        let trimmed = raw.trim();
        if trimmed.is_empty() {
            None
        } else {
            Some(trimmed.to_string())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parser_reads_single_facet() {
        let s = "solid cube\n  facet normal 0 0 1\n    outer loop\n      vertex 0 0 0\n      vertex 1 0 0\n      vertex 0 1 0\n    endloop\n  endfacet\nendsolid cube\n";
        let scene = decode(s.as_bytes()).unwrap();
        assert_eq!(scene.meshes.len(), 1);
        let m = &scene.meshes[0];
        assert_eq!(m.name.as_deref(), Some("cube"));
        let p = &m.primitives[0];
        assert_eq!(p.positions.len(), 3);
        assert_eq!(p.normals.as_ref().unwrap().len(), 3);
    }

    #[test]
    fn encoder_emits_facet_block() {
        let mut s = Scene3D::new();
        let mut prim = Primitive::new(Topology::Triangles);
        prim.positions = vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        prim.normals = Some(vec![[0.0, 0.0, 1.0]; 3]);
        let mesh = Mesh::new(Some("t".to_string())).with_primitive(prim);
        s.add_mesh(mesh);
        let out = encode(&s).unwrap();
        let txt = std::str::from_utf8(&out).unwrap();
        assert!(txt.starts_with("solid t"));
        assert!(txt.contains("facet normal 0 0 1"));
        assert!(txt.contains("vertex 1 0 0"));
        assert!(txt.trim_end().ends_with("endsolid t"));
    }
}

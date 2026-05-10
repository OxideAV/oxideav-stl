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

use std::collections::HashMap;
use std::fmt::Write as _;

use oxideav_mesh3d::{Axis, Error, Mesh, Node, Primitive, Result, Scene3D, Topology, Unit};

/// Parse an ASCII STL byte slice into a [`Scene3D`].
pub fn decode(bytes: &[u8]) -> Result<Scene3D> {
    // ASCII STL is restricted to printable ASCII + standard whitespace
    // by the spec; we tolerate UTF-8 in the optional `<name>` field via
    // a lossy decode, since real-world files do ship non-ASCII names.
    let text = std::str::from_utf8(bytes)
        .map_err(|e| Error::InvalidData(format!("ASCII STL is not valid UTF-8: {e}")))?;

    let mut p = Parser::new(text);
    p.skip_ws();
    p.expect_keyword("solid")?;
    let name = p.read_optional_line_remainder();

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

        for _ in 0..3 {
            p.skip_ws();
            p.expect_keyword("vertex")?;
            let v = [p.read_float()?, p.read_float()?, p.read_float()?];
            positions.push(v);
            normals.push(n);
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

    let primitive = Primitive {
        topology: Topology::Triangles,
        positions,
        normals: Some(normals),
        tangents: None,
        uvs: Vec::new(),
        colors: Vec::new(),
        joints: None,
        weights: None,
        indices: None,
        material: None,
        extras: prim_extras,
    };

    let mesh = Mesh {
        name: name.filter(|s| !s.is_empty()),
        primitives: vec![primitive],
    };

    let mut scene = Scene3D::new();
    scene.up_axis = Axis::PosZ;
    scene.unit = Unit::Millimetres;
    let mesh_id = scene.add_mesh(mesh);
    let mut node = Node::new();
    node.mesh = Some(mesh_id);
    let node_id = scene.add_node(node);
    scene.add_root(node_id);
    Ok(scene)
}

/// Serialise a [`Scene3D`] as ASCII STL.
pub fn encode(scene: &Scene3D) -> Result<Vec<u8>> {
    let mut out = String::new();

    // Pick a name from the first mesh that has one — the spec only
    // allows one `solid` block, so even multi-mesh scenes flatten.
    let name = scene
        .meshes
        .iter()
        .find_map(|m| m.name.as_deref())
        .unwrap_or("");

    if name.is_empty() {
        out.push_str("solid\n");
    } else {
        let _ = writeln!(out, "solid {name}");
    }

    for mesh in &scene.meshes {
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
                let _ = writeln!(
                    out,
                    "  facet normal {} {} {}",
                    fmt_f32(n[0]),
                    fmt_f32(n[1]),
                    fmt_f32(n[2])
                );
                out.push_str("    outer loop\n");
                let _ = writeln!(
                    out,
                    "      vertex {} {} {}",
                    fmt_f32(v0[0]),
                    fmt_f32(v0[1]),
                    fmt_f32(v0[2])
                );
                let _ = writeln!(
                    out,
                    "      vertex {} {} {}",
                    fmt_f32(v1[0]),
                    fmt_f32(v1[1]),
                    fmt_f32(v1[2])
                );
                let _ = writeln!(
                    out,
                    "      vertex {} {} {}",
                    fmt_f32(v2[0]),
                    fmt_f32(v2[1]),
                    fmt_f32(v2[2])
                );
                out.push_str("    endloop\n");
                out.push_str("  endfacet\n");
            }
        }
    }

    if name.is_empty() {
        out.push_str("endsolid\n");
    } else {
        let _ = writeln!(out, "endsolid {name}");
    }

    Ok(out.into_bytes())
}

/// f32 formatter that emits a guaranteed-finite-looking string. The
/// spec allows `1.23456E+789`-style scientific notation; Rust's
/// default `Display` for `f32` is already round-trip-safe, so we use
/// it directly and translate non-finite values to `0` (STL has no
/// representation for NaN / Inf).
fn fmt_f32(v: f32) -> String {
    if v.is_finite() {
        format!("{v}")
    } else {
        "0".to_string()
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

    /// Skip ASCII whitespace including newlines + tabs.
    fn skip_ws(&mut self) {
        let bytes = self.src.as_bytes();
        while self.pos < bytes.len() {
            let b = bytes[self.pos];
            if b == b' ' || b == b'\t' || b == b'\r' || b == b'\n' {
                self.pos += 1;
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
        let mesh = Mesh {
            name: Some("t".into()),
            primitives: vec![Primitive {
                topology: Topology::Triangles,
                positions: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
                normals: Some(vec![[0.0, 0.0, 1.0]; 3]),
                tangents: None,
                uvs: Vec::new(),
                colors: Vec::new(),
                joints: None,
                weights: None,
                indices: None,
                material: None,
                extras: HashMap::new(),
            }],
        };
        s.add_mesh(mesh);
        let out = encode(&s).unwrap();
        let txt = std::str::from_utf8(&out).unwrap();
        assert!(txt.starts_with("solid t"));
        assert!(txt.contains("facet normal 0 0 1"));
        assert!(txt.contains("vertex 1 0 0"));
        assert!(txt.trim_end().ends_with("endsolid t"));
    }
}

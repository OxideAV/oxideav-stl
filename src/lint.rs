//! Strict-spec ASCII-STL conformance lint.
//!
//! [`lint_ascii`] walks an ASCII STL byte slice with exactly the same
//! grammar tolerance as [`crate::ascii::decode`] — every input the
//! decoder accepts, the lint accepts, and vice versa — but instead of
//! building a [`Scene3D`](oxideav_mesh3d::Scene3D) it returns a typed
//! [`AsciiLintReport`] recording every place the file leans on a
//! tolerance the strict 1989 spec letter does not grant. It is the
//! ASCII counterpart of [`crate::inspect_binary_header`]: pre-decode
//! triage for pipelines that must *emit* (or *demand*) letter-strict
//! files while still reading the tolerant real-world dialect.
//!
//! The rules come from the spec transcription staged at
//! `docs/3d/stl/fabbers-stl-format.html` (§6.5.2, *StL ASCII Format*):
//!
//! 1. **Keyword case** — "Bold face indicates a keyword; these must
//!    appear in lower case." The decoder accepts any case; the lint
//!    counts every keyword token that is not entirely lower-case
//!    under [`AsciiLintReport::keyword_case_defects`].
//! 2. **Tab indentation** — "Indentation must be with spaces; tabs
//!    are not allowed." The decoder accepts tabs anywhere whitespace
//!    is allowed; the lint counts every line whose *leading*
//!    whitespace contains a tab under
//!    [`AsciiLintReport::tab_indented_lines`].
//! 3. **Vertex-coordinate sign** — "A facet normal coordinate may
//!    have a leading minus sign; a vertex coordinate may not." The
//!    decoder parses any finite float in either slot; the lint counts
//!    every `vertex` coordinate token that lexically starts with `-`
//!    under [`AsciiLintReport::negative_vertex_coordinate_defects`].
//!    (This is the lexical face of the geometric all-positive-octant
//!    rule `validate` checks via `check_positive_octant`; pair with
//!    [`crate::repair_translate_to_positive_octant`] to fix the
//!    geometry before re-emit.) Normal coordinates are never flagged.
//! 4. **Comment lines** — the 1989 spec defines no comment syntax;
//!    the `;` / `#` whole-line comments the decoder skips (a vendor
//!    tolerance, see [`crate::ascii`]) are counted under
//!    [`AsciiLintReport::comment_lines`].
//! 5. **Single `solid` block** — the spec grammar describes one
//!    `solid … endsolid` block per file; the back-to-back multi-block
//!    concatenation the decoder accepts (an old Pro/E + AutoCAD
//!    tolerance) is counted under
//!    [`AsciiLintReport::extra_solid_blocks`].
//! 6. **Leading BOM** — the format is ASCII; a UTF-8 byte-order mark
//!    is a non-ASCII prefix some text editors prepend. The decoder
//!    strips it; the lint records it on
//!    [`AsciiLintReport::leading_bom`].
//!
//! A report with [`AsciiLintReport::is_strict_spec`] `== true`
//! certifies the input uses none of the six tolerances. The crate's
//! own ASCII encoder emits lower-case keywords, space indentation, no
//! comments, and no BOM, so its output lints clean except where the
//! *geometry* forces rule 3 (negative coordinates) or the *scene*
//! forces rule 5 (multiple meshes).

use oxideav_mesh3d::{Error, Result};

/// Cap on the per-rule illustrative example lists — same budget as
/// [`crate::validate::MAX_REPORTED_DEFECTS`]. Counts are always
/// complete; only the example lists are truncated.
pub const MAX_REPORTED_LINT_FINDINGS: usize = 32;

/// One illustrative lint finding: the 1-based source line plus the
/// offending token as written in the file.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AsciiLintFinding {
    /// 1-based line number of the offending token.
    pub line: usize,
    /// The offending token, verbatim.
    pub token: String,
}

/// Outcome of a [`lint_ascii`] call.
///
/// Every counter is complete (never capped); the `_examples` lists
/// are capped at [`MAX_REPORTED_LINT_FINDINGS`] entries each.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct AsciiLintReport {
    /// Number of `solid … endsolid` blocks walked. Always `>= 1` on
    /// a successful lint (zero blocks is a parse error, exactly as in
    /// [`crate::ascii::decode`]).
    pub solid_blocks: usize,
    /// Number of complete `facet … endfacet` records walked across
    /// every block.
    pub triangles_walked: usize,
    /// Keyword tokens not written entirely in lower case (rule 1).
    pub keyword_case_defects: usize,
    /// Up to [`MAX_REPORTED_LINT_FINDINGS`] illustrative rule-1
    /// findings.
    pub keyword_case_examples: Vec<AsciiLintFinding>,
    /// Lines whose leading whitespace contains a tab (rule 2).
    pub tab_indented_lines: usize,
    /// `vertex` coordinate tokens with a lexical leading `-` (rule 3).
    /// `facet normal` coordinates are never counted here.
    pub negative_vertex_coordinate_defects: usize,
    /// Up to [`MAX_REPORTED_LINT_FINDINGS`] illustrative rule-3
    /// findings.
    pub negative_vertex_coordinate_examples: Vec<AsciiLintFinding>,
    /// Whole-line `;` / `#` comments skipped at token boundaries
    /// (rule 4 — the spec defines no comment syntax).
    pub comment_lines: usize,
    /// `solid` blocks beyond the first (rule 5) —
    /// `solid_blocks - 1`.
    pub extra_solid_blocks: usize,
    /// Whether the input started with a UTF-8 byte-order mark
    /// (rule 6).
    pub leading_bom: bool,
}

impl AsciiLintReport {
    /// `true` iff the file uses none of the six tolerances — i.e. it
    /// conforms to the strict letter of the 1989 spec's ASCII grammar
    /// as far as this lint checks. Equivalent to
    /// `finding_total() == 0`.
    pub fn is_strict_spec(&self) -> bool {
        self.finding_total() == 0
    }

    /// One scalar headline — the sum of every per-rule counter (the
    /// BOM contributes `1` when present). Zero iff
    /// [`Self::is_strict_spec`].
    pub fn finding_total(&self) -> usize {
        self.keyword_case_defects
            + self.tab_indented_lines
            + self.negative_vertex_coordinate_defects
            + self.comment_lines
            + self.extra_solid_blocks
            + usize::from(self.leading_bom)
    }

    /// Labeled per-rule breakdown — six stable string keys safe to
    /// use as metric names. Sums exactly to [`Self::finding_total`].
    pub fn findings_by_rule(&self) -> [(&'static str, usize); 6] {
        [
            ("keyword_case", self.keyword_case_defects),
            ("tab_indentation", self.tab_indented_lines),
            (
                "negative_vertex_coordinate",
                self.negative_vertex_coordinate_defects,
            ),
            ("comment_line", self.comment_lines),
            ("extra_solid_block", self.extra_solid_blocks),
            ("leading_bom", usize::from(self.leading_bom)),
        ]
    }
}

/// Lint an ASCII STL byte slice against the strict 1989 spec letter.
///
/// Accepts exactly the inputs [`crate::ascii::decode`] accepts (same
/// keyword-case / tab / comment / multi-block / BOM tolerances, same
/// rejection of malformed grammar and unparseable floats) and returns
/// the same `Error::InvalidData` shape on the inputs it rejects — the
/// lint never succeeds where the decoder fails, and never fails where
/// the decoder succeeds.
///
/// This function does NOT attempt to distinguish ASCII vs binary —
/// route bytes here only after classification (binary bytes go to
/// [`crate::inspect_binary_header`]).
pub fn lint_ascii(bytes: &[u8]) -> Result<AsciiLintReport> {
    let mut report = AsciiLintReport::default();

    let stripped = bytes.strip_prefix(&[0xEF, 0xBB, 0xBF]);
    report.leading_bom = stripped.is_some();
    let bytes = stripped.unwrap_or(bytes);
    let text = std::str::from_utf8(bytes)
        .map_err(|e| Error::InvalidData(format!("ASCII STL is not valid UTF-8: {e}")))?;

    // Rule 2 — line-level pre-pass, independent of the grammar walk:
    // a line whose *leading* whitespace contains a tab violates
    // "Indentation must be with spaces; tabs are not allowed". Tabs
    // after the first non-whitespace byte are token separators, not
    // indentation, and are left to the grammar's whitespace model.
    for line in text.split('\n') {
        let indent_end = line
            .as_bytes()
            .iter()
            .position(|&b| b != b' ' && b != b'\t')
            .unwrap_or(line.len());
        if line.as_bytes()[..indent_end].contains(&b'\t') {
            report.tab_indented_lines += 1;
        }
    }

    let mut w = Walker {
        src: text,
        pos: 0,
        line: 1,
        report: &mut report,
    };

    // Grammar walk — mirrors `crate::ascii::decode` block for block.
    loop {
        w.skip_ws();
        if w.is_eof() {
            break;
        }
        if !w.peek_keyword_eq("solid") {
            let snippet: String = w.src[w.pos..]
                .chars()
                .take(16)
                .filter(|c| !c.is_control())
                .collect();
            return Err(Error::InvalidData(format!(
                "ASCII STL: expected `solid` or end-of-file, got `{snippet}`"
            )));
        }
        w.expect_keyword("solid")?;
        w.skip_line_remainder();

        loop {
            w.skip_ws();
            if w.peek_keyword_eq("endsolid") {
                w.expect_keyword("endsolid")?;
                w.skip_line_remainder();
                break;
            }
            w.expect_keyword("facet")?;
            w.expect_keyword("normal")?;
            for _ in 0..3 {
                // Rule 3 carve-out: "A facet normal coordinate may
                // have a leading minus sign" — never flagged.
                w.read_float(false)?;
            }
            w.expect_keyword("outer")?;
            w.expect_keyword("loop")?;
            for _ in 0..3 {
                w.expect_keyword("vertex")?;
                for _ in 0..3 {
                    // Rule 3: "a vertex coordinate may not".
                    w.read_float(true)?;
                }
            }
            w.expect_keyword("endloop")?;
            w.expect_keyword("endfacet")?;
            w.report.triangles_walked += 1;
        }
        w.report.solid_blocks += 1;
    }

    if report.solid_blocks == 0 {
        return Err(Error::InvalidData(
            "ASCII STL: no `solid` block found".into(),
        ));
    }
    report.extra_solid_blocks = report.solid_blocks - 1;
    Ok(report)
}

/// Grammar walker with line tracking + finding collection. Same
/// whitespace / comment / token model as the `crate::ascii` parser,
/// so acceptance is identical by construction.
struct Walker<'a> {
    src: &'a str,
    pos: usize,
    /// 1-based current line. Newlines are only ever consumed inside
    /// [`Self::skip_ws`], so this is the single place it advances.
    line: usize,
    report: &'a mut AsciiLintReport,
}

impl Walker<'_> {
    fn is_eof(&self) -> bool {
        self.pos >= self.src.len()
    }

    /// Skip whitespace + whole-line `;` / `#` comments, counting
    /// newlines into [`Self::line`] and comments into the report
    /// (rule 4). Comment recognition happens only at token-expect
    /// positions, exactly as in the decoder.
    fn skip_ws(&mut self) {
        let bytes = self.src.as_bytes();
        loop {
            while self.pos < bytes.len() {
                match bytes[self.pos] {
                    b'\n' => {
                        self.line += 1;
                        self.pos += 1;
                    }
                    b' ' | b'\t' | b'\r' => self.pos += 1,
                    _ => break,
                }
            }
            if self.pos < bytes.len() && (bytes[self.pos] == b';' || bytes[self.pos] == b'#') {
                self.report.comment_lines += 1;
                while self.pos < bytes.len() && bytes[self.pos] != b'\n' {
                    self.pos += 1;
                }
            } else {
                break;
            }
        }
    }

    /// Read the next whitespace-delimited token (no leading-ws skip).
    fn read_token(&mut self) -> Option<&str> {
        let bytes = self.src.as_bytes();
        let start = self.pos;
        while self.pos < bytes.len() {
            match bytes[self.pos] {
                b' ' | b'\t' | b'\r' | b'\n' => break,
                _ => self.pos += 1,
            }
        }
        if self.pos == start {
            None
        } else {
            Some(&self.src[start..self.pos])
        }
    }

    /// Consume `kw` (case-insensitively, matching the decoder) and
    /// record a rule-1 finding when the token isn't written entirely
    /// in lower case.
    fn expect_keyword(&mut self, kw: &str) -> Result<()> {
        self.skip_ws();
        let line = self.line;
        let tok = match self.read_token() {
            Some(t) => t,
            None => {
                return Err(Error::InvalidData(format!(
                    "ASCII STL: expected `{kw}`, got end-of-file"
                )))
            }
        };
        if !tok.eq_ignore_ascii_case(kw) {
            return Err(Error::InvalidData(format!(
                "ASCII STL: expected `{kw}`, got `{tok}`"
            )));
        }
        if tok != kw {
            let token = tok.to_string();
            self.report.keyword_case_defects += 1;
            if self.report.keyword_case_examples.len() < MAX_REPORTED_LINT_FINDINGS {
                self.report
                    .keyword_case_examples
                    .push(AsciiLintFinding { line, token });
            }
        }
        Ok(())
    }

    /// Lookahead for a keyword without consuming it (case-insensitive,
    /// no finding recorded — the consuming `expect_keyword` records).
    fn peek_keyword_eq(&mut self, kw: &str) -> bool {
        let save = (self.pos, self.line, self.report.comment_lines);
        self.skip_ws();
        let hit = self
            .read_token()
            .map(|t| t.eq_ignore_ascii_case(kw))
            .unwrap_or(false);
        (self.pos, self.line, self.report.comment_lines) = save;
        hit
    }

    /// Parse one float token; when `is_vertex`, record a rule-3
    /// finding on a lexical leading `-`.
    fn read_float(&mut self, is_vertex: bool) -> Result<()> {
        self.skip_ws();
        let line = self.line;
        let tok = match self.read_token() {
            Some(t) => t,
            None => {
                return Err(Error::InvalidData(
                    "ASCII STL: expected float, got end-of-file".into(),
                ))
            }
        };
        tok.parse::<f32>().map_err(|e| {
            Error::InvalidData(format!("ASCII STL: `{tok}` is not a valid f32: {e}"))
        })?;
        if is_vertex && tok.starts_with('-') {
            let token = tok.to_string();
            self.report.negative_vertex_coordinate_defects += 1;
            if self.report.negative_vertex_coordinate_examples.len() < MAX_REPORTED_LINT_FINDINGS {
                self.report
                    .negative_vertex_coordinate_examples
                    .push(AsciiLintFinding { line, token });
            }
        }
        Ok(())
    }

    /// Skip the rest of the current line (the optional `<name>` after
    /// `solid` / `endsolid`), preserving the newline for `skip_ws`.
    fn skip_line_remainder(&mut self) {
        let bytes = self.src.as_bytes();
        while self.pos < bytes.len() && bytes[self.pos] != b'\n' && bytes[self.pos] != b'\r' {
            self.pos += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const STRICT: &str = "solid part\n  facet normal 0 0 1\n    outer loop\n      vertex 0.5 0.5 0\n      vertex 1.5 0.5 0\n      vertex 0.5 1.5 0\n    endloop\n  endfacet\nendsolid part\n";

    #[test]
    fn strict_file_lints_clean() {
        let rep = lint_ascii(STRICT.as_bytes()).unwrap();
        assert!(rep.is_strict_spec());
        assert_eq!(rep.finding_total(), 0);
        assert_eq!(rep.solid_blocks, 1);
        assert_eq!(rep.triangles_walked, 1);
    }

    #[test]
    fn uppercase_keywords_are_counted_with_lines() {
        // Note: the replacement hits both `solid part` (line 1) and
        // the tail of `endsolid part` (line 9), so three keyword
        // tokens carry upper-case letters.
        let s = STRICT
            .replace("solid part", "SOLID part")
            .replace("endfacet", "EndFacet");
        let rep = lint_ascii(s.as_bytes()).unwrap();
        assert_eq!(rep.keyword_case_defects, 3);
        assert_eq!(rep.keyword_case_examples[0].line, 1);
        assert_eq!(rep.keyword_case_examples[0].token, "SOLID");
        assert_eq!(rep.keyword_case_examples[1].line, 8);
        assert_eq!(rep.keyword_case_examples[1].token, "EndFacet");
        assert_eq!(rep.keyword_case_examples[2].line, 9);
        assert_eq!(rep.keyword_case_examples[2].token, "endSOLID");
        assert!(!rep.is_strict_spec());
    }

    #[test]
    fn negative_normal_coordinate_is_not_flagged() {
        let s = STRICT.replace("normal 0 0 1", "normal 0 0 -1");
        let rep = lint_ascii(s.as_bytes()).unwrap();
        assert!(rep.is_strict_spec(), "{rep:?}");
    }

    #[test]
    fn negative_vertex_coordinate_is_flagged() {
        let s = STRICT.replace("vertex 1.5 0.5 0", "vertex -1.5 0.5 0");
        let rep = lint_ascii(s.as_bytes()).unwrap();
        assert_eq!(rep.negative_vertex_coordinate_defects, 1);
        assert_eq!(rep.negative_vertex_coordinate_examples[0].line, 5);
        assert_eq!(rep.negative_vertex_coordinate_examples[0].token, "-1.5");
    }

    #[test]
    fn rejects_what_decode_rejects() {
        assert!(lint_ascii(b"").is_err());
        assert!(lint_ascii(b"solid x\n facet garbage\nendsolid x\n").is_err());
        assert!(lint_ascii(b"not an stl at all").is_err());
    }
}

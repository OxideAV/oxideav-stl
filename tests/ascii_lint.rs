//! Integration tests for `oxideav_stl::lint_ascii` — the strict-spec
//! ASCII conformance lint (`docs/3d/stl/fabbers-stl-format.html`
//! §6.5.2 prose rules).

use oxideav_mesh3d::{Mesh, Mesh3DEncoder, Primitive, Scene3D, Topology};
use oxideav_stl::{
    lint_ascii, repair_translate_to_positive_octant, StlEncoder, DEFAULT_POSITIVE_OCTANT_MARGIN,
    MAX_REPORTED_LINT_FINDINGS,
};

const STRICT: &str = "solid part\n  facet normal 0 0 1\n    outer loop\n      vertex 0.5 0.5 1\n      vertex 1.5 0.5 1\n      vertex 0.5 1.5 1\n    endloop\n  endfacet\nendsolid part\n";

#[test]
fn strict_file_is_strict() {
    let rep = lint_ascii(STRICT.as_bytes()).unwrap();
    assert!(rep.is_strict_spec());
    assert_eq!(rep.finding_total(), 0);
    assert_eq!(rep.solid_blocks, 1);
    assert_eq!(rep.extra_solid_blocks, 0);
    assert_eq!(rep.triangles_walked, 1);
    assert!(!rep.leading_bom);
}

#[test]
fn keyword_case_rule_counts_every_non_lowercase_keyword() {
    // "Bold face indicates a keyword; these must appear in lower
    // case." — uppercase / mixed-case keywords decode fine but lint
    // as rule-1 findings.
    let s = STRICT
        .replace("facet normal", "FACET Normal")
        .replace("endsolid part", "ENDSOLID part");
    let rep = lint_ascii(s.as_bytes()).unwrap();
    assert_eq!(rep.keyword_case_defects, 3);
    assert_eq!(rep.keyword_case_examples.len(), 3);
    assert_eq!(rep.keyword_case_examples[0].token, "FACET");
    assert_eq!(rep.keyword_case_examples[0].line, 2);
    assert_eq!(rep.keyword_case_examples[1].token, "Normal");
    assert_eq!(rep.keyword_case_examples[2].token, "ENDSOLID");
    assert_eq!(rep.keyword_case_examples[2].line, 9);
    assert!(!rep.is_strict_spec());
    // The decoder accepts the same bytes — lint is diagnostic only.
    oxideav_stl::ascii::decode(s.as_bytes()).unwrap();
}

#[test]
fn tab_indentation_rule_counts_leading_tab_lines() {
    // "Indentation must be with spaces; tabs are not allowed."
    let s = STRICT
        .replace("  facet normal 0 0 1", "\tfacet normal 0 0 1")
        .replace("    outer loop", " \t outer loop");
    let rep = lint_ascii(s.as_bytes()).unwrap();
    assert_eq!(rep.tab_indented_lines, 2);
    assert!(!rep.is_strict_spec());
    // A tab used as a mid-line token separator is NOT indentation.
    let sep = STRICT.replace("facet normal 0 0 1", "facet\tnormal\t0 0 1");
    let rep = lint_ascii(sep.as_bytes()).unwrap();
    assert_eq!(rep.tab_indented_lines, 0);
    assert!(rep.is_strict_spec());
}

#[test]
fn vertex_sign_rule_flags_vertex_but_not_normal() {
    // "A facet normal coordinate may have a leading minus sign; a
    // vertex coordinate may not."
    let s = STRICT
        .replace("normal 0 0 1", "normal -0 0.707 -0.707")
        .replace("vertex 0.5 1.5 1", "vertex -0.5 1.5 -1e-3");
    let rep = lint_ascii(s.as_bytes()).unwrap();
    assert_eq!(rep.negative_vertex_coordinate_defects, 2);
    let ex = &rep.negative_vertex_coordinate_examples;
    assert_eq!(ex.len(), 2);
    assert_eq!(ex[0].token, "-0.5");
    assert_eq!(ex[0].line, 6);
    assert_eq!(ex[1].token, "-1e-3");
    assert!(!rep.is_strict_spec());
}

#[test]
fn comment_lines_and_bom_are_findings() {
    let mut bytes = vec![0xEF, 0xBB, 0xBF];
    bytes.extend_from_slice(b"; hand-edited preamble\n");
    bytes.extend_from_slice(STRICT.as_bytes());
    bytes.extend_from_slice(b"# trailing note\n");
    let rep = lint_ascii(&bytes).unwrap();
    assert!(rep.leading_bom);
    assert_eq!(rep.comment_lines, 2);
    assert_eq!(rep.finding_total(), 3);
    assert!(!rep.is_strict_spec());
}

#[test]
fn multi_solid_counts_extra_blocks() {
    let two = format!("{STRICT}{STRICT}");
    let rep = lint_ascii(two.as_bytes()).unwrap();
    assert_eq!(rep.solid_blocks, 2);
    assert_eq!(rep.extra_solid_blocks, 1);
    assert_eq!(rep.triangles_walked, 2);
    assert_eq!(rep.finding_total(), 1);
}

#[test]
fn findings_by_rule_sums_to_total_with_stable_labels() {
    let s = format!(
        "; comment\nSOLID a\n\tfacet normal 0 0 1\n outer loop\n vertex -1 0 1\n vertex 1 0 1\n vertex 0 1 1\n endloop\n endfacet\nendsolid a\n{STRICT}"
    );
    let rep = lint_ascii(s.as_bytes()).unwrap();
    let by_rule = rep.findings_by_rule();
    let labels: Vec<&str> = by_rule.iter().map(|(l, _)| *l).collect();
    assert_eq!(
        labels,
        [
            "keyword_case",
            "tab_indentation",
            "negative_vertex_coordinate",
            "comment_line",
            "extra_solid_block",
            "leading_bom",
        ]
    );
    let sum: usize = by_rule.iter().map(|(_, c)| c).sum();
    assert_eq!(sum, rep.finding_total());
    assert_eq!(rep.keyword_case_defects, 1);
    assert_eq!(rep.tab_indented_lines, 1);
    assert_eq!(rep.negative_vertex_coordinate_defects, 1);
    assert_eq!(rep.comment_lines, 1);
    assert_eq!(rep.extra_solid_blocks, 1);
}

#[test]
fn example_lists_cap_but_counts_stay_complete() {
    // 40 facets, each with one negative vertex coordinate — counts
    // are complete, examples cap at MAX_REPORTED_LINT_FINDINGS.
    let mut s = String::from("solid caps\n");
    for i in 0..40 {
        s.push_str(&format!(
            "facet normal 0 0 1\nouter loop\nvertex -{i} 0 1\nvertex 1 0 1\nvertex 0 1 1\nendloop\nendfacet\n"
        ));
    }
    s.push_str("endsolid caps\n");
    let rep = lint_ascii(s.as_bytes()).unwrap();
    assert_eq!(rep.negative_vertex_coordinate_defects, 40);
    assert_eq!(
        rep.negative_vertex_coordinate_examples.len(),
        MAX_REPORTED_LINT_FINDINGS
    );
    assert_eq!(rep.triangles_walked, 40);
}

/// Acceptance parity: on a corpus of accept/reject inputs, `lint_ascii`
/// succeeds exactly where `ascii::decode` succeeds.
#[test]
fn lint_acceptance_matches_decode_acceptance() {
    let corpus: Vec<Vec<u8>> = vec![
        STRICT.as_bytes().to_vec(),
        STRICT.to_uppercase().into_bytes(),
        STRICT.replace(' ', "\t").into_bytes(),
        format!("{STRICT}{STRICT}{STRICT}").into_bytes(),
        b"; only a comment\n".to_vec(),
        b"".to_vec(),
        b"solid x\nendsolid x\n".to_vec(),
        b"solid x\nfacet normal 0 0 1\nendsolid x\n".to_vec(),
        b"solid x\nfacet normal 0 0 one\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendfacet\nendsolid x\n".to_vec(),
        STRICT.replace("endloop", "").into_bytes(),
        STRICT.replace("outer loop", "outerloop").into_bytes(),
        b"garbage before\nsolid x\nendsolid x\n".to_vec(),
        vec![0xEF, 0xBB, 0xBF],
        vec![0xFF, 0xFE, 0x00],
    ];
    for (i, input) in corpus.iter().enumerate() {
        let decoded = oxideav_stl::ascii::decode(input);
        let linted = lint_ascii(input);
        assert_eq!(
            decoded.is_ok(),
            linted.is_ok(),
            "case {i}: decode {:?} vs lint {:?}",
            decoded.as_ref().map(|_| ()),
            linted.as_ref().map(|_| ())
        );
    }
}

/// The crate's own ASCII encoder lints clean on positive-octant
/// geometry; negative geometry trips only the vertex-sign rule until
/// `repair_translate_to_positive_octant` shifts it.
#[test]
fn encoder_output_lints_clean_after_octant_repair() {
    let mut scene = Scene3D::new();
    let mut prim = Primitive::new(Topology::Triangles);
    prim.positions = vec![[-1.0, -2.0, 0.5], [1.0, -2.0, 0.5], [-1.0, 2.0, 0.5]];
    prim.normals = Some(vec![[0.0, 0.0, 1.0]; 3]);
    scene.add_mesh(Mesh::new(Some("part".into())).with_primitive(prim));

    let mut enc = StlEncoder::new_ascii();
    let out = enc.encode(&scene).unwrap();
    let rep = lint_ascii(&out).unwrap();
    // Only the geometry-driven rule fires; the emitted grammar itself
    // is letter-strict (lowercase keywords, space indentation, no
    // comments, no BOM, single block).
    assert_eq!(rep.keyword_case_defects, 0);
    assert_eq!(rep.tab_indented_lines, 0);
    assert_eq!(rep.comment_lines, 0);
    assert_eq!(rep.extra_solid_blocks, 0);
    assert!(!rep.leading_bom);
    assert!(rep.negative_vertex_coordinate_defects > 0);
    assert!(!rep.is_strict_spec());

    // Shift into the all-positive octant and re-emit: fully strict.
    let _ = repair_translate_to_positive_octant(&mut scene, DEFAULT_POSITIVE_OCTANT_MARGIN);
    let out = enc.encode(&scene).unwrap();
    let rep = lint_ascii(&out).unwrap();
    assert!(rep.is_strict_spec(), "{rep:?}");
    assert_eq!(rep.triangles_walked, 1);
}

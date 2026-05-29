//! Long-running deterministic driver for `oxideav_stl::validate`'s
//! default-on rule set (facet orientation + unit normal +
//! watertight/manifold + consistent winding). The T-junction
//! sub-check is intentionally left off the default opt-in path so
//! this driver matches the production-default profile; turn it on by
//! flipping `ValidationOptions::check_t_junctions` if a separate
//! T-junction flamegraph is needed.

use oxideav_stl::{validate, ValidationOptions};

#[path = "profile_common/mod.rs"]
mod profile_common;

const N_TRIS: usize = 10_000;
const ITERATIONS: usize = 200;

fn main() {
    let scene = profile_common::synth_scene_unindexed(N_TRIS);
    let opts = ValidationOptions::default();
    let mut acc: usize = 0;
    for _ in 0..ITERATIONS {
        let report = validate(&scene, &opts);
        acc = acc.wrapping_add(report.triangles_total);
    }
    println!(
        "profile_validate: iterations={ITERATIONS} triangles_per_iter={N_TRIS} triangles_total={acc}"
    );
}

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Per-probe wall-clock + density harness for the nester — ground truth for the perf-tuning work
//! (the A-stack: collector reuse, hazkey cache, incremental loss). Reports best-of-N milliseconds
//! per probe so a speedup is visible against the noise floor. `cargo run -p ironnest-optimizer
//! --release --example bench`.
//!
//! DETERMINISM(ironnest): this is a BENCHMARK, never a placement path — the `Instant` wall-clock here
//! only times the engine; it never feeds a placement decision (which is why the determinism gate's
//! `Instant::now` ban does not apply, exactly as the std-trig ban is waived for the test-input
//! generators in `tests/nest.rs`). The placements themselves remain a pure function of `(inputs, seed,
//! budget)`. Timing is reported only to stderr-style stdout text; it is NOT part of any golden.

use ironnest_optimizer::{Scalar, nest, nest_multistart};

fn rect(w: Scalar, h: Scalar) -> Vec<[Scalar; 2]> {
    vec![[0.0, 0.0], [w, 0.0], [w, h], [0.0, h]]
}

/// Best-of-`reps` wall-clock (ms) of `f`. Best (min) is the most stable estimator for a CPU-bound,
/// allocation-sensitive routine: it filters scheduler/allocator noise that only ever adds time.
#[allow(clippy::disallowed_methods)] // wall-clock for benchmarking only — never a placement input
fn best_ms<F: FnMut()>(reps: u32, mut f: F) -> Scalar {
    let mut best = Scalar::INFINITY;
    for _ in 0..reps {
        let t = std::time::Instant::now();
        f();
        let ms = t.elapsed().as_secs_f64() * 1000.0;
        best = best.min(ms);
    }
    best
}

#[allow(clippy::too_many_arguments)]
fn probe(
    label: &str,
    item: Vec<[Scalar; 2]>,
    item_area: Scalar,
    qty: usize,
    side: Scalar,
    budget: u64,
    reps: u32,
    separation_heavy: bool,
) {
    let container = rect(side, side);
    let rotations = [0.0, 90.0, 180.0, 270.0];

    // Capture density once (it is deterministic), then time the call best-of-reps.
    let mut placed = 0usize;
    let ms = best_ms(reps, || {
        let sol = nest(
            std::slice::from_ref(&item),
            &[qty],
            &container,
            &[],
            0.0,
            &rotations,
            1,
            budget,
        );
        placed = sol.placements.len();
    });
    let util = (placed as Scalar * item_area) / (side * side) * 100.0;
    let tag = if separation_heavy { " [sep]" } else { "" };
    println!(
        "{label:<24} placed {placed:>4}/{qty:<4}  util {util:>5.1}%  best {ms:>8.2} ms  (budget {budget}, best of {reps}){tag}"
    );
}

fn main() {
    println!("--- bench (per-probe best-of-N wall-clock; [sep] = separation-search-heavy) ---");
    probe(
        "10x10 in 100 (exact)",
        rect(10.0, 10.0),
        100.0,
        100,
        100.0,
        2000,
        3,
        false,
    );
    probe(
        "10x10 in 100.5 (slack)",
        rect(10.0, 10.0),
        100.0,
        100,
        100.5,
        2000,
        3,
        false,
    );
    probe(
        "7x7 squares",
        rect(7.0, 7.0),
        49.0,
        220,
        100.0,
        2000,
        3,
        false,
    );
    probe(
        "13x7 bricks",
        rect(13.0, 7.0),
        91.0,
        120,
        100.0,
        2000,
        3,
        true,
    );
    let pentagon = vec![
        [0.0, 0.0],
        [20.0, 0.0],
        [20.0, 12.0],
        [12.0, 20.0],
        [0.0, 20.0],
    ];
    probe("pentagon ~344", pentagon, 344.0, 40, 100.0, 4000, 3, true);
    let right_tri = vec![[0.0, 0.0], [10.0, 0.0], [0.0, 10.0]];
    probe(
        "2 right-tris pair->sq",
        right_tri,
        50.0,
        2,
        11.0,
        4000,
        5,
        true,
    );

    println!("\n--- multi-start best-of-K (B1): density vs time on the seed-variant case ---");
    multistart_probe(
        "13x7 bricks",
        rect(13.0, 7.0),
        91.0,
        120,
        100.0,
        2000,
        &[1, 4, 8],
    );
}

/// Runs `nest_multistart` at several K and reports the best-of-K density + total wall-clock, so the
/// density/time trade of the multi-start lever is visible. K=1 is the single-start reference.
fn multistart_probe(
    label: &str,
    item: Vec<[Scalar; 2]>,
    item_area: Scalar,
    qty: usize,
    side: Scalar,
    budget: u64,
    ks: &[usize],
) {
    let container = rect(side, side);
    let rotations = [0.0, 90.0, 180.0, 270.0];
    for &k in ks {
        let mut placed = 0usize;
        let ms = best_ms(1, || {
            let sol = nest_multistart(
                std::slice::from_ref(&item),
                &[qty],
                &container,
                &[],
                0.0,
                &rotations,
                1,
                budget,
                k,
            );
            placed = sol.placements.len();
        });
        let util = (placed as Scalar * item_area) / (side * side) * 100.0;
        println!(
            "{label:<18} K={k:<3} placed {placed:>4}/{qty:<4}  util {util:>5.1}%  {ms:>9.1} ms"
        );
    }
}

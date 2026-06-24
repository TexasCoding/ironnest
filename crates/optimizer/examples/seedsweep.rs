// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Empirical test of the multi-start ("best-of-K") density thesis, through the PUBLIC API only
//! (sweep the `seed` argument; keep the best). No engine changes. Answers: does best-of-K lift
//! density, and for which shapes? `cargo run -p ironnest-optimizer --release --example seedsweep`

use std::collections::BTreeMap;

use ironnest_optimizer::{Scalar, nest};

fn rect(w: Scalar, h: Scalar) -> Vec<[Scalar; 2]> {
    vec![[0.0, 0.0], [w, 0.0], [w, h], [0.0, h]]
}

/// Sweeps `seeds` seeds for one probe and prints the placed-count distribution + best/worst.
fn sweep(
    label: &str,
    item: Vec<[Scalar; 2]>,
    item_area: Scalar,
    qty: usize,
    side: Scalar,
    budget: u64,
    seeds: u64,
) {
    let container = rect(side, side);
    let rotations = [0.0, 90.0, 180.0, 270.0];

    let mut dist: BTreeMap<usize, usize> = BTreeMap::new();
    let mut best = 0usize;
    let mut worst = usize::MAX;
    for seed in 1..=seeds {
        let sol = nest(
            std::slice::from_ref(&item),
            &[qty],
            &container,
            &[],
            0.0,
            &rotations,
            seed,
            budget,
        );
        let placed = sol.placements.len();
        *dist.entry(placed).or_default() += 1;
        best = best.max(placed);
        worst = worst.min(placed);
    }

    let util = |n: usize| (n as Scalar * item_area) / (side * side) * 100.0;
    let dist_str: Vec<String> = dist.iter().map(|(k, v)| format!("{k}:{v}")).collect();
    println!(
        "{label:<22} K={seeds:<3} best {best}/{qty} ({:.1}%)  worst {worst}/{qty} ({:.1}%)  spread +{:.1}pp  dist {{{}}}",
        util(best),
        util(worst),
        util(best) - util(worst),
        dist_str.join(", "),
    );
}

fn main() {
    println!("--- best-of-K seed sweep (public API, single ordering) ---");
    // Bricks: the roadmap CLAIMS best-of-K lifts 91.9% -> ~95.5%. Verify.
    sweep("13x7 bricks", rect(13.0, 7.0), 91.0, 120, 100.0, 2000, 16);
    // Squares: roadmap claims ZERO seed variance (rotation-invariant).
    sweep("7x7 squares", rect(7.0, 7.0), 49.0, 220, 100.0, 2000, 16);
    // Pentagon: roadmap claims FLAT at 23/40 across all seeds (architectural cap).
    let pentagon = vec![
        [0.0, 0.0],
        [20.0, 0.0],
        [20.0, 12.0],
        [12.0, 20.0],
        [0.0, 20.0],
    ];
    sweep("pentagon ~344", pentagon, 344.0, 40, 100.0, 4000, 16);
}

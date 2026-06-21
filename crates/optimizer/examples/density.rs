// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Quick density probe for the constructive nester (baseline before the separation local search).
//! `cargo run -p ironnest-optimizer --release --example density`

use ironnest_optimizer::{Scalar, nest};

fn rect(w: Scalar, h: Scalar) -> Vec<[Scalar; 2]> {
    vec![[0.0, 0.0], [w, 0.0], [w, h], [0.0, h]]
}

fn probe(
    label: &str,
    item: Vec<[Scalar; 2]>,
    item_area: Scalar,
    qty: usize,
    side: Scalar,
    budget: u64,
) {
    let container = rect(side, side);
    let sol = nest(
        &[item],
        &[qty],
        &container,
        0.0,
        &[0.0, 90.0, 180.0, 270.0],
        1,
        budget,
    );
    let placed = sol.placements.len();
    let util = (placed as Scalar * item_area) / (side * side) * 100.0;
    println!("{label:<24} placed {placed:>4}/{qty:<4}  util {util:>5.1}%  (budget {budget})");
}

fn main() {
    println!("--- density (LBF-drop + slide compaction; cardinal rotations) ---");
    // Exact-fit pathology: a perfect tiling needs ZERO gap, but collision detection requires
    // sub-micron separation, so only 9 (not 10) fit per row.
    probe(
        "10x10 in 100 (exact)",
        rect(10.0, 10.0),
        100.0,
        100,
        100.0,
        2000,
    );
    // Same parts, a hair of container slack → the artifact disappears.
    probe(
        "10x10 in 100.5 (slack)",
        rect(10.0, 10.0),
        100.0,
        100,
        100.5,
        2000,
    );

    println!("--- realistic (non-tiling) parts ---");
    probe("7x7 squares", rect(7.0, 7.0), 49.0, 220, 100.0, 2000);
    probe("13x7 bricks", rect(13.0, 7.0), 91.0, 120, 100.0, 2000);
    let pentagon = vec![
        [0.0, 0.0],
        [20.0, 0.0],
        [20.0, 12.0],
        [12.0, 20.0],
        [0.0, 20.0],
    ];
    probe("pentagon ~344", pentagon, 344.0, 40, 100.0, 4000);

    println!("--- interlocking (separation search must discover) ---");
    // Two right triangles pair into a 10x10 square. Their bounding boxes are both 10x10, so two
    // cannot fit in an 11x11 container side-by-side — the ONLY way both fit is the interlocked
    // pairing (one rotated 180°). Greedy construction places just one; the separation search has to
    // rearrange to discover the pair. ~82.6% util iff both placed; ~41% if only one.
    let right_tri = vec![[0.0, 0.0], [10.0, 0.0], [0.0, 10.0]];
    probe("2 right-tris pair->sq", right_tri, 50.0, 2, 11.0, 4000);
}

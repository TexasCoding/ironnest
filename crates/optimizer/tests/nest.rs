// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! End-to-end tests for the deterministic constructive nester: feasibility, capacity, separation,
//! and the in-process determinism golden (same inputs → byte-identical placements).

use ironnest_optimizer::{Scalar, nest};

/// An axis-aligned `w × h` rectangle with its lower-left corner at the origin (CCW).
fn rect(w: Scalar, h: Scalar) -> Vec<[Scalar; 2]> {
    vec![[0.0, 0.0], [w, 0.0], [w, h], [0.0, h]]
}

const CARDINAL: [Scalar; 4] = [0.0, 90.0, 180.0, 270.0];

#[test]
fn places_all_when_there_is_room() {
    // 16 × (10×10) squares into a 100×100 container = 16% fill → all must place.
    let items = vec![rect(10.0, 10.0)];
    let sol = nest(&items, &[16], &rect(100.0, 100.0), 0.0, &CARDINAL, 1, 2000);
    assert!(
        sol.unplaced.is_empty(),
        "all 16 should fit: {} unplaced",
        sol.unplaced.len()
    );
    assert_eq!(sol.placements.len(), 16);
    // every placement uses an allowed rotation
    for p in &sol.placements {
        assert!(
            CARDINAL.iter().any(|&r| (p.rotation_deg - r).abs() < 1e-9),
            "rotation {} not in the allowed set",
            p.rotation_deg
        );
        assert_eq!(p.item, 0);
    }
}

#[test]
fn capacity_overflow_reports_unplaced() {
    // Two 6×6 squares cannot both fit in a 10×10 container → exactly one places, three are unplaced.
    let items = vec![rect(6.0, 6.0)];
    let sol = nest(&items, &[4], &rect(10.0, 10.0), 0.0, &CARDINAL, 1, 3000);
    assert_eq!(sol.placements.len(), 1, "only one 6×6 fits in 10×10");
    assert_eq!(sol.unplaced, vec![0, 0, 0], "the other three are unplaced");
}

#[test]
fn determinism_same_seed_is_byte_identical() {
    let items = vec![rect(10.0, 10.0), rect(20.0, 5.0)];
    let qty = [8, 4];
    let container = rect(100.0, 100.0);
    let a = nest(&items, &qty, &container, 1.0, &CARDINAL, 12345, 2000);
    let b = nest(&items, &qty, &container, 1.0, &CARDINAL, 12345, 2000);
    assert_eq!(
        a, b,
        "same inputs + seed must produce byte-identical placements"
    );
    assert!(!a.placements.is_empty());
}

#[test]
fn separation_never_hands_place_item_an_out_of_bounds_pose() {
    // Regression: the GLS coordinate descent moves a part freely in x/y, so without an in-bounds
    // guard it could return a pose poking outside the quadtree root bbox — which `place_item` then
    // registers, tripping the CDE constrict assertion (qt_hazard.rs) in a debug build. This exact
    // case panicked before the `SampleEval::Invalid`-for-unplaceable guard in the evaluator.
    // Reaching the assertions below (no panic) is the test; feasibility is a sanity check.
    let items = vec![rect(10.0, 10.0)];
    let sol = nest(&items, &[12], &rect(25.0, 25.0), 0.0, &CARDINAL, 0, 1500);
    assert!(
        sol.placements.len() >= 4,
        "at least the 2×2 grid of 10×10 squares must place in 25×25, got {}",
        sol.placements.len()
    );
}

#[test]
fn separation_search_is_deterministic_and_finds_interlock() {
    // Two right triangles pair into a 10×10 square; their 10×10 bboxes can't both fit in 11×11
    // side-by-side, so the only way both place is the interlocked pairing — which greedy
    // construction misses and the GLS separation search must discover. This exercises the whole
    // separation stack (seed sampler, coordinate descent, colliding-item shuffle, GLS tracker), so
    // byte-identity across two same-seed runs proves that stack is reproducible.
    let tri = vec![[0.0, 0.0], [10.0, 0.0], [0.0, 10.0]];
    let items = std::slice::from_ref(&tri);
    let container = rect(11.0, 11.0);
    let a = nest(items, &[2], &container, 0.0, &CARDINAL, 42, 3000);
    let b = nest(items, &[2], &container, 0.0, &CARDINAL, 42, 3000);
    assert_eq!(
        a, b,
        "the separation path must be byte-identical for the same seed"
    );
    assert_eq!(
        a.placements.len(),
        2,
        "separation should interlock both triangles into the square"
    );
}

#[test]
fn min_separation_path_still_places() {
    // Exercises the geo-buffer min-sep offset (the documented residual) on the placement path.
    // Four 10×10 parts with 5.0 separation (→ ~15×15 footprint) easily fit in 100×100.
    let items = vec![rect(10.0, 10.0)];
    let sol = nest(&items, &[4], &rect(100.0, 100.0), 5.0, &CARDINAL, 7, 3000);
    assert!(sol.unplaced.is_empty(), "4 separated parts should fit");
    assert_eq!(sol.placements.len(), 4);
}

#[test]
fn no_rotation_set_defaults_to_zero() {
    let items = vec![rect(10.0, 10.0)];
    let sol = nest(&items, &[4], &rect(100.0, 100.0), 0.0, &[], 1, 1000);
    assert_eq!(sol.placements.len(), 4);
    for p in &sol.placements {
        assert!(
            (p.rotation_deg - 0.0).abs() < 1e-9,
            "no-rotation ⇒ 0°, got {}",
            p.rotation_deg
        );
    }
}

#[test]
fn drop_on_place_packs_densely() {
    // True bottom-left-fill (drop each part to contact) should tile slack squares near-perfectly.
    // Small case (5×5 grid) so it stays fast in debug builds; the release `density` example covers
    // the larger sheets. 25 × (10×10) into 51×51 → all 25 should fit.
    let items = vec![rect(10.0, 10.0)];
    let container = rect(51.0, 51.0);
    let sol = nest(&items, &[25], &container, 0.0, &CARDINAL, 1, 800);
    assert!(
        sol.placements.len() >= 24,
        "drop-on-place should pack ≥24/25 slack squares, got {}",
        sol.placements.len()
    );
}

#[test]
fn empty_demand_places_nothing() {
    let items = vec![rect(10.0, 10.0)];
    let sol = nest(&items, &[0], &rect(100.0, 100.0), 0.0, &CARDINAL, 1, 1000);
    assert!(sol.placements.is_empty());
    assert!(sol.unplaced.is_empty());
}

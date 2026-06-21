// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! End-to-end tests for the deterministic constructive nester: feasibility, capacity, separation,
//! interior-void holes, multi-sheet, and the in-process determinism golden (same inputs →
//! byte-identical placements).

use ironnest_optimizer::{Scalar, Sheet, nest, nest_multi};

/// An axis-aligned `w × h` rectangle with its lower-left corner at the origin (CCW).
fn rect(w: Scalar, h: Scalar) -> Vec<[Scalar; 2]> {
    vec![[0.0, 0.0], [w, 0.0], [w, h], [0.0, h]]
}

/// An axis-aligned rectangle spanning `[x0,y0]`–`[x1,y1]` (CCW).
fn rect_at(x0: Scalar, y0: Scalar, x1: Scalar, y1: Scalar) -> Vec<[Scalar; 2]> {
    vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1]]
}

const CARDINAL: [Scalar; 4] = [0.0, 90.0, 180.0, 270.0];

#[test]
fn places_all_when_there_is_room() {
    // 16 × (10×10) squares into a 100×100 container = 16% fill → all must place.
    let items = vec![rect(10.0, 10.0)];
    let sol = nest(
        &items,
        &[16],
        &rect(100.0, 100.0),
        &[],
        0.0,
        &CARDINAL,
        1,
        2000,
    );
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
    let sol = nest(
        &items,
        &[4],
        &rect(10.0, 10.0),
        &[],
        0.0,
        &CARDINAL,
        1,
        3000,
    );
    assert_eq!(sol.placements.len(), 1, "only one 6×6 fits in 10×10");
    assert_eq!(sol.unplaced, vec![0, 0, 0], "the other three are unplaced");
}

#[test]
fn determinism_same_seed_is_byte_identical() {
    let items = vec![rect(10.0, 10.0), rect(20.0, 5.0)];
    let qty = [8, 4];
    let container = rect(100.0, 100.0);
    let a = nest(&items, &qty, &container, &[], 1.0, &CARDINAL, 12345, 2000);
    let b = nest(&items, &qty, &container, &[], 1.0, &CARDINAL, 12345, 2000);
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
    let sol = nest(
        &items,
        &[12],
        &rect(25.0, 25.0),
        &[],
        0.0,
        &CARDINAL,
        0,
        1500,
    );
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
    let a = nest(items, &[2], &container, &[], 0.0, &CARDINAL, 42, 3000);
    let b = nest(items, &[2], &container, &[], 0.0, &CARDINAL, 42, 3000);
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
    let sol = nest(
        &items,
        &[4],
        &rect(100.0, 100.0),
        &[],
        5.0,
        &CARDINAL,
        7,
        3000,
    );
    assert!(sol.unplaced.is_empty(), "4 separated parts should fit");
    assert_eq!(sol.placements.len(), 4);
}

#[test]
fn no_rotation_set_defaults_to_zero() {
    let items = vec![rect(10.0, 10.0)];
    let sol = nest(&items, &[4], &rect(100.0, 100.0), &[], 0.0, &[], 1, 1000);
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
    let sol = nest(&items, &[25], &container, &[], 0.0, &CARDINAL, 1, 800);
    assert!(
        sol.placements.len() >= 24,
        "drop-on-place should pack ≥24/25 slack squares, got {}",
        sol.placements.len()
    );
}

#[test]
fn empty_demand_places_nothing() {
    let items = vec![rect(10.0, 10.0)];
    let sol = nest(
        &items,
        &[0],
        &rect(100.0, 100.0),
        &[],
        0.0,
        &CARDINAL,
        1,
        1000,
    );
    assert!(sol.placements.is_empty());
    assert!(sol.unplaced.is_empty());
}

// ---------------------------------------------------------------------------
// Phase 6: interior-void holes
// ---------------------------------------------------------------------------

#[test]
fn parts_never_overlap_a_keep_out_hole() {
    // A 60×60 sheet with a central 20×20 keep-out hole at (20,20)–(40,40). No placed part may
    // overlap the hole: the CDE `Hole` hazard is the arbiter, so any returned pose's bbox must be
    // disjoint from the hole (a necessary condition for shape-disjointness with axis-aligned parts).
    // No rotation, so each 10×10 part's footprint is exactly (p.x, p.y)–(p.x+10, p.y+10) and the
    // bbox-overlap check below is a faithful disjointness test against the hole.
    let items = vec![rect(10.0, 10.0)];
    let hole = rect_at(20.0, 20.0, 40.0, 40.0);
    let sol = nest(&items, &[12], &rect(60.0, 60.0), &[hole], 0.0, &[], 5, 1000);
    assert!(
        !sol.placements.is_empty(),
        "parts should nest around the hole"
    );
    for p in &sol.placements {
        let (x0, y0) = (p.x, p.y);
        let (x1, y1) = (p.x + 10.0, p.y + 10.0);
        // Open-overlap with a tiny tolerance: touching the hole edge is allowed (min_sep = 0).
        let eps = 1e-6;
        let overlaps = x0 < 40.0 - eps && x1 > 20.0 + eps && y0 < 40.0 - eps && y1 > 20.0 + eps;
        assert!(
            !overlaps,
            "part at ({x0},{y0})-({x1},{y1}) overlaps the keep-out hole (20,20)-(40,40)"
        );
    }
}

#[test]
fn holes_path_is_deterministic() {
    let items = vec![rect(10.0, 10.0)];
    let holes = [rect_at(20.0, 20.0, 40.0, 40.0)];
    let a = nest(
        &items,
        &[12],
        &rect(60.0, 60.0),
        &holes,
        0.0,
        &CARDINAL,
        5,
        1000,
    );
    let b = nest(
        &items,
        &[12],
        &rect(60.0, 60.0),
        &holes,
        0.0,
        &CARDINAL,
        5,
        1000,
    );
    assert_eq!(
        a, b,
        "the holes path must be byte-identical for the same seed"
    );
}

#[test]
fn nest_inside_a_void_framed_by_keep_outs() {
    // A 60×60 sheet whose interior is walled off except a 12×12 void in the middle, modeled as four
    // keep-out rectangles. Parts may only land inside the void.
    let items = vec![rect(10.0, 10.0)];
    let walls = vec![
        rect_at(0.0, 0.0, 60.0, 24.0),   // bottom band
        rect_at(0.0, 36.0, 60.0, 60.0),  // top band
        rect_at(0.0, 24.0, 24.0, 36.0),  // left of void
        rect_at(36.0, 24.0, 60.0, 36.0), // right of void
    ];
    // No rotation: the 10×10 footprint is exactly (p.x,p.y)–(p.x+10,p.y+10).
    let sol = nest(&items, &[4], &rect(60.0, 60.0), &walls, 0.0, &[], 3, 1500);
    // Only a single 10×10 fits in the 12×12 void.
    assert_eq!(
        sol.placements.len(),
        1,
        "exactly one 10×10 fits the 12×12 void"
    );
    let p = &sol.placements[0];
    assert!(
        p.x >= 24.0 - 1e-6
            && p.x + 10.0 <= 36.0 + 1e-6
            && p.y >= 24.0 - 1e-6
            && p.y + 10.0 <= 36.0 + 1e-6,
        "the placed part must sit inside the (24,24)-(36,36) void, got ({},{})",
        p.x,
        p.y
    );
}

#[test]
fn malformed_hole_reports_all_unplaced_no_panic() {
    // A degenerate (zero-area / fewer-than-3-vertex) hole fails import; the whole container import
    // returns Err, so nest() reports everything unplaced rather than silently ignoring the keep-out.
    let items = vec![rect(10.0, 10.0)];
    let degenerate_hole = vec![[20.0, 20.0], [20.0, 20.0]]; // not a polygon
    let sol = nest(
        &items,
        &[4],
        &rect(60.0, 60.0),
        &[degenerate_hole],
        0.0,
        &CARDINAL,
        1,
        1000,
    );
    assert!(sol.placements.is_empty());
    assert_eq!(sol.unplaced, vec![0, 0, 0, 0]);
}

#[test]
fn oversized_hole_clips_to_a_keepout_without_panicking() {
    // Regression (Phase 6 review blocker): a keep-out hole whose bbox encloses the quadtree root used
    // to trip the CDE constrict assertion at IMPORT time in a debug build. Clipping holes to the root
    // fixes it — an enclosing hole becomes a full-sheet keep-out ⇒ nothing is placeable, no panic.
    let item = rect(10.0, 10.0);
    // (a) a hole enclosing the sheet, and (b) one only a hair larger — both must clip, not panic.
    for hole in [
        rect_at(-10.0, -10.0, 70.0, 70.0),
        rect_at(-0.001, -0.001, 60.001, 60.001),
    ] {
        let sol = nest(
            std::slice::from_ref(&item),
            &[4],
            &rect(60.0, 60.0),
            &[hole],
            0.0,
            &[],
            1,
            500,
        );
        assert!(
            sol.placements.is_empty(),
            "a sheet-covering keep-out leaves nothing placeable"
        );
        assert_eq!(sol.unplaced.len(), 4);
    }
    // A hole entirely OUTSIDE the sheet is clipped away (no constraint): all parts still place.
    let outside = rect_at(100.0, 100.0, 120.0, 120.0);
    let sol = nest(
        &[item],
        &[4],
        &rect(60.0, 60.0),
        &[outside],
        0.0,
        &[],
        1,
        500,
    );
    assert_eq!(
        sol.placements.len(),
        4,
        "an out-of-bounds hole must not block placement"
    );
}

// ---------------------------------------------------------------------------
// Phase 6: multi-sheet
// ---------------------------------------------------------------------------

#[test]
fn multi_sheet_spills_overflow_to_the_next_sheet() {
    // One 6×6 fits per 10×10 sheet; demand 3 across two sheets → 2 place, 1 unplaced.
    let items = vec![rect(6.0, 6.0)];
    let sheets = vec![
        Sheet {
            outline: rect(10.0, 10.0),
            holes: vec![],
        },
        Sheet {
            outline: rect(10.0, 10.0),
            holes: vec![],
        },
    ];
    let sol = nest_multi(&items, &[3], &sheets, 0.0, &CARDINAL, 1, 3000);
    let placed: usize = sol.per_sheet.iter().map(Vec::len).sum();
    assert_eq!(placed, 2, "one part fits on each of the two sheets");
    assert_eq!(sol.unplaced, vec![0], "the third part fits on no sheet");
}

#[test]
fn multi_sheet_is_deterministic() {
    let items = vec![rect(10.0, 10.0)];
    let sheets = vec![
        Sheet {
            outline: rect(35.0, 35.0),
            holes: vec![],
        },
        Sheet {
            outline: rect(35.0, 35.0),
            holes: vec![],
        },
    ];
    let a = nest_multi(&items, &[16], &sheets, 0.0, &CARDINAL, 9, 1200);
    let b = nest_multi(&items, &[16], &sheets, 0.0, &CARDINAL, 9, 1200);
    assert_eq!(a, b, "multi-sheet must be byte-identical for the same seed");
}

#[test]
fn multi_sheet_zero_sheets_reports_all_unplaced() {
    let items = vec![rect(10.0, 10.0)];
    let sol = nest_multi(&items, &[3], &[], 0.0, &CARDINAL, 1, 1000);
    assert!(sol.per_sheet.is_empty());
    assert_eq!(sol.unplaced, vec![0, 0, 0]);
}

#[test]
fn multi_sheet_demand_exceeding_capacity_no_underflow() {
    // Demand far exceeds total capacity; the per-item remaining count must never underflow.
    let items = vec![rect(10.0, 10.0)];
    let sheets = vec![Sheet {
        outline: rect(12.0, 12.0),
        holes: vec![],
    }];
    let sol = nest_multi(&items, &[10], &sheets, 0.0, &CARDINAL, 1, 1000);
    let placed: usize = sol.per_sheet.iter().map(Vec::len).sum();
    assert_eq!(placed, 1, "only one 10×10 fits the single 12×12 sheet");
    assert_eq!(sol.unplaced.len(), 9);
}

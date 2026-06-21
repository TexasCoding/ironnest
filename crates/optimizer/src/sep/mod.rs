// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! `sep` — the overlap-minimization **separation search** (Phase 2b), ported from sparrow's Guided
//! Local Search (MIT) to ironnest's deterministic, fixed-budget, single-worker loop.
//!
//! Greedy construction can place each part only where it locally fits; it cannot *discover*
//! arrangements where parts must interlock. This module fixes that: it lets parts overlap, then
//! iteratively shoves them apart under GLS weighting until they separate — the only mechanism that
//! rearranges already-placed parts to make room.
//!
//! Layering (leaf → root): [`proxy`] (the smooth overlap signal) → [`tracker`] (per-pair loss + GLS
//! weights) → [`evaluator`] (CDE-arbitrated candidate scoring) → [`search`] (sample + coordinate
//! descent) → [`separator`] (the strike loop) → this module (the **bin-packing insertion driver**).
//!
//! Driver (sparrow's strip-shrink replaced by fixed-container insertion, doc §4.5): after the
//! constructive `improve()` pass, for each still-unplaced part (largest-first) — snapshot the layout,
//! seed the part at its lowest-overlap pose, run the separator over the *whole* layout (it may move
//! neighbours), and keep the part iff the result is feasible (the CDE is the arbiter); otherwise
//! restore the snapshot. All randomness is the seeded [`Prng`]; budgets are fixed (never a clock).

mod evaluator;
mod proxy;
mod search;
mod separator;
mod tracker;

use crate::prng::Prng;
use ironnest_cde::entities::{Item, Layout};
use search::SampleConfig;
use separator::SepConfig;
use tracker::CollisionTracker;

/// The fixed per-insertion separation budget. Matches sparrow's separator defaults (single worker):
/// 50 container + 25 focused samples, 3 coordinate descents, 100 no-improvement iterations, 3
/// strikes. All integers — never a wall clock.
const SEP_CONFIG: SepConfig = SepConfig {
    sample: SampleConfig {
        n_container_samples: 80,
        n_focussed_samples: 40,
        n_coord_descents: 3,
    },
    iter_no_imprv_limit: 150,
    strike_limit: 4,
};

/// Poses sampled when seeding a new (overlapping) part at its lowest-overlap position. Fixed.
const SEED_SAMPLES: usize = 400;

/// Tries to insert every still-unplaced instance (largest-first) via overlap-then-separate, updating
/// `placed_per_type`. Runs after the constructive `improve()` pass; a no-op when nothing is unplaced
/// (so the dense rectangular cases skip it entirely).
pub fn run_separation(
    layout: &mut Layout,
    entities: &[Option<Item>],
    order: &[usize],
    qty: &[usize],
    prng: &mut Prng,
    placed_per_type: &mut [usize],
) {
    for &item_id in order {
        let Some(item) = entities[item_id].as_ref() else {
            continue;
        };
        while placed_per_type[item_id] < qty[item_id] {
            if try_insert(layout, entities, item, prng) {
                placed_per_type[item_id] += 1;
            } else {
                // If one more of this type cannot be made to fit, neither can further copies.
                break;
            }
        }
    }
}

/// Attempts to add one `item`: seed it (allowing overlap), separate the whole layout, and keep it iff
/// the layout becomes feasible. Returns whether the item was kept.
fn try_insert(
    layout: &mut Layout,
    entities: &[Option<Item>],
    item: &Item,
    prng: &mut Prng,
) -> bool {
    let snapshot = layout.save();

    let Some(seed_dt) = search::lowest_overlap_pose(layout, item, prng, SEED_SAMPLES) else {
        return false; // the item does not fit the container in any orientation at all
    };
    layout.place_item(item, seed_dt);

    let mut tracker = CollisionTracker::new(layout);
    let feasible = separator::separate(layout, entities, &mut tracker, prng, SEP_CONFIG);

    // `separate`'s boolean is advisory (it reflects the proxy/CDE tracker's `total_loss`); the exact
    // `Layout::is_feasible` is the sole arbiter we keep on. They can disagree only conservatively —
    // a symmetric `PairMatrix` cell may read 0 for a pair the exact CDE still rejects in an fp edge
    // case — which at worst costs a missed placement, never an infeasible accepted one.
    if feasible && layout.is_feasible() {
        true
    } else {
        layout.restore(&snapshot);
        false
    }
}

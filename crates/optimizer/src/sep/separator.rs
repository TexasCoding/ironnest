// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! The separator: drive a layout (with overlaps) toward feasibility by repeatedly shoving colliding
//! parts to lower-overlap poses, guided by the GLS weights. Ported from sparrow
//! (`src/optimizer/{separator,worker}.rs`, MIT) as a **single deterministic worker** (no rayon, no
//! wall clock).
//!
//! `separate` is **Algorithm 9**, `move_items` is **Algorithm 5/10**. The budget is a fixed
//! `strike_limit × iter_no_imprv_limit` (never a clock); each move's effort is a fixed
//! [`SampleConfig`]. Between strikes the layout rolls back to the incumbent but the GLS **weights
//! are kept** — that accumulated pressure is what lets a wedged part jump elsewhere on the next try.

use super::evaluator::SeparationEvaluator;
use super::search::{SampleConfig, search_placement};
use super::tracker::CollisionTracker;
use crate::prng::Prng;
use ironnest_cde::entities::{Item, Layout, PItemKey};
use ironnest_geo::Scalar;

/// Fixed-budget configuration for one [`separate`] run.
#[derive(Clone, Copy, Debug)]
pub struct SepConfig {
    pub sample: SampleConfig,
    /// Max consecutive iterations without a (substantial) improvement before a strike. sparrow
    /// `iter_no_imprv_limit`.
    pub iter_no_imprv_limit: u32,
    /// Max strikes (rollback-and-retry-with-evolved-weights) before giving up. sparrow `strike_limit`.
    pub strike_limit: u32,
}

/// A "substantial" improvement is a drop below this fraction of the previous best. sparrow uses 0.98.
const SUBSTANTIAL_IMPROVEMENT: Scalar = 0.98;

/// Separates `layout` (which may contain overlaps) toward feasibility, mutating it in place and
/// leaving it at the lowest-overlap arrangement found. Returns `true` iff that arrangement is
/// collision-free (total loss reached zero). `tracker` must already describe `layout`.
pub fn separate(
    layout: &mut Layout,
    entities: &[Option<Item>],
    tracker: &mut CollisionTracker,
    prng: &mut Prng,
    cfg: SepConfig,
) -> bool {
    let mut best_layout = layout.save();
    let mut best_tracker = tracker.save();
    let mut min_loss = tracker.total_loss();

    if min_loss == 0.0 {
        return true; // already feasible — nothing to separate
    }

    let mut n_strikes = 0;
    'outer: while n_strikes < cfg.strike_limit {
        let mut n_no_improve = 0;
        let initial_strike_loss = tracker.total_loss();

        while n_no_improve < cfg.iter_no_imprv_limit {
            move_items(layout, entities, tracker, prng, cfg.sample);
            let loss = tracker.total_loss();

            if loss == 0.0 {
                best_layout = layout.save();
                best_tracker = tracker.save();
                min_loss = 0.0;
                break 'outer;
            } else if loss < min_loss {
                if loss < min_loss * SUBSTANTIAL_IMPROVEMENT {
                    n_no_improve = 0;
                }
                best_layout = layout.save();
                best_tracker = tracker.save();
                min_loss = loss;
            } else {
                n_no_improve += 1;
            }

            tracker.update_weights();
        }

        if initial_strike_loss * SUBSTANTIAL_IMPROVEMENT <= min_loss {
            n_strikes += 1;
        } else {
            n_strikes = 0;
        }

        // Roll back to the incumbent, but keep the (evolved) GLS weights.
        layout.restore(&best_layout);
        tracker.restore_but_keep_weights(&best_tracker);
    }

    // Leave the layout at the best arrangement found.
    layout.restore(&best_layout);
    tracker.restore_but_keep_weights(&best_tracker);
    min_loss == 0.0
}

/// One sweep: every currently-colliding part, in a PRNG-shuffled order, gets a chance to move to a
/// lower-overlap pose. sparrow `move_items` (Algorithm 5).
fn move_items(
    layout: &mut Layout,
    entities: &[Option<Item>],
    tracker: &mut CollisionTracker,
    prng: &mut Prng,
    sample: SampleConfig,
) {
    let mut candidates: Vec<PItemKey> = layout
        .placed_items
        .keys()
        .filter(|&pk| tracker.loss(pk) > 0.0)
        .collect();
    prng.shuffle(&mut candidates);

    for pk in candidates {
        // A previous move this sweep may already have resolved this one.
        if tracker.loss(pk) > 0.0 {
            move_one(layout, entities, tracker, prng, sample, pk);
        }
    }
}

/// Removes the part at `pk`, searches the now-reduced layout for its lowest-overlap pose, and
/// re-places it there. The search always includes the part's current pose, so a move never makes the
/// part worse. Updates the tracker.
fn move_one(
    layout: &mut Layout,
    entities: &[Option<Item>],
    tracker: &mut CollisionTracker,
    prng: &mut Prng,
    sample: SampleConfig,
    pk: PItemKey,
) {
    let pi = layout.placed_items[pk].clone();
    let item = entities[pi.item_id]
        .as_ref()
        .expect("placed item type must exist");
    let ref_dt = pi.d_transf;
    let focus_bbox = pi.shape.bbox;
    let container_bbox = layout.container.outer_cd.bbox;

    // Remove first, then search against the others — no self-collision to filter (the tracker still
    // holds `pk`'s weight row, which the evaluator reads).
    layout.remove_item(pk);

    let new_dt = {
        let mut ev = SeparationEvaluator::new(layout, tracker, item, pk);
        search_placement(
            item,
            container_bbox,
            ref_dt,
            focus_bbox,
            &mut ev,
            sample,
            prng,
        )
        .map_or(ref_dt, |(dt, _)| dt)
    };

    let new_pk = layout.place_item(item, new_dt);
    tracker.register_item_move(layout, pk, new_pk);
}

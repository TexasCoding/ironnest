// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Deterministic improvement search (milestone 2b, increment 1): bottom-left slide compaction + fill.
//!
//! After the constructive fill, each round (1) **compacts** every placed item with a geometric
//! *slide toward the bottom-left* — remove it, binary-search how far it can drop in Y (then in X)
//! without colliding, and re-place it there — then (2) **fills** the freed top-right space with any
//! still-unplaced items. The slide closes the gaps the random constructive pass leaves between
//! parts (e.g. squares that didn't quite abut), which both raises density and frees contiguous space
//! for more parts.
//!
//! Determinism: the slide is pure geometry (binary search + collision checks — no PRNG); the fill
//! reuses the same seeded [`Prng`] stream. Fixed round/iteration counts; never a wall clock. Every
//! re-placement is verified feasible, so compaction can only improve or no-op — never overlap.
//!
//! (A future increment can add an overlap-allowing separation pass — sparrow's guided local search —
//! for the last few percent on tightly-interlocking irregular parts.)

use crate::prng::Prng;
use crate::search::{feasible_at, search};
use ironnest_cde::entities::{Item, Layout, PItemKey};
use ironnest_geo::primitives::SPolygon;
use ironnest_geo::{DTransformation, Scalar};

/// Bottom-left slide alternations (drop-Y then drop-X) per item, per round.
const SLIDE_PASSES: u32 = 4;
/// Binary-search depth per axis (~`span / 2^30` ≈ sub-micron on a metre-scale sheet).
const BSEARCH_ITERS: u32 = 30;
/// Stop sliding an item once a full pass moves it less than this (container units).
const CONVERGE_EPS: Scalar = 1e-6;

/// Runs `rounds` of compaction-then-fill over `layout`, updating `placed_per_type`.
#[allow(clippy::too_many_arguments)]
pub fn improve(
    layout: &mut Layout,
    entities: &[Option<Item>],
    order: &[usize],
    qty: &[usize],
    rotations_rad: &[Scalar],
    prng: &mut Prng,
    budget: u64,
    rounds: u32,
    placed_per_type: &mut [usize],
) {
    for _ in 0..rounds {
        compact(layout, entities);
        fill_unplaced(
            layout,
            entities,
            order,
            qty,
            rotations_rad,
            prng,
            budget,
            placed_per_type,
        );
    }
}

/// Slides every placed item toward the bottom-left until it contacts a neighbour or the boundary.
fn compact(layout: &mut Layout, entities: &[Option<Item>]) {
    // Snapshot keys: each item is processed once; the slide re-inserts under a fresh key.
    let keys: Vec<PItemKey> = layout.placed_items.keys().collect();
    for pk in keys {
        let pi = layout.placed_items[pk].clone();
        let item = entities[pi.item_id].as_ref().unwrap();
        layout.remove_item(pk);
        let settled = slide_bottom_left(layout, item, &pi.d_transf);
        layout.place_item(item, settled);
    }
}

/// Places `item` at `transf` after first sliding it to its bottom-left contact pose — "drop" it into
/// place (true Left-Bottom-Fill) so it sits flush against existing parts instead of at the loose
/// sampled position. `transf` must be feasible with `item` not yet in the layout.
pub(crate) fn place_dropped(layout: &mut Layout, item: &Item, transf: DTransformation) {
    let settled = slide_bottom_left(layout, item, &transf);
    layout.place_item(item, settled);
}

/// The pose `item` settles into when slid toward the bottom-left from `current` (which must be a
/// feasible pose with `item` *removed* from `layout`'s CDE). Rotation is preserved.
fn slide_bottom_left(layout: &Layout, item: &Item, current: &DTransformation) -> DTransformation {
    let cde = layout.cde();
    let mut buffer = (*item.shape_cd).clone();
    buffer.surrogate = None;

    let rot = current.rotation();
    let (mut x, mut y) = current.translation();
    let bounds = cde.bbox();

    for _ in 0..SLIDE_PASSES {
        let ny = drop_coord(cde, item, &mut buffer, rot, Axis::Y, x, y, bounds.y_min);
        let nx = drop_coord(cde, item, &mut buffer, rot, Axis::X, ny, x, bounds.x_min);
        let moved = (y - ny).abs() + (x - nx).abs();
        x = nx;
        y = ny;
        if moved < CONVERGE_EPS {
            break;
        }
    }
    DTransformation::new(rot, (x, y))
}

#[derive(Clone, Copy)]
enum Axis {
    X,
    Y,
}

/// Binary-searches the lowest feasible value of one coordinate. `from` is the current (feasible)
/// value of the sliding axis; `fixed` is the other coordinate; `min` is the lower bound. Returns a
/// value in `[min, from]` that is guaranteed feasible (the deepest contact the search resolves).
#[allow(clippy::too_many_arguments)]
fn drop_coord(
    cde: &ironnest_cde::collision_detection::CDEngine,
    item: &Item,
    buffer: &mut SPolygon,
    rot: Scalar,
    axis: Axis,
    fixed: Scalar,
    from: Scalar,
    min: Scalar,
) -> Scalar {
    let mut lo = min; // possibly infeasible
    let mut hi = from; // feasible by precondition
    for _ in 0..BSEARCH_ITERS {
        let mid = 0.5 * (lo + hi);
        let feasible = match axis {
            Axis::X => feasible_at(cde, item, buffer, rot, mid, fixed),
            Axis::Y => feasible_at(cde, item, buffer, rot, fixed, mid),
        };
        if feasible {
            hi = mid; // can sit this low — try lower
        } else {
            lo = mid; // too low — back off
        }
    }
    hi
}

/// Tries to place every still-unplaced instance (largest-first) in whatever space is now free.
#[allow(clippy::too_many_arguments)]
fn fill_unplaced(
    layout: &mut Layout,
    entities: &[Option<Item>],
    order: &[usize],
    qty: &[usize],
    rotations_rad: &[Scalar],
    prng: &mut Prng,
    budget: u64,
    placed_per_type: &mut [usize],
) {
    for &item_id in order {
        let Some(item) = entities[item_id].as_ref() else {
            continue;
        };
        while placed_per_type[item_id] < qty[item_id] {
            match search(layout.cde(), item, rotations_rad, prng, budget) {
                Some(transf) => {
                    place_dropped(layout, item, transf);
                    placed_per_type[item_id] += 1;
                }
                None => break,
            }
        }
    }
}

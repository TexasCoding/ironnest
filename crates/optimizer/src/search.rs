// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! The per-item placement search: find the lowest-loss feasible pose for one item in a layout's CDE.
//!
//! Adapted from jagua-rs `lbf`'s `search` (MIT), but **deterministic by construction**: a fixed
//! sample budget (never a wall clock), our portable [`Prng`], and a uniform-shrink local refine in
//! place of `lbf`'s normal-distribution sampler (whose ziggurat would pull in platform `exp`/`ln`).

use crate::loss::LbfLoss;
use crate::prng::Prng;
use ironnest_cde::collision_detection::CDEngine;
use ironnest_cde::collision_detection::hazards::filter::NoFilter;
use ironnest_cde::entities::Item;
use ironnest_geo::Scalar;
use ironnest_geo::geo_traits::{Transformable, TransformableFrom};
use ironnest_geo::primitives::{Point, Rect};
use ironnest_geo::{DTransformation, Transformation};

/// Fraction of the per-item sample budget spent on the local refine (the rest is uniform sampling).
/// Mirrors `lbf`'s `ls_frac = 0.2`. Integer split below avoids any float cast.
const LS_NUM: u64 = 2;
const LS_DEN: u64 = 10;

/// Searches `cde` for the lowest-[`LbfLoss`] feasible placement of `item`, using exactly `budget`
/// samples. `rotations_rad` is the allowed discrete rotation set (radians); empty ⇒ no rotation.
/// Returns the chosen pose, or `None` if no feasible placement was sampled.
#[must_use]
pub fn search(
    cde: &CDEngine,
    item: &Item,
    rotations_rad: &[Scalar],
    prng: &mut Prng,
    budget: u64,
) -> Option<DTransformation> {
    let surrogate = item.shape_cd.surrogate();
    // A reusable buffer shape we transform per sample; drop its surrogate (we don't need it here).
    let mut buffer = (*item.shape_cd).clone();
    buffer.surrogate = None;

    let ls_budget = budget * LS_NUM / LS_DEN;
    let uni_budget = budget - ls_budget;

    let mut best: Option<(DTransformation, LbfLoss)> = None;
    let mut sample_bbox = cde.bbox();

    // ---- Uniform phase: sample the (shrinking) container bbox, keep the best feasible pose. ----
    for _ in 0..uni_budget {
        let d_transf = sample_pose(prng, sample_bbox, rotations_rad);
        let transf = d_transf.compose();
        if cde.detect_surrogate_collision(surrogate, &transf, &NoFilter) {
            continue; // fail-fast on the surrogate before the (costlier) full check
        }
        buffer.transform_from(&item.shape_cd, &transf);
        let cost = LbfLoss::from_bbox(buffer.bbox);

        let worth_testing = best.as_ref().is_none_or(|(_, bc)| cost.cost() < bc.cost());
        if worth_testing && !cde.detect_poly_collision(&buffer, &NoFilter) {
            sample_bbox = cost.tighten_sample_bbox(sample_bbox);
            best = Some((d_transf, cost));
        }
    }

    // ---- Local refine: shrink a box around the best position, keep its rotation fixed. ----
    let (mut best_t, mut best_c) = best?;
    let container_bbox = cde.bbox();
    // Start the refine window at a quarter of the item's diameter; decay it linearly to zero.
    let init_half = item.shape_cd.diameter * 0.25;
    for i in 0..ls_budget {
        #[allow(clippy::cast_precision_loss)]
        let progress = i as Scalar / ls_budget.max(1) as Scalar;
        let half = init_half * (1.0 - progress);
        let (cx, cy) = best_t.translation();
        let window = clamp_rect(cx, cy, half, container_bbox);

        let d_transf = sample_position(prng, window, best_t.rotation());
        let transf = d_transf.compose();
        if cde.detect_surrogate_collision(surrogate, &transf, &NoFilter) {
            continue;
        }
        buffer.transform_from(&item.shape_cd, &transf);
        let cost = LbfLoss::from_bbox(buffer.bbox);
        if cost.cost() < best_c.cost() && !cde.detect_poly_collision(&buffer, &NoFilter) {
            best_t = d_transf;
            best_c = cost;
        }
    }

    Some(best_t)
}

/// True iff placing `item` at `(rot, x, y)` collides with nothing currently in `cde`.
/// `buffer` is a scratch copy of `item.shape_cd` (surrogate dropped); it is overwritten in place.
#[must_use]
pub fn feasible_at(
    cde: &CDEngine,
    item: &Item,
    buffer: &mut ironnest_geo::primitives::SPolygon,
    rot: Scalar,
    x: Scalar,
    y: Scalar,
) -> bool {
    let transf = DTransformation::new(rot, (x, y)).compose();
    buffer.transform_from(&item.shape_cd, &transf);
    !cde.detect_poly_collision(buffer, &NoFilter)
}

/// Samples a full pose (rotation drawn from the allowed set, then x, then y) in `bbox`.
/// The draw order is fixed — it is part of the determinism contract.
fn sample_pose(prng: &mut Prng, bbox: Rect, rotations_rad: &[Scalar]) -> DTransformation {
    let rot = if rotations_rad.is_empty() {
        0.0
    } else {
        rotations_rad[prng.below(rotations_rad.len())]
    };
    let x = prng.range(bbox.x_min, bbox.x_max);
    let y = prng.range(bbox.y_min, bbox.y_max);
    DTransformation::new(rot, (x, y))
}

/// Samples a position (x, then y) in `bbox` at a fixed rotation `rot`.
fn sample_position(prng: &mut Prng, bbox: Rect, rot: Scalar) -> DTransformation {
    let x = prng.range(bbox.x_min, bbox.x_max);
    let y = prng.range(bbox.y_min, bbox.y_max);
    DTransformation::new(rot, (x, y))
}

/// A `2*half`-square centered on `(cx, cy)`, clipped to `bounds`.
fn clamp_rect(cx: Scalar, cy: Scalar, half: Scalar, bounds: Rect) -> Rect {
    Rect {
        x_min: Scalar::max(cx - half, bounds.x_min),
        y_min: Scalar::max(cy - half, bounds.y_min),
        x_max: Scalar::min(cx + half, bounds.x_max),
        y_max: Scalar::min(cy + half, bounds.y_max),
    }
}

/// The rigid transform that maps an item's *original* (caller-supplied) outline to its placed pose,
/// i.e. `placed = Rot(θ)·original + (x, y)`. jagua centroid-centers items at import, so the placed
/// `DTransformation` translates the *centroid-centered* shape; we fold the centroid back out here so
/// the public placement is anchor-free for the consumer.
///
/// `centroid` is the original outline's centroid (recoverable as `-pre_transform.translation`).
#[must_use]
pub fn original_to_placed(
    d_transf: &DTransformation,
    centroid: (Scalar, Scalar),
) -> (Scalar, Scalar, Scalar) {
    let theta = d_transf.rotation();
    let (tx, ty) = d_transf.translation();
    // Rot(θ)·centroid, via the geometry layer's (libm-backed, deterministic) rotation.
    let rot = Transformation::from_rotation(theta);
    let mut c = Point(centroid.0, centroid.1);
    c.transform(&rot);
    (tx - c.0, ty - c.1, theta.to_degrees())
}

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Sample evaluation: score one candidate pose for the item being moved.
//!
//! Ported/adapted from sparrow (`src/eval/{sample_eval,sep_evaluator}.rs`, MIT). The **CDE is the
//! feasibility arbiter** — [`CDEngine::collect_poly_collisions`] reports exactly which entities a
//! candidate pose collides with — and the [`super::proxy`] only *ranks* those collisions, weighted
//! by the GLS [`CollisionTracker`] weights. A pose with no collisions is [`SampleEval::Clear`];
//! otherwise [`SampleEval::Collision`] with the total weighted overlap.
//!
//! ironnest adaptation: the item being moved is **removed from the layout before** the search, so
//! there is no self-collision to filter (sparrow keeps it in and excludes it). We forgo sparrow's
//! `upper_bound` early-bailout in favour of a **canonical summation order** (sorted [`HazKey`]) so
//! the weighted loss is byte-identical regardless of CDE traversal order — robustness over the
//! constant-factor speedup. `+ − × ÷ sqrt` only.

use super::proxy::{
    quantify_collision_poly_container, quantify_collision_poly_hole, quantify_collision_poly_poly,
};
use super::tracker::CollisionTracker;
use ironnest_cde::collision_detection::hazards::collector::BasicHazardCollector;
use ironnest_cde::collision_detection::hazards::{HazKey, HazardEntity};
use ironnest_cde::entities::{Item, Layout, PItemKey};
use ironnest_cde::geometry::geo_enums::GeoRelation;
use ironnest_cde::geometry::primitives::{Rect, SPolygon};
use ironnest_geo::geo_traits::TransformableFrom;
use ironnest_geo::{DTransformation, Scalar};
use std::cmp::Ordering;

/// The outcome of evaluating a candidate pose. Ordered worst-last: any [`Self::Clear`] beats any
/// [`Self::Collision`], which beats [`Self::Invalid`]; within a variant, lower `loss` is better.
#[derive(Clone, Copy, Debug)]
pub enum SampleEval {
    /// No collisions — a feasible pose (`loss` is always `0.0`).
    Clear { loss: Scalar },
    /// Collides — `loss` is the total weighted overlap proxy.
    Collision { loss: Scalar },
    /// Not a usable pose (e.g. produced by a degenerate sampler). Sorts as worst.
    Invalid,
}

impl SampleEval {
    /// The comparable loss key (`+∞` for [`Self::Invalid`]). Uses [`Scalar::total_cmp`] downstream
    /// so the order is a deterministic total order (no rounding, no `NaN` ambiguity).
    fn rank(self) -> (u8, Scalar) {
        match self {
            SampleEval::Clear { loss } => (0, loss),
            SampleEval::Collision { loss } => (1, loss),
            SampleEval::Invalid => (2, Scalar::INFINITY),
        }
    }
}

impl Ord for SampleEval {
    // DETERMINISM(ironnest): we use the exact `Scalar::total_cmp` (a total order over all f64 bit
    // patterns) where sparrow uses an `FPA`-rounded `partial_cmp`. This is *intentionally* different:
    // total_cmp is byte-deterministic and never panics on a hypothetical NaN, but it does change
    // tie-breaks vs the reference, so a future "why doesn't this match sparrow?" is expected here.
    fn cmp(&self, other: &Self) -> Ordering {
        let (a_tag, a_loss) = self.rank();
        let (b_tag, b_loss) = other.rank();
        a_tag.cmp(&b_tag).then_with(|| a_loss.total_cmp(&b_loss))
    }
}

impl PartialOrd for SampleEval {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for SampleEval {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for SampleEval {}

/// Evaluates candidate poses for one item against a layout the item has been **removed** from.
pub struct SeparationEvaluator<'a> {
    layout: &'a Layout,
    tracker: &'a CollisionTracker,
    item: &'a Item,
    /// The item's (pre-move) key — still valid in the tracker for weight lookups.
    current_pk: PItemKey,
    container_bbox: Rect,
    /// Scratch shape (keeps its surrogate, which the proxy needs), transformed per candidate.
    shape_buff: SPolygon,
}

impl<'a> SeparationEvaluator<'a> {
    #[must_use]
    pub fn new(
        layout: &'a Layout,
        tracker: &'a CollisionTracker,
        item: &'a Item,
        current_pk: PItemKey,
    ) -> Self {
        Self {
            layout,
            tracker,
            item,
            current_pk,
            container_bbox: layout.container.outer_cd.bbox,
            shape_buff: (*item.shape_cd).clone(),
        }
    }

    /// Scores the pose `dt`: transform the item there, ask the CDE which hazards it hits, and sum
    /// their weighted overlap proxy in canonical [`HazKey`] order.
    pub fn evaluate_sample(&mut self, dt: DTransformation) -> SampleEval {
        let t = dt.compose();
        self.shape_buff.transform_from(&self.item.shape_cd, &t);

        // Reject any pose the CDE quadtree cannot hold: a shape whose bbox is not fully surrounded by
        // the quadtree root bbox is *unplaceable* (`Layout::place_item` would register a hazard
        // outside all quadrants — a debug panic / invariant break). The coordinate descent moves
        // freely in x/y, so it can wander here; marking it `Invalid` (worst rank) guarantees the
        // search never *returns* such a pose. Poses inside the (inflated-square) quadtree but outside
        // the container are still placeable — they score as an `Exterior` collision below, which the
        // separator then pushes back in. Pure bbox relation → determinism-safe.
        if self.layout.cde().bbox().relation_to(self.shape_buff.bbox) != GeoRelation::Surrounding {
            return SampleEval::Invalid;
        }

        let mut collector = BasicHazardCollector::with_capacity(self.layout.placed_items.len() + 1);
        self.layout
            .cde()
            .collect_poly_collisions(&self.shape_buff, &mut collector);

        if collector.is_empty() {
            return SampleEval::Clear { loss: 0.0 };
        }

        // Canonical summation: gather (HazKey, weighted term), sort by key, then sum — so the total
        // is independent of the CDE's traversal order and byte-identical across platforms.
        let mut terms: Vec<(HazKey, Scalar)> = collector
            .iter()
            .map(|(hkey, haz)| {
                let term = match haz {
                    HazardEntity::PlacedItem { pk: other_pk, .. } => {
                        let other_shape = &self.layout.placed_items[*other_pk].shape;
                        let loss = quantify_collision_poly_poly(other_shape, &self.shape_buff);
                        loss * self.tracker.pair_weight(self.current_pk, *other_pk)
                    }
                    HazardEntity::Exterior => {
                        let loss = quantify_collision_poly_container(
                            &self.shape_buff,
                            self.container_bbox,
                        );
                        loss * self.tracker.container_weight(self.current_pk)
                    }
                    // A hole / keep-out zone the part must avoid (the interior-void path); shares the
                    // item's single static-hazard GLS weight with the exterior.
                    HazardEntity::Hole { .. } | HazardEntity::InferiorQualityZone { .. } => {
                        let hole_shape = &self.layout.cde().hazards_map[hkey].shape;
                        let loss = quantify_collision_poly_hole(&self.shape_buff, hole_shape);
                        loss * self.tracker.container_weight(self.current_pk)
                    }
                };
                (hkey, term)
            })
            .collect();
        terms.sort_unstable_by_key(|(hkey, _)| *hkey);

        let loss: Scalar = terms.iter().map(|(_, term)| *term).sum();
        SampleEval::Collision { loss }
    }
}

/// The **unweighted** overlap of `shape` against everything currently in `layout` (no item is
/// excluded — used to seed a not-yet-placed part at its lowest-overlap pose, where there is no GLS
/// weight row yet). Summed in canonical [`HazKey`] order. Returns `0.0` for a collision-free pose.
#[must_use]
pub fn unweighted_overlap(layout: &Layout, shape: &SPolygon) -> Scalar {
    let mut collector = BasicHazardCollector::with_capacity(layout.placed_items.len() + 1);
    layout.cde().collect_poly_collisions(shape, &mut collector);
    if collector.is_empty() {
        return 0.0;
    }
    let container_bbox = layout.container.outer_cd.bbox;
    let mut terms: Vec<(HazKey, Scalar)> = collector
        .iter()
        .map(|(hkey, haz)| {
            let term = match haz {
                HazardEntity::PlacedItem { pk: other_pk, .. } => {
                    quantify_collision_poly_poly(&layout.placed_items[*other_pk].shape, shape)
                }
                HazardEntity::Exterior => quantify_collision_poly_container(shape, container_bbox),
                HazardEntity::Hole { .. } | HazardEntity::InferiorQualityZone { .. } => {
                    quantify_collision_poly_hole(shape, &layout.cde().hazards_map[hkey].shape)
                }
            };
            (hkey, term)
        })
        .collect();
    terms.sort_unstable_by_key(|(hkey, _)| *hkey);
    terms.iter().map(|(_, term)| *term).sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sample_eval_total_order() {
        let clear = SampleEval::Clear { loss: 0.0 };
        let cheap = SampleEval::Collision { loss: 1.0 };
        let dear = SampleEval::Collision { loss: 2.0 };
        let invalid = SampleEval::Invalid;

        // Clear beats any collision beats Invalid; within collisions, lower loss wins.
        assert!(clear < cheap);
        assert!(cheap < dear);
        assert!(dear < invalid);
        assert!(clear < invalid);
        // Reflexive equality (so dedup / first-min ties are well-defined).
        assert!(cheap == SampleEval::Collision { loss: 1.0 });
        assert_eq!(invalid.cmp(&invalid), Ordering::Equal);
    }
}

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! The Left-Bottom-Fill placement loss.
//!
//! Adapted from jagua-rs `lbf`'s `LBFLoss` (MIT), ported to [`Scalar`]. The loss prefers placements
//! toward the bottom-left of the container; the horizontal axis is weighted heavier so the search
//! packs in vertical "columns" left-to-right (a near-lexicographic preference without the brittle
//! tie behaviour of a pure lexicographic compare on continuous values).

use ironnest_geo::Scalar;
use ironnest_geo::primitives::Rect;

/// Horizontal dominance weight (jagua's `X_MULTIPLIER`).
const X_MULTIPLIER: Scalar = 10.0;

/// The loss assigned to a candidate placement: a weighted sum of the placed shape's `x_max` and
/// `y_max`. Lower is better (more bottom-left).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct LbfLoss {
    x_max: Scalar,
    y_max: Scalar,
}

impl LbfLoss {
    #[must_use]
    pub fn from_bbox(bbox: Rect) -> Self {
        Self {
            x_max: bbox.x_max,
            y_max: bbox.y_max,
        }
    }

    /// The scalar cost (lower = more bottom-left). Determinism: `+ * ` only.
    #[must_use]
    pub fn cost(&self) -> Scalar {
        self.x_max * X_MULTIPLIER + self.y_max
    }

    /// Tightens a sampling [`Rect`] to drop the region that could never beat `self`'s loss: any
    /// placement whose `x_max` exceeds `cost / X_MULTIPLIER` is already worse, so the sampler need
    /// not look there. Monotonically shrinks the search toward the current best.
    #[must_use]
    pub fn tighten_sample_bbox(&self, sample_bbox: Rect) -> Rect {
        let x_max_bound = self.cost() / X_MULTIPLIER;
        let mut tightened = sample_bbox;
        tightened.x_max = Scalar::min(sample_bbox.x_max, x_max_bound);
        tightened
    }
}

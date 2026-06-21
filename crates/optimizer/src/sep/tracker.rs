// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! The collision tracker — a cache of pairwise (and container) overlap *loss* plus the **Guided
//! Local Search** weights that let the separator escape local minima.
//!
//! Ported from sparrow (`src/quantify/{pair_matrix,tracker}.rs`, MIT) to [`Scalar`] = f64. Each
//! placed item is assigned a dense index `0..n`; pairwise losses/weights live in a symmetric
//! triangular [`PairMatrix`] and container losses/weights in a parallel `Vec`. The *objective* is
//! the total **unweighted** overlap (zero ⇒ feasible); the **weighted** overlap (`Σ weight·loss`) is
//! what each move minimizes, and weights grow on persistently-overlapping pairs until a part is
//! forced to jump elsewhere — the GLS escape, no annealing.
//!
//! DETERMINISM(ironnest): dense indices come from [`Layout::placed_items`] key order (slotmap slot
//! order — deterministic for a deterministic op history). All loss/weight sums run over the dense
//! `Vec`/index ranges in a **fixed canonical order**, so totals are byte-identical. `+ − × ÷` only.

use super::proxy::{quantify_collision_poly_container, quantify_collision_poly_poly};
use ironnest_cde::collision_detection::hazards::HazardEntity;
use ironnest_cde::collision_detection::hazards::collector::BasicHazardCollector;
use ironnest_cde::entities::{Layout, PItemKey};
use ironnest_geo::Scalar;
use slotmap::SecondaryMap;
use std::ops::{Index, IndexMut};

/// Worst-collision weight growth (`m` at `loss == max_loss`). sparrow `GLS_WEIGHT_MAX_INC_RATIO`.
const GLS_WEIGHT_MAX_INC_RATIO: Scalar = 2.0;
/// Mildest-collision weight growth (`m` as `loss → 0⁺`). sparrow `GLS_WEIGHT_MIN_INC_RATIO`.
const GLS_WEIGHT_MIN_INC_RATIO: Scalar = 1.2;
/// Per-iteration decay toward 1.0 for separated pairs. sparrow `GLS_WEIGHT_DECAY`.
const GLS_WEIGHT_DECAY: Scalar = 0.95;

/// One cell of the tracker: the (unweighted) overlap `loss` and its GLS `weight` (≥ 1.0).
#[derive(Debug, Clone, Copy)]
pub struct CTEntry {
    pub loss: Scalar,
    pub weight: Scalar,
}

impl CTEntry {
    fn fresh() -> Self {
        Self {
            weight: 1.0,
            loss: 0.0,
        }
    }
}

/// A symmetric triangular matrix of pairwise [`CTEntry`]s, indexed by dense item index. Storage is
/// the upper triangle flattened row-major; `(row, col)` and `(col, row)` map to the same cell.
#[derive(Debug, Clone)]
pub struct PairMatrix {
    pub size: usize,
    pub data: Vec<CTEntry>,
}

impl PairMatrix {
    fn new(size: usize) -> Self {
        let len = size * (size + 1) / 2;
        Self {
            size,
            data: vec![CTEntry::fresh(); len],
        }
    }
}

/// Maps a (symmetric) `(row, col)` index to the flattened upper-triangle offset. sparrow `calc_idx`.
fn calc_idx(row: usize, col: usize, size: usize) -> usize {
    debug_assert!(row < size && col < size);
    if row <= col {
        (row * size) + col - ((row * (row + 1)) / 2)
    } else {
        (col * size) + row - ((col * (col + 1)) / 2)
    }
}

impl Index<(usize, usize)> for PairMatrix {
    type Output = CTEntry;
    fn index(&self, (row, col): (usize, usize)) -> &Self::Output {
        &self.data[calc_idx(row, col, self.size)]
    }
}

impl IndexMut<(usize, usize)> for PairMatrix {
    fn index_mut(&mut self, (row, col): (usize, usize)) -> &mut Self::Output {
        &mut self.data[calc_idx(row, col, self.size)]
    }
}

/// Tracks collisions between item pairs and between items and the container, plus the GLS weights.
#[derive(Debug, Clone)]
pub struct CollisionTracker {
    size: usize,
    pk_idx_map: SecondaryMap<PItemKey, usize>,
    pair_collisions: PairMatrix,
    container_collisions: Vec<CTEntry>,
}

impl CollisionTracker {
    /// Builds a tracker for `layout`, computing the current loss for every placed item. Weights all
    /// start at 1.0.
    #[must_use]
    pub fn new(layout: &Layout) -> Self {
        let size = layout.placed_items.len();
        let mut ct = Self {
            size,
            pk_idx_map: layout
                .placed_items
                .keys()
                .enumerate()
                .map(|(i, pk)| (pk, i))
                .collect(),
            pair_collisions: PairMatrix::new(size),
            container_collisions: vec![CTEntry::fresh(); size],
        };
        let keys: Vec<PItemKey> = layout.placed_items.keys().collect();
        for pk in keys {
            ct.recompute_loss_for_item(pk, layout);
        }
        ct
    }

    /// Recomputes (and overwrites) all loss values in `pk`'s row from the CDE's current state.
    /// Weights are untouched. `pk` must currently be placed in `layout`.
    fn recompute_loss_for_item(&mut self, pk: PItemKey, layout: &Layout) {
        let idx = self.pk_idx_map[pk];
        let shape = &layout.placed_items[pk].shape;

        // Reset this item's losses (pair row + container).
        for i in 0..self.size {
            self.pair_collisions[(idx, i)].loss = 0.0;
        }
        self.container_collisions[idx].loss = 0.0;

        // Which hazards does the item currently collide with? Pre-seed the collector with the item's
        // own hazard so the collection skips it (a shape "collides" with its own coincident hazard).
        let self_hkey = layout
            .cde()
            .haz_key_from_pi_key(pk)
            .expect("placed item must be registered in the CDE");
        let mut collector = BasicHazardCollector::with_capacity(layout.placed_items.len() + 1);
        collector.insert(
            self_hkey,
            HazardEntity::from((pk, &layout.placed_items[pk])),
        );
        layout.cde().collect_poly_collisions(shape, &mut collector);

        // This loop only *assigns* per-cell losses (never `+=`), so its iteration order over the
        // `SecondaryMap` collector is irrelevant — the determinism-sensitive summation happens later
        // in `loss()` / `total_loss()`, which fold over the dense storage in a fixed canonical order.
        let container_bbox = layout.container.outer_cd.bbox;
        for (hkey, haz) in &collector {
            if hkey == self_hkey {
                continue;
            }
            if let HazardEntity::PlacedItem { pk: other_pk, .. } = haz {
                let shape_other = &layout.placed_items[*other_pk].shape;
                let idx_other = self.pk_idx_map[*other_pk];
                let loss = quantify_collision_poly_poly(shape, shape_other);
                self.pair_collisions[(idx, idx_other)].loss = loss;
            } else {
                // Exterior (and, in a future holes milestone, Hole / InferiorQualityZone) — quantify
                // against the container bbox. Only `Exterior` occurs on the current `nest()` path.
                let loss = quantify_collision_poly_container(shape, container_bbox);
                self.container_collisions[idx].loss = loss;
            }
        }
    }

    /// After an item is removed and re-placed (acquiring a new key but the same dense index),
    /// re-point the index map and recompute its loss row.
    pub fn register_item_move(&mut self, layout: &Layout, old_pk: PItemKey, new_pk: PItemKey) {
        let idx = self
            .pk_idx_map
            .remove(old_pk)
            .expect("moved item must have an index");
        self.pk_idx_map.insert(new_pk, idx);
        self.recompute_loss_for_item(new_pk, layout);
    }

    /// **Algorithm 8** — multiplies every weight: colliding cells grow (worst grows fastest),
    /// separated cells decay toward 1.0; floored at 1.0.
    pub fn update_weights(&mut self) {
        let max_loss = self
            .pair_collisions
            .data
            .iter()
            .chain(self.container_collisions.iter())
            .map(|e| e.loss)
            .fold(0.0, Scalar::max);

        // No collisions anywhere ⇒ nothing to grow, and dividing by `max_loss` below would be 0/0.
        // (Unreachable from `separate`, which only calls this while loss > 0, but cheap to harden:
        // a single NaN weight would poison every subsequent ranking.)
        if max_loss == 0.0 {
            return;
        }

        for e in self
            .pair_collisions
            .data
            .iter_mut()
            .chain(self.container_collisions.iter_mut())
        {
            let multiplier = if e.loss == 0.0 {
                GLS_WEIGHT_DECAY
            } else {
                GLS_WEIGHT_MIN_INC_RATIO
                    + (GLS_WEIGHT_MAX_INC_RATIO - GLS_WEIGHT_MIN_INC_RATIO) * (e.loss / max_loss)
            };
            e.weight = (e.weight * multiplier).max(1.0);
        }
    }

    /// The GLS weight for the pair `(pk1, pk2)`.
    #[must_use]
    pub fn pair_weight(&self, pk1: PItemKey, pk2: PItemKey) -> Scalar {
        let (idx1, idx2) = (self.pk_idx_map[pk1], self.pk_idx_map[pk2]);
        self.pair_collisions[(idx1, idx2)].weight
    }

    /// The GLS weight for `pk`'s collision with the container.
    #[must_use]
    pub fn container_weight(&self, pk: PItemKey) -> Scalar {
        let idx = self.pk_idx_map[pk];
        self.container_collisions[idx].weight
    }

    /// `pk`'s total (unweighted) loss: its container loss + the sum of its pairwise losses. Summed in
    /// dense-index order (canonical).
    #[must_use]
    pub fn loss(&self, pk: PItemKey) -> Scalar {
        let idx = self.pk_idx_map[pk];
        let pair_loss: Scalar = (0..self.size)
            .map(|i| self.pair_collisions[(idx, i)].loss)
            .sum();
        self.container_collisions[idx].loss + pair_loss
    }

    /// The layout's total (unweighted) overlap loss — zero iff the layout is collision-free. Summed
    /// over the dense storage in a fixed canonical order.
    #[must_use]
    pub fn total_loss(&self) -> Scalar {
        let cont: Scalar = self.container_collisions.iter().map(|e| e.loss).sum();
        let pair: Scalar = self.pair_collisions.data.iter().map(|e| e.loss).sum();
        cont + pair
    }

    /// A snapshot clone of the tracker (loss + weights + index map).
    #[must_use]
    pub fn save(&self) -> Self {
        self.clone()
    }

    /// Restores the loss values and index map from `snapshot`, but **keeps** the current weights —
    /// the GLS escape relies on weights surviving a rollback.
    pub fn restore_but_keep_weights(&mut self, snapshot: &Self) {
        self.pk_idx_map = snapshot.pk_idx_map.clone();
        for (a, b) in self
            .pair_collisions
            .data
            .iter_mut()
            .zip(snapshot.pair_collisions.data.iter())
        {
            a.loss = b.loss;
        }
        for (a, b) in self
            .container_collisions
            .iter_mut()
            .zip(snapshot.container_collisions.iter())
        {
            a.loss = b.loss;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calc_idx_is_symmetric_and_covers_every_cell_once() {
        // The 4×4 upper-triangle layout from the sparrow comment: 10 cells, indices 0..10, and
        // (row, col) maps to the same cell as (col, row).
        let size = 4;
        let mut upper = Vec::new();
        for r in 0..size {
            for c in 0..size {
                assert_eq!(
                    calc_idx(r, c, size),
                    calc_idx(c, r, size),
                    "({r},{c}) must equal ({c},{r})"
                );
                if r <= c {
                    upper.push(calc_idx(r, c, size));
                }
            }
        }
        upper.sort_unstable();
        assert_eq!(
            upper,
            (0..10).collect::<Vec<_>>(),
            "upper triangle is dense 0..10"
        );
    }

    #[test]
    #[allow(clippy::float_cmp)] // the multipliers are exact small-integer arithmetic
    fn update_weights_matches_algorithm8() {
        // Two-item tracker (3 pair cells + 2 container cells); set one pair cell to the max loss.
        let mut ct = CollisionTracker {
            size: 2,
            pk_idx_map: SecondaryMap::new(),
            pair_collisions: PairMatrix::new(2),
            container_collisions: vec![CTEntry::fresh(); 2],
        };
        ct.pair_collisions.data[0].loss = 2.0; // the worst collision (== max_loss)
        ct.update_weights();

        // Worst collision: m = 1.2 + (2.0 − 1.2)·(2/2) = 2.0 → weight 1·2 = 2.0.
        assert_eq!(ct.pair_collisions.data[0].weight, 2.0);
        // Separated cells: m = 0.95 → (1·0.95).max(1.0) = 1.0 (floored, not allowed below 1).
        assert_eq!(ct.pair_collisions.data[1].weight, 1.0);
        assert_eq!(ct.container_collisions[0].weight, 1.0);
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn update_weights_no_collisions_is_a_noop() {
        // All losses zero ⇒ early return, no NaN from 0/0, weights untouched.
        let mut ct = CollisionTracker {
            size: 2,
            pk_idx_map: SecondaryMap::new(),
            pair_collisions: PairMatrix::new(2),
            container_collisions: vec![CTEntry::fresh(); 2],
        };
        ct.pair_collisions.data[1].weight = 1.7; // a previously-grown weight
        ct.update_weights();
        assert_eq!(ct.pair_collisions.data[1].weight, 1.7, "untouched, no NaN");
    }
}

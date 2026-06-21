// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use crate::Scalar;
use crate::entities::LayoutSnapshot;
use crate::probs::bpp::entities::{BPInstance, LayKey};
use slotmap::SecondaryMap;

/// Snapshot of [`BPProblem`](crate::probs::bpp::entities::BPProblem) at a specific moment.
/// Can be used to restore to a previous state.
#[derive(Debug, Clone)]
pub struct BPSolution {
    /// A map of the layout snapshots, identified by the same keys as in the problem
    pub layout_snapshots: SecondaryMap<LayKey, LayoutSnapshot>,
    /// Logical creation stamp. DETERMINISM(ironnest): jagua used `web_time::Instant::now()` here;
    /// wall-clock is never a placement/output input, so we fix it to `0` for byte-reproducible
    /// output (CLAUDE.md determinism rules).
    pub time_stamp: u64,
}

impl BPSolution {
    #[must_use]
    pub fn density(&self, instance: &BPInstance) -> Scalar {
        let total_bin_area = self
            .layout_snapshots
            .values()
            .map(|ls| ls.container.area())
            .sum::<Scalar>();

        let total_item_area = self
            .layout_snapshots
            .values()
            .map(|ls| ls.placed_item_area(instance))
            .sum::<Scalar>();

        total_item_area / total_bin_area
    }

    #[must_use]
    pub fn cost(&self, instance: &BPInstance) -> u64 {
        self.layout_snapshots
            .values()
            .map(|ls| ls.container.id)
            .map(|id| instance.bins[id].cost)
            .sum()
    }
}

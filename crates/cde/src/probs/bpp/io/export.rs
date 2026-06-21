// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use crate::io::export::export_layout_snapshot;
use crate::probs::bpp::entities::{BPInstance, BPSolution};
use crate::probs::bpp::io::ext_repr::ExtBPSolution;

/// Exports a solution out of the library.
///
/// DETERMINISM(ironnest): `epoch` and `time_stamp` are `u64` logical stamps (jagua used
/// `web_time::Instant`). With both fixed to `0`, `run_time_sec` is a deterministic `0` — wall-clock
/// never influences exported output (CLAUDE.md determinism rules).
#[must_use]
pub fn export(instance: &BPInstance, solution: &BPSolution, epoch: u64) -> ExtBPSolution {
    ExtBPSolution {
        cost: solution.cost(instance),
        layouts: solution
            .layout_snapshots
            .values()
            .map(|sl| export_layout_snapshot(sl, instance))
            .collect(),
        run_time_sec: solution.time_stamp.saturating_sub(epoch),
        density: solution.density(instance),
    }
}

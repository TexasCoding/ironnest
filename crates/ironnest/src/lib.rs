// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! ironnest — the public placement-oracle API.
//!
//! In: polygons + an irregular container (boundary + holes / keepout zones) + a min-separation +
//! an allowed-rotation set + a seed + an iteration budget. Out: `(item, x, y, rotation)`.
//!
//! The engine knows **nothing** about kerf, lead-ins, pierces, cut sequencing, G-code, or any
//! machine number — those live in the consumer, which re-validates every layout. Same inputs MUST
//! produce byte-identical placements on every shipped platform. See
//! `docs/00-ironnest-architecture-and-plan.md` §7.

use ironnest_optimizer::Scalar;

/// A single resolved placement — the only thing the oracle emits.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Placement {
    /// Index into the caller's input item list.
    pub item: usize,
    /// Placement origin X, in the container's coordinate space.
    pub x: Scalar,
    /// Placement origin Y, in the container's coordinate space.
    pub y: Scalar,
    /// Rotation in degrees; always a member of the caller's allowed set (e.g. {0, 90, 180, 270}).
    pub rotation_deg: Scalar,
}

// TODO(Phase 2): the real entry point. This stub keeps the workspace green; the full signature
//
//     pub fn nest(items, qty, container, min_sep, rotations, seed, budget) -> NestSolution
//
// lands once the geo/cde types (Polygon, Container) exist. Determinism contract: explicit seed,
// fixed iteration budget (no wall clock), discrete rotations only, no `rand::random()` fallback.

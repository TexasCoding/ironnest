// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! ironnest-geo — forked jagua-rs geometry primitives + shape modification, ported to f64.
//!
//! Phase 1 lands here: vendor `geometry/*` from jagua-rs @ `43e8137` (Point, Rect, Circle,
//! SPolygon, Edge, transforms, fail-fast surrogates, and the `shape_modification` min-sep
//! offsetter) and flip every `f32` to [`Scalar`]. See `docs/01-jagua-source-verification.md` §B.

/// The single scalar width for the entire engine.
///
/// `f64` for numerical robustness — a near-tangent "just fits vs overlaps" decision must not flip
/// on f32's ~7 significant digits, and a fixed grid-snap stays clean. Determinism comes from the
/// rules in `CLAUDE.md` (discrete rotations, no platform transcendentals, no FMA, …), **not** from
/// the width. Route any future width change through this alias only; never hard-code a float type
/// on a placement path.
pub type Scalar = f64;

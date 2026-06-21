// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! ironnest-cde — forked jagua-rs collision-detection engine + entities + io + bpp, at f64.
//!
//! Forked from jagua-rs @ `43e8137`: `collision_detection/*` (`CDEngine`, quadtree, hazard model,
//! surrogate fail-fast), `entities/*` (`Item`, `Container` with quality-zone holes, `Layout`,
//! `PlacedItem`, `Instance`), `io/{import,ext_repr,export}` (drives `min_item_separation`), and
//! `probs/bpp/*` (the
//! `place_item`/`remove_item`/`save`/`restore` problem driver). The geometry layer lives in the
//! [`ironnest_geo`] crate and is re-exported below as [`geometry`] so upstream `crate::geometry::*`
//! paths resolve unchanged.
//!
//! ## Determinism scrub (forked changes vs upstream)
//! - **`probs/bpp/io/import.rs`** — rayon `par_iter` → sequential `iter` (no parallel float
//!   reduction on any import path; keeps the dep graph thread-free).
//! - **`util/assertions.rs`** — debug-only `HashSet` set-comparisons rewritten to deterministic
//!   `Vec` containment (no per-process hash iteration).
//! - **wall-clock** — jagua's `time_stamp: Instant` (a `web_time::Instant::now()`) is replaced by a
//!   `u64` logical stamp fixed to `0`: wall-clock is never a placement/output input (CLAUDE.md).
#![warn(
    clippy::pedantic,
    clippy::correctness,
    clippy::suspicious,
    clippy::complexity,
    clippy::perf,
    clippy::style,
    clippy::disallowed_types,
    clippy::disallowed_methods
)]
#![allow(clippy::missing_panics_doc, clippy::missing_errors_doc)]

pub use ironnest_geo::Scalar;

/// The geometry layer ([`ironnest_geo`]), re-exported so upstream `crate::geometry::*` paths and
/// re-exports (`Point`, `SPolygon`, `Transformation`, `shape_modification`, `fail_fast`, …) resolve
/// without rewrites across the fork.
pub use ironnest_geo as geometry;

/// Everything related to the Collision Detection Engine
pub mod collision_detection;

/// Entities to model 2D Irregular Cutting and Packing Problems
pub mod entities;

/// Importing problem instances into and exporting solutions out of this library
pub mod io;

/// Helper functions which do not belong to any specific module
pub mod util;

/// Enabled variants of the 2D irregular Cutting and Packing Problem (bin packing only).
pub mod probs;

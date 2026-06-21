// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! ironnest-geo — forked jagua-rs geometry primitives + shape modification, ported to f64.
//!
//! Forked from jagua-rs @ `43e8137` (`geometry/*` + `util/fpa`), with every `f32` flipped to
//! [`Scalar`]. This crate is the geometry **leaf** of the workspace (`geo ← cde ← optimizer ←
//! ironnest`); it has no dependency on `entities`/`io`/`cde`. See
//! `docs/01-jagua-source-verification.md` §B.
//!
//! ## Determinism notes (forked changes vs upstream)
//! - **`transformation.rs`** — `f32::sin_cos`/`atan2` (platform libm, non-portable) are replaced by
//!   the pure-Rust [`libm`] crate (byte-identical across platforms). Rotation trig DOES run on the
//!   placement path (`DTransformation::compose()` per placed item); determinism holds *because* it
//!   routes through `libm`, never std. This MUST stay `libm` — see the note on `rot_m`.
//! - **`shape_modification.rs`** — the min-separation offsetter still routes through `geo-buffer`
//!   (pinned). It is the one remaining cross-platform residual (plan risk #2) and must be proven
//!   bit-stable in the x-platform golden, or replaced with our own deterministic offsetter.
//!
//! Lints: jagua's upstream quality posture is preserved, plus the workspace determinism gate
//! (`clippy::disallowed_{types,methods}`).
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

/// The single scalar width for the entire engine.
///
/// `f64` for numerical robustness — a near-tangent "just fits vs overlaps" decision must not flip
/// on f32's ~7 significant digits, and a fixed grid-snap stays clean. Determinism comes from the
/// rules in `CLAUDE.md` (discrete rotations, no platform transcendentals, no FMA, …), **not** from
/// the width. Route any future width change through this alias only; never hard-code a float type
/// on a placement path.
pub type Scalar = f64;

/// Set of functions to compute and generate [convex hulls](https://en.wikipedia.org/wiki/Convex_hull)
pub mod convex_hull;

mod d_transformation;

/// The *fail-fast surrogate* and all logic pertaining to its generation
pub mod fail_fast;

/// Set of enums representing various geometric properties
pub mod geo_enums;

/// Set of traits representing various geometric properties & operations
pub mod geo_traits;

/// Set of geometric primitives - atomic building blocks for the geometry module
pub mod primitives;

mod transformation;

mod original_shape;

/// Vendored straight-skeleton polygon offsetter (geo-buffer 0.2.0, Apache-2.0), forked for
/// determinism (libm trig) and to drop the `geo` dependency. Used by [`shape_modification`].
mod buffer;

/// Set of functions to modify geometric shapes
pub mod shape_modification;

/// Float comparison helper (approximate equality within a tolerance)
pub mod fpa;

#[doc(inline)]
pub use d_transformation::DTransformation;

#[doc(inline)]
pub use transformation::Transformation;

#[doc(inline)]
pub use original_shape::OriginalShape;

#[doc(inline)]
pub use d_transformation::normalize_rotation;

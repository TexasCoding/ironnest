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
//! produce byte-identical placements on every shipped platform (proven by the cross-platform CI
//! golden — see `docs/00-ironnest-architecture-and-plan.md` §6/§7).
//!
//! This crate is the **stable public surface** that `crates/py` (the `ironnest` wheel) and Rust
//! embedders depend on; the implementation lives in [`ironnest_optimizer`] (the deterministic
//! placement search, sitting on `ironnest-cde` + `ironnest-geo`, the f64 fork of jagua-rs). Phase 2
//! grew the real [`nest`] there; this crate graduates it to the public name.

#[doc(inline)]
pub use ironnest_optimizer::{NestSolution, Placement, Scalar, nest};

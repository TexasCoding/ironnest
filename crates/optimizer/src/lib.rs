// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! ironnest-optimizer — OUR deterministic placement search (new code; the brain).
//!
//! Construction (a deterministic constructive placement over the irregular container, querying the
//! CDE for "does this collide?") plus improvement (a separation / overlap-minimization local search
//! lifting sparrow's guided-local-search math, MIT — not linked) — driven by a **fixed
//! iteration/sample budget, never a wall clock**. Discrete rotations {0, 90, 180, 270} only (exact
//! coordinate swaps; zero trig). Single canonical worker; seeded portable PRNG. Built on
//! [`ironnest_cde`].

pub use ironnest_cde::Scalar;

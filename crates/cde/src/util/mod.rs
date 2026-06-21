// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// Set of functions used throughout to assure the correctness of the library.
pub mod assertions;

// NOTE(ironnest): jagua's `util::fpa` (the `FPA` approx-equality wrapper) lives in the geometry
// leaf crate `ironnest_geo` instead — reach it as `ironnest_geo::fpa::FPA` if ever needed here.

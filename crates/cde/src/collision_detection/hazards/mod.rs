// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

mod hazard;

/// Everything related to hazard filters
pub mod filter;

/// Everything related to hazard collectors
pub mod collector;

#[doc(inline)]
pub use hazard::HazKey;
#[doc(inline)]
pub use hazard::Hazard;
#[doc(inline)]
pub use hazard::HazardEntity;

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

// NOTE(ironnest): jagua gated `spp`/`bpp`/`mspp` behind Cargo features and only `bpp` is forked
// (docs/00 §4). We drop the feature gates and the `spp`/`mspp` modules entirely — `bpp` is always on.

/// Bin Packing Problem (BPP) module
pub mod bpp;

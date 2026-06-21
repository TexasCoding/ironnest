// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! ironnest-cde — forked jagua-rs collision-detection engine + entities + io + bpp, at f64.
//!
//! Phase 1: vendor `collision_detection/*` (CDEngine, quadtree, hazard model, surrogate
//! fail-fast), `entities/*` (Item, Container with quality-zone holes, Layout, PlacedItem,
//! Instance), `io/*` (import/ext_repr/export — drives `min_item_separation`), and `probs/bpp/*`
//! (the `place_item`/`remove_item`/`save`/`restore` problem driver). Built on [`ironnest_geo`].

pub use ironnest_geo::Scalar;

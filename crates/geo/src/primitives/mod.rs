// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![allow(clippy::inline_always)]

mod circle;
mod edge;
mod point;
mod rect;
mod simple_polygon;

#[doc(inline)]
pub use circle::Circle;
#[doc(inline)]
pub use edge::Edge;
#[doc(inline)]
pub use point::Point;
#[doc(inline)]
pub use rect::Rect;
#[doc(inline)]
pub use simple_polygon::SPolygon;

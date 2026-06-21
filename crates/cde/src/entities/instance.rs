// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use crate::entities::Container;
use crate::entities::Item;
use std::any::Any;

/// The (abstract) static representation of a problem instance.
/// This trait defines shared functionality between any instance variant.
pub trait Instance: Any {
    /// All items
    fn items(&self) -> impl Iterator<Item = &Item>;

    /// All containers
    fn containers(&self) -> impl Iterator<Item = &Container>;

    /// A specific item
    fn item(&self, id: usize) -> &Item;

    /// A specific container
    fn container(&self, id: usize) -> &Container;
}

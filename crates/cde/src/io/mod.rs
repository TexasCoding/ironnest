// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

/// External (serializable) representations of the entities within the library.
pub mod ext_repr;

/// All logic for converting external representations into internal ones
pub mod import;

/// All logic for exporting internal representations into external ones
pub mod export;

// NOTE(ironnest): jagua's `io/svg` is intentionally NOT forked — SVG rendering is out of scope for
// a headless placement oracle (see docs/00 §4 "Do NOT take").

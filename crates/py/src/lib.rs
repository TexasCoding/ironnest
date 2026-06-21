// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! PyO3 binding — builds the `ironnest` abi3-py313 wheel (`import ironnest`).
//!
//! Phase 4 wires `ironnest::nest(...)` to a single `#[pyfunction]`, marshalling polygons as plain
//! `Vec<Vec<[f64; 2]>>` (no numpy, no JSON wire — the JSON wire is exactly the drift class that
//! killed the old CLI stub). For now this is a stub that imports cleanly.

use pyo3::prelude::*;

/// The `ironnest` Python module. (Must match the `[lib] name` for the abi3 init symbol.)
#[pymodule]
fn ironnest(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}

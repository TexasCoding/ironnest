// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! PyO3 binding — the `ironnest` abi3-py313 wheel (`import ironnest`).
//!
//! A single [`nest`] `#[pyfunction]` wrapping [`ironnest_core::nest`]. Polygons marshal as plain
//! `list[list[tuple[float, float]]]` (PyO3 `Vec<Vec<[f64; 2]>>`) — **no numpy, no JSON wire** (the
//! JSON wire is exactly the float-drift class that killed the old CLI stub). Python `float` *is* IEEE
//! f64, so the marshalling is exact and introduces no nondeterminism: the wheel inherits the engine's
//! byte-identical, cross-platform-reproducible output (proven by the Phase-3 golden).

use ironnest_core::{Sheet, nest as nest_impl, nest_multi as nest_multi_impl};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

/// One placed instance returned to Python: `(item, x, y, rotation_deg)`.
type PyPlacement = (usize, f64, f64, f64);

/// One sheet for [`nest_multi`]: `(outline, holes)`.
type PySheet = (Vec<[f64; 2]>, Vec<Vec<[f64; 2]>>);

/// Nest `items` (one outline per part type, each `[[x, y], …]` in item-local coordinates) into a
/// single irregular `container` outline.
///
/// Parameters
/// ----------
/// items : list[list[tuple[float, float]]]
///     One polygon outline per item *type*, in item-local coordinates.
/// qty : list[int]
///     Demand per item type; ``len(qty) == len(items)``.
/// container : list[tuple[float, float]]
///     The container boundary outline.
/// holes : list[list[tuple[float, float]]]
///     Keep-out polygons inside the container that no part may overlap — interior voids, sheet
///     defects, or (to "nest inside a part") the solid region of an already-placed part. ``[]`` for
///     none.
/// min_sep : float
///     Minimum separation between any two parts (and part↔boundary); ``0.0`` disables it.
///     NOTE: nonzero separation is not yet byte-identical across platforms (the offsetter still
///     uses non-portable trig), so keep it ``0.0`` where reproducibility is required.
/// rotations : list[float]
///     Allowed discrete orientations in degrees (e.g. ``[0, 90, 180, 270]``); empty ⇒ no rotation.
/// seed : int
///     Explicit PRNG seed. There is **no** entropy fallback — determinism is the contract.
/// budget : int
///     Samples per item placement (a fixed budget; never a wall clock).
///
/// Returns
/// -------
/// tuple[list[tuple[int, float, float, float]], list[int]]
///     ``(placements, unplaced)`` where each placement is ``(item, x, y, rotation_deg)`` mapping the
///     item's original outline as ``placed = Rot(rotation_deg)·original + (x, y)``, and ``unplaced``
///     lists the item-type index of every instance that did not fit.
///
/// The same arguments always produce a byte-identical result.
#[pyfunction]
#[pyo3(signature = (items, qty, container, holes, min_sep, rotations, seed, budget))]
#[allow(clippy::needless_pass_by_value)] // PyO3 marshals owned Vecs out of the Python objects
#[allow(clippy::too_many_arguments)] // the oracle's full input surface; mirrors the Rust API
fn nest(
    items: Vec<Vec<[f64; 2]>>,
    qty: Vec<usize>,
    container: Vec<[f64; 2]>,
    holes: Vec<Vec<[f64; 2]>>,
    min_sep: f64,
    rotations: Vec<f64>,
    seed: u64,
    budget: u64,
) -> PyResult<(Vec<PyPlacement>, Vec<usize>)> {
    if items.len() != qty.len() {
        return Err(PyValueError::new_err(format!(
            "items ({}) and qty ({}) must have the same length",
            items.len(),
            qty.len()
        )));
    }

    let solution = nest_impl(
        &items, &qty, &container, &holes, min_sep, &rotations, seed, budget,
    );

    let placements = solution
        .placements
        .iter()
        .map(|p| (p.item, p.x, p.y, p.rotation_deg))
        .collect();

    Ok((placements, solution.unplaced))
}

/// Nest `items` (demand `qty`) across several `sheets` in order, spilling leftover demand to the next.
///
/// Parameters
/// ----------
/// items, qty, min_sep, rotations, seed, budget
///     As in [`nest`].
/// sheets : list[tuple[list[tuple[float, float]], list[list[tuple[float, float]]]]]
///     Each sheet is ``(outline, holes)`` — a boundary outline plus its keep-out holes.
///
/// Returns
/// -------
/// tuple[list[list[tuple[int, float, float, float]]], list[int]]
///     ``(per_sheet, unplaced)`` where ``per_sheet[i]`` are the placements on ``sheets[i]`` and
///     ``unplaced`` lists the item-type index of every instance that fit on no sheet.
///
/// Each sheet is nested with a seed derived deterministically from `seed`, so the result is
/// byte-identical for the same arguments.
#[pyfunction]
#[pyo3(signature = (items, qty, sheets, min_sep, rotations, seed, budget))]
#[allow(clippy::needless_pass_by_value)]
fn nest_multi(
    items: Vec<Vec<[f64; 2]>>,
    qty: Vec<usize>,
    sheets: Vec<PySheet>,
    min_sep: f64,
    rotations: Vec<f64>,
    seed: u64,
    budget: u64,
) -> PyResult<(Vec<Vec<PyPlacement>>, Vec<usize>)> {
    if items.len() != qty.len() {
        return Err(PyValueError::new_err(format!(
            "items ({}) and qty ({}) must have the same length",
            items.len(),
            qty.len()
        )));
    }

    let sheets: Vec<Sheet> = sheets
        .into_iter()
        .map(|(outline, holes)| Sheet { outline, holes })
        .collect();

    let solution = nest_multi_impl(&items, &qty, &sheets, min_sep, &rotations, seed, budget);

    let per_sheet = solution
        .per_sheet
        .iter()
        .map(|sheet| {
            sheet
                .iter()
                .map(|p| (p.item, p.x, p.y, p.rotation_deg))
                .collect()
        })
        .collect();

    Ok((per_sheet, solution.unplaced))
}

/// The `ironnest` Python module. (The function name must match `[lib] name` for the abi3
/// `PyInit_ironnest` symbol.)
#[pymodule]
fn ironnest(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    m.add_function(wrap_pyfunction!(nest, m)?)?;
    m.add_function(wrap_pyfunction!(nest_multi, m)?)?;
    Ok(())
}

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! PyO3 binding — the `ironnest` abi3-py313 wheel (`import ironnest`).
//!
//! Two `#[pyfunction]`s — [`nest`] (one container) and [`nest_multi`] (spill across sheets) —
//! wrapping [`ironnest_core::nest`] / [`ironnest_core::nest_multi`]. Polygons marshal as plain
//! `list[list[tuple[float, float]]]` (PyO3 `Vec<Vec<[f64; 2]>>`) — **no numpy, no JSON wire** (the
//! JSON wire is exactly the float-drift class that killed the old CLI stub). Python `float` *is* IEEE
//! f64, so the marshalling is exact and introduces no nondeterminism: the wheel inherits the engine's
//! byte-identical, cross-platform-reproducible output (proven by the Phase-3 golden).

use ironnest_core::{Sheet, nest_multi_per_item as nest_multi_impl, nest_per_item as nest_impl};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::types::PyString;

/// One placed instance returned to Python: `(item, x, y, rotation_deg)`.
type PyPlacement = (usize, f64, f64, f64);

/// One sheet for [`nest_multi`]: `(outline, holes)`.
type PySheet = (Vec<[f64; 2]>, Vec<Vec<[f64; 2]>>);

/// Parses the Python `rotations` argument into a per-item list of length `n_items` (the form the
/// engine's per-item entry consumes). Accepts EITHER:
/// * a flat `list[float]` — one orientation set applied to **every** item (broadcast); or
/// * a `list[list[float]]` — one set per item type (`len == n_items`, each inner list non-empty).
///
/// The two forms are told apart by the **type** of the first element (a number ⇒ uniform; a nested
/// sequence ⇒ per-item), never by length — so a single-item nest is unambiguous (`[0.0]` is uniform,
/// `[[0.0]]` is per-item). An empty OUTER list is the historical uniform "no rotation" default.
///
/// Raises `ValueError` (never silently coerces) on: a per-item length mismatch with `items`, an
/// empty inner set (a part must allow at least one orientation — pass `[0.0]` for "no rotation"), or
/// any non-numeric / non-finite (NaN, ±inf) angle.
fn parse_rotations(rotations: &Bound<'_, PyAny>, n_items: usize) -> PyResult<Vec<Vec<f64>>> {
    // A bare `str` is iterable (it yields characters) but is never a valid rotations value — reject
    // it up front with a clear message instead of letting it fail deep in per-character extraction.
    if rotations.is_instance_of::<PyString>() {
        return Err(PyValueError::new_err(
            "rotations must be a list[float] or a list[list[float]], not a string",
        ));
    }

    let outer: Vec<Bound<'_, PyAny>> = rotations
        .try_iter()
        .map_err(|_| {
            PyValueError::new_err("rotations must be a list[float] or a list[list[float]]")
        })?
        .collect::<PyResult<_>>()?;

    // Empty outer ⇒ uniform "no rotation" for every item (the historical meaning of `rotations=[]`).
    if outer.is_empty() {
        return Ok(vec![Vec::new(); n_items]);
    }

    if outer[0].extract::<f64>().is_ok() {
        // Uniform: a flat list[float] applied to every item.
        let angles = extract_angles(&outer, "rotations")?;
        Ok(vec![angles; n_items])
    } else {
        // Per-item: one list[float] per item type.
        if outer.len() != n_items {
            return Err(PyValueError::new_err(format!(
                "per-item rotations has {} entr{} but there {} {} item type(s); pass one rotation \
                 list per item, or a single list[float] applied to all",
                outer.len(),
                if outer.len() == 1 { "y" } else { "ies" },
                if n_items == 1 { "is" } else { "are" },
                n_items,
            )));
        }
        let mut per_item = Vec::with_capacity(n_items);
        for (k, inner) in outer.iter().enumerate() {
            let inner_elems: Vec<Bound<'_, PyAny>> = inner
                .try_iter()
                .map_err(|_| {
                    PyValueError::new_err(format!(
                        "rotations[{k}] must be a list[float] (a per-item orientation set)"
                    ))
                })?
                .collect::<PyResult<_>>()?;
            if inner_elems.is_empty() {
                return Err(PyValueError::new_err(format!(
                    "rotations[{k}] is empty; a part must allow at least one orientation \
                     (use [0.0] for no rotation)"
                )));
            }
            per_item.push(extract_angles(&inner_elems, &format!("rotations[{k}]"))?);
        }
        Ok(per_item)
    }
}

/// Extracts a finite-`f64` angle list from `elems`, raising `ValueError` on any non-numeric or
/// non-finite (NaN/±inf) entry. `ctx` names the list in error messages (e.g. `"rotations[2]"`).
fn extract_angles(elems: &[Bound<'_, PyAny>], ctx: &str) -> PyResult<Vec<f64>> {
    let mut out = Vec::with_capacity(elems.len());
    for (i, e) in elems.iter().enumerate() {
        let angle: f64 = e
            .extract()
            .map_err(|_| PyValueError::new_err(format!("{ctx}[{i}] is not a real number")))?;
        if !angle.is_finite() {
            return Err(PyValueError::new_err(format!(
                "{ctx}[{i}] = {angle} is not a finite angle (NaN and ±inf are not allowed)"
            )));
        }
        out.push(angle);
    }
    Ok(out)
}

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
///     Minimum separation enforced part↔part, part↔boundary, and part↔hole; ``0.0`` disables it.
///     Any value is fully cross-platform byte-identical: the separation offset is computed by the
///     vendored straight-skeleton offsetter with all trig routed through ``libm`` (a nonzero-
///     ``min_sep`` case is in the cross-platform determinism golden). A production part gap is the
///     intended use — there is no reproducibility reason to keep it ``0.0``.
/// rotations : list[float] | list[list[float]]
///     Allowed discrete orientations **in degrees**, in either of two forms:
///
///     * ``list[float]`` — one set applied to *every* item (e.g. ``[0, 90, 180, 270]``); ``[]`` ⇒
///       no rotation. This is the original, uniform form.
///     * ``list[list[float]]`` — one set *per item type*, so ``len(rotations) == len(items)`` and
///       ``rotations[k]`` is the allowed-orientation set for ``items[k]``. Lets different shapes use
///       different orientations in one nest (e.g. rectangles pinned to ``[0, 90]``, right triangles
///       free to interlock on ``[0, 45, 90, 135, 180, 225, 270, 315]``). Each inner list must be
///       non-empty — use ``[0.0]`` for "do not rotate this part".
///
///     The form is decided by the *type* of the first element (a number ⇒ uniform; a list ⇒
///     per-item), never by length, so a single-item nest is unambiguous. A returned placement's
///     rotation for item ``k`` is always a member of that item's set.
///
///     Every listed angle is applied cross-platform-deterministically — the rotation trig routes
///     through ``libm`` (byte-identical on every target), so a non-cardinal angle like ``45`` is
///     just as reproducible. The cardinal set ``{0, 90, 180, 270}`` is nonetheless recommended where
///     it suffices: those orientations also yield coordinate-exact placements (integer rotation
///     matrix, no sub-ULP trig fuzz), which is the cleanest input for a cut-realizable audit trail.
///
///     Raises ``ValueError`` on a per-item length mismatch, an empty inner set, or any non-numeric
///     or non-finite (NaN/±inf) angle.
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
#[allow(clippy::too_many_arguments)] // injected `py` GIL token + the oracle's input surface (mirrors the Rust API)
fn nest(
    py: Python<'_>,
    items: Vec<Vec<[f64; 2]>>,
    qty: Vec<usize>,
    container: Vec<[f64; 2]>,
    holes: Vec<Vec<[f64; 2]>>,
    min_sep: f64,
    rotations: Bound<'_, PyAny>,
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

    // Resolve the uniform-or-per-item `rotations` into one set per item BEFORE releasing the GIL —
    // it touches Python objects, so it must run while the GIL is held. The result is a plain, owned
    // `Vec<Vec<f64>>` the `Ungil` solve closure can take by reference.
    let rotations = parse_rotations(&rotations, items.len())?;

    // Release the GIL for the (synchronous, potentially long) solve so the caller's other Python
    // threads keep running — without this a worker-thread nest freezes the whole interpreter: the
    // asyncio event loop stalls ("connection lost") and SIGINT/Ctrl+C handling dies. Every input is
    // an owned, `Send` Rust `Vec` and the returned `Solution` is plain data, so the closure is
    // `Ungil` and this is the textbook PyO3 release pattern. It does NOT touch the determinism
    // contract: no threads run *inside* the solve (single canonical worker, CLAUDE.md), so output
    // stays byte-identical.
    let solution = py.detach(|| {
        nest_impl(
            &items, &qty, &container, &holes, min_sep, &rotations, seed, budget,
        )
    });

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
///     As in [`nest`]. In particular ``rotations`` accepts both the uniform ``list[float]`` and the
///     per-item ``list[list[float]]`` form (with ``len(rotations) == len(items)``), validated
///     identically.
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
#[allow(clippy::too_many_arguments)] // `py` is the injected GIL token (not an oracle input); the rest are the oracle's input surface
fn nest_multi(
    py: Python<'_>,
    items: Vec<Vec<[f64; 2]>>,
    qty: Vec<usize>,
    sheets: Vec<PySheet>,
    min_sep: f64,
    rotations: Bound<'_, PyAny>,
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

    // Resolve `rotations` (uniform or per-item) into one set per item while the GIL is held — see the
    // note in `nest`.
    let rotations = parse_rotations(&rotations, items.len())?;

    let sheets: Vec<Sheet> = sheets
        .into_iter()
        .map(|(outline, holes)| Sheet { outline, holes })
        .collect();

    // Release the GIL for the solve — see the note in `nest` for the why and the safety argument.
    let solution =
        py.detach(|| nest_multi_impl(&items, &qty, &sheets, min_sep, &rotations, seed, budget));

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
    // Build provenance for the consumer's per-cut audit sidecar (issue #258 `engine{}` block): the
    // upstream jagua-rs commit this CDE was forked from, and this wheel's own build commit. Both are
    // metadata strings that never touch the placement path — they do not affect the determinism
    // contract. `__commit__` is injected by CI via the `IRONNEST_GIT_SHA` env (`option_env!` reads it
    // at compile time — no `build.rs`, per CLAUDE.md); local/sdist builds without it read "unknown".
    m.add("__jagua_fork_rev__", "43e8137")?; // upstream jagua-rs 0.7.2 base — see docs/01
    m.add(
        "__commit__",
        option_env!("IRONNEST_GIT_SHA").unwrap_or("unknown"),
    )?;
    m.add_function(wrap_pyfunction!(nest, m)?)?;
    m.add_function(wrap_pyfunction!(nest_multi, m)?)?;
    Ok(())
}

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Canonical solver-output dump for the cross-platform determinism golden (Phase 3, the headline
//! contract: byte-identical placements on macOS-arm64 == Windows-x64 == linux-x64).
//!
//! Runs a fixed corpus of nests and prints the placements as plain `item x y rot` lines. This is the
//! single source of truth for both golden tests (`tests/golden.rs`): the insta snapshot (level 3 +
//! the cross-platform gate — every CI platform must reproduce the committed `.snap`) and the
//! cross-subprocess byte-diff (level 2). Run it by hand to inspect / re-bless:
//! `cargo run -p ironnest-optimizer --bin golden_dump`.
//!
//! DETERMINISM(ironnest):
//! - **`min_sep = 0` on every case** — nonzero separation routes through `geo-buffer`, which still
//!   calls std `sin`/`cos` (docs/00 risk #2), so those layouts are *not yet* byte-identical
//!   cross-platform. The golden stays strictly inside the proven-deterministic envelope.
//! - Coordinates are printed with the default `f64` `Display` (Rust's pure-Rust shortest-round-trip
//!   `flt2dec`), which is a deterministic, **injective** function of the bits: two different f64
//!   values never print the same string, so the text diff is a faithful byte-identity check.
//! - The placement emit order is `nest`'s own order (slotmap slot order — a pure function of the
//!   deterministic op history); it is part of the contract, so it is dumped unsorted.

use ironnest_optimizer::{Placement, Scalar, nest};
use std::fmt::Write as _;

/// `w × h` axis-aligned rectangle, lower-left at the origin (CCW).
fn rect(w: Scalar, h: Scalar) -> Vec<[Scalar; 2]> {
    vec![[0.0, 0.0], [w, 0.0], [w, h], [0.0, h]]
}

const CARDINAL: [Scalar; 4] = [0.0, 90.0, 180.0, 270.0];

/// One golden case. Everything here is fixed — seeds, budgets, geometry — so the output is a pure
/// function of the engine.
struct Case {
    name: &'static str,
    items: Vec<Vec<[Scalar; 2]>>,
    qty: Vec<usize>,
    container: Vec<[Scalar; 2]>,
    holes: Vec<Vec<[Scalar; 2]>>,
    rotations: Vec<Scalar>,
    seed: u64,
    budget: u64,
}

fn corpus() -> Vec<Case> {
    let pentagon = vec![
        [0.0, 0.0],
        [20.0, 0.0],
        [20.0, 12.0],
        [12.0, 20.0],
        [0.0, 20.0],
    ];
    let right_tri = vec![[0.0, 0.0], [10.0, 0.0], [0.0, 10.0]];
    // A 60×60 sheet with a central 20×20 keep-out hole at (20,20)–(40,40): parts must nest around it.
    let center_hole = vec![[20.0, 20.0], [40.0, 20.0], [40.0, 40.0], [20.0, 40.0]];
    vec![
        // Construction + compaction over two part types and full cardinal rotation.
        Case {
            name: "mixed-rects",
            items: vec![rect(10.0, 10.0), rect(20.0, 5.0)],
            qty: vec![6, 4],
            container: rect(60.0, 60.0),
            holes: vec![],
            rotations: CARDINAL.to_vec(),
            seed: 7,
            budget: 1500,
        },
        // The no-rotation path (rotations empty ⇒ 0° only).
        Case {
            name: "no-rotation-squares",
            items: vec![rect(10.0, 10.0)],
            qty: vec![4],
            container: rect(50.0, 50.0),
            holes: vec![],
            rotations: vec![],
            seed: 1,
            budget: 800,
        },
        // Separation search must discover the interlocked pairing (one triangle rotated 180°).
        Case {
            name: "interlock-triangles",
            items: vec![right_tri],
            qty: vec![2],
            container: rect(11.0, 11.0),
            holes: vec![],
            rotations: CARDINAL.to_vec(),
            seed: 42,
            budget: 2000,
        },
        // Separation search on an irregular part at meaningful demand.
        Case {
            name: "pentagon",
            items: vec![pentagon],
            qty: vec![10],
            container: rect(80.0, 80.0),
            holes: vec![],
            rotations: CARDINAL.to_vec(),
            seed: 3,
            budget: 2000,
        },
        // Interior-void path: parts must avoid the central keep-out hole (Phase 6). Quantity kept
        // low so construction places them all (no slow debug separation) — the point is to exercise
        // the holes path deterministically in the cross-platform golden.
        Case {
            name: "sheet-with-hole",
            items: vec![rect(10.0, 10.0)],
            qty: vec![12],
            container: rect(60.0, 60.0),
            holes: vec![center_hole],
            rotations: CARDINAL.to_vec(),
            seed: 5,
            budget: 1000,
        },
    ]
}

/// Renders the full corpus to canonical text. Each case is a `# name` header, one `item x y rot`
/// line per placement (in `nest`'s emit order), then an `unplaced <ids…>` line.
fn dump() -> String {
    let mut out = String::new();
    for case in corpus() {
        writeln!(out, "# {}", case.name).unwrap();
        let sol = nest(
            &case.items,
            &case.qty,
            &case.container,
            &case.holes,
            0.0, // min_sep = 0 — see the module note (geo-buffer is not yet byte-deterministic)
            &case.rotations,
            case.seed,
            case.budget,
        );
        for Placement {
            item,
            x,
            y,
            rotation_deg,
        } in &sol.placements
        {
            writeln!(out, "{item} {x} {y} {rotation_deg}").unwrap();
        }
        write!(out, "unplaced").unwrap();
        for id in &sol.unplaced {
            write!(out, " {id}").unwrap();
        }
        writeln!(out).unwrap();
    }
    out
}

fn main() {
    print!("{}", dump());
}

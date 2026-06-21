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
//! - The corpus includes a **nonzero-`min_sep`** case (`separated-squares`). Separation routes
//!   through the *vendored, libm-deterministic* offsetter (`ironnest_geo::buffer`, ex-`geo-buffer`),
//!   so these layouts are now byte-identical cross-platform too — this case is the standing proof
//!   that docs/00 risk #2 is resolved.
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

/// A 64-vertex CCW "circle" of radius `r` centered at the origin — a high-vertex curved part that
/// stands in for the consumer's hundreds-of-vertex developed-cone shells and triggers collision
/// decimation (64 > `DECIMATION_MIN_VERTICES`).
///
/// DETERMINISM(ironnest): the golden_dump output must be byte-identical on every platform, so the
/// *input geometry* must be too. We therefore build the vertices with a fixed rotation recurrence
/// using only `+ − ×` on the **f64 literals** `cos(π/32)` / `sin(π/32)` — never std `sin`/`cos` at
/// dump time (whose platform libm would diverge). Literals + IEEE arithmetic ⇒ identical bits
/// everywhere; the recurrence's sub-ULP radius drift over 64 steps is immaterial (it is still a fixed,
/// valid, curved polygon — the point is a many-vertex convex outline, not a perfect circle).
fn circle_64(r: Scalar) -> Vec<[Scalar; 2]> {
    const C: Scalar = 0.9951847266721969; // cos(π/32)
    const S: Scalar = 0.0980171403295606; // sin(π/32)
    let mut pts = Vec::with_capacity(64);
    let (mut x, mut y) = (r, 0.0);
    for _ in 0..64 {
        pts.push([x, y]);
        (x, y) = (C * x - S * y, S * x + C * y);
    }
    pts
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
    min_sep: Scalar,
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
            min_sep: 0.0,
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
            min_sep: 0.0,
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
            min_sep: 0.0,
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
            min_sep: 0.0,
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
            min_sep: 0.0,
            rotations: CARDINAL.to_vec(),
            seed: 5,
            budget: 1000,
        },
        // Nonzero min-separation path: each part is inflated by min_sep/2 via the vendored, libm-
        // deterministic offsetter (ex-geo-buffer). This is the proof that resolving docs/00 risk #2
        // makes separated layouts byte-identical across platforms.
        Case {
            name: "separated-squares",
            items: vec![rect(10.0, 10.0)],
            qty: vec![4],
            container: rect(60.0, 60.0),
            holes: vec![],
            min_sep: 4.0,
            rotations: CARDINAL.to_vec(),
            seed: 8,
            budget: 1000,
        },
        // Collision-footprint decimation path (Inflate / item side): a high-vertex (64-gon) curved part
        // at nonzero min_sep. Its collision footprint is Douglas–Peucker-simplified then offset by
        // min_sep/2 + tol — both the DP (pure +−×÷) and the offset (vendored libm offsetter) are
        // cross-platform-deterministic, so this layout is byte-identical on every target too. The
        // reported placements are in the *original* 64-gon frame. (Proof decimation kept determinism.)
        Case {
            name: "decimated-circles",
            items: vec![circle_64(12.0)],
            qty: vec![4],
            container: rect(60.0, 60.0),
            holes: vec![],
            min_sep: 1.0,
            rotations: CARDINAL.to_vec(),
            seed: 8,
            budget: 1000,
        },
        // Decimation path on the Deflate / container side: a high-vertex (64-gon) curved CONTAINER is
        // DP-simplified then deflated by min_sep/2 + tol. Exercises the symmetric container/boundary
        // over-reservation across platforms; simple square parts (4 vtx) stay on the exact path.
        Case {
            name: "decimated-curved-container",
            items: vec![rect(8.0, 8.0)],
            qty: vec![6],
            container: circle_64(30.0),
            holes: vec![],
            min_sep: 1.0,
            rotations: CARDINAL.to_vec(),
            seed: 4,
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
            case.min_sep,
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

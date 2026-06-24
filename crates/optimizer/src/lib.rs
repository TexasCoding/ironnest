// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! ironnest-optimizer — OUR deterministic placement search (new code; the brain).
//!
//! Milestone 2a (this module) is a **deterministic constructive nester**: order items by descending
//! size, then for each item sample candidate poses over the container, fail-fast via the CDE
//! surrogate, keep the lowest-[`loss`](crate::loss) feasible pose (a Left-Bottom-Fill preference),
//! and place it. Driven by a **fixed sample budget, never a wall clock**; all randomness comes from
//! the self-contained portable [`Prng`](crate::prng::Prng). Rotations are a caller-supplied discrete
//! set (default `{0,90,180,270}`, configurable per call). Built on [`ironnest_cde`].
//!
//! Milestone 2b (next) adds a sparrow-style separation / overlap-minimization local search on top.
#![warn(
    clippy::pedantic,
    clippy::correctness,
    clippy::suspicious,
    clippy::complexity,
    clippy::perf,
    clippy::style,
    clippy::disallowed_types,
    clippy::disallowed_methods
)]
#![allow(clippy::missing_panics_doc, clippy::missing_errors_doc)]

mod improve;
mod loss;
mod prng;
mod search;
mod sep;

pub use ironnest_geo::Scalar;
pub use prng::Prng;

/// Number of improvement rounds run after the constructive fill (each round compacts every placed
/// item, then tries to place any still-unplaced items in the freed space). Fixed — never a wall clock.
const IMPROVE_ROUNDS: u32 = 3;

/// Douglas–Peucker tolerance for collision-footprint decimation, as a fraction of `min_sep`. Every
/// hazard's footprint (items inflate, container/holes deflate/inflate) is offset by `min_sep/2 + tol`
/// instead of `min_sep/2` on the high-vertex curved path (the per-shape vertex gate lives in
/// `ironnest_cde::geometry`'s `DECIMATION_MIN_VERTICES`). So `tol` is the over-reservation margin
/// that (a) compensates DP's bounded inward deviation and (b) comfortably exceeds the offsetter's
/// sub-mil polygonal curve deficit (`tol` is ~100× it) — keeping the placed *original* outlines ≥
/// `min_sep` apart from each other AND from a curved container boundary. `min_sep/16` ≈ 0.023 at the
/// consumer-validated 0.375-in `min_sep` (their validated 0.02-in tol), a negligible density cost
/// relative to the curved-part speedup.
const DECIMATION_TOL_FRACTION: Scalar = 1.0 / 16.0;

use std::cmp::Ordering;

use ironnest_cde::collision_detection::CDEConfig;
use ironnest_cde::entities::{Container, Item, Layout};
use ironnest_cde::geometry::fail_fast::SPSurrogateConfig;
use ironnest_cde::geometry::primitives::Rect;
use ironnest_cde::io::ext_repr::{ExtContainer, ExtItem, ExtQualityZone, ExtSPolygon, ExtShape};
use ironnest_cde::io::import::Importer;

/// A single resolved placement — the only thing the oracle emits.
///
/// The pose maps the item's *original* (caller-supplied) outline into container coordinates:
/// `placed_point = Rot(rotation_deg)·original_point + (x, y)`. No anchor knowledge is required by
/// the consumer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Placement {
    /// Index into the caller's `items` slice.
    pub item: usize,
    /// Placement origin X, in the container's coordinate space.
    pub x: Scalar,
    /// Placement origin Y, in the container's coordinate space.
    pub y: Scalar,
    /// Rotation in degrees; always a member of the caller's allowed set.
    pub rotation_deg: Scalar,
}

/// The result of a nest: every placed instance, plus the item-type index of every instance that did
/// not fit (one entry per unplaced instance).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct NestSolution {
    pub placements: Vec<Placement>,
    pub unplaced: Vec<usize>,
}

/// One sheet for a multi-sheet nest: a boundary outline plus optional keep-out holes.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Sheet {
    /// The sheet boundary outline.
    pub outline: Vec<[Scalar; 2]>,
    /// Keep-out polygons inside the sheet that no part may overlap (see [`nest`]'s `holes`).
    pub holes: Vec<Vec<[Scalar; 2]>>,
}

/// The result of a multi-sheet nest: the placements on each sheet (parallel to the input `sheets`),
/// plus the item-type index of every instance that fit on no sheet.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MultiSheetSolution {
    /// `per_sheet[i]` are the placements on `sheets[i]`.
    pub per_sheet: Vec<Vec<Placement>>,
    pub unplaced: Vec<usize>,
}

/// The collision-detection configuration. Mirrors `lbf`'s reference defaults; internal for 2a.
fn default_cde_config() -> CDEConfig {
    CDEConfig {
        quadtree_depth: 5,
        cd_threshold: 16,
        item_surrogate_config: SPSurrogateConfig {
            n_pole_limits: [(100, 0.0), (20, 0.75), (10, 0.90)],
            n_ff_poles: 2,
            n_ff_piers: 0,
        },
    }
}

fn ext_spolygon(outline: &[[Scalar; 2]]) -> ExtSPolygon {
    ExtSPolygon(outline.iter().map(|p| (p[0], p[1])).collect())
}

/// Clips polygon `poly` to the axis-aligned rectangle `r` (Sutherland–Hodgman against the four
/// half-planes). Returns `None` if the result has fewer than 3 vertices (the polygon lay outside
/// `r`). Deterministic — only `+ − × ÷` and comparisons; division is reached only on a segment that
/// straddles an edge (non-parallel), so it never divides by zero. A polygon already inside `r` is
/// returned vertex-for-vertex unchanged.
fn clip_polygon_to_rect(poly: &[[Scalar; 2]], r: Rect) -> Option<Vec<[Scalar; 2]>> {
    // 0=left (x≥x_min), 1=right (x≤x_max), 2=bottom (y≥y_min), 3=top (y≤y_max).
    let inside = |p: [Scalar; 2], edge: u8| -> bool {
        match edge {
            0 => p[0] >= r.x_min,
            1 => p[0] <= r.x_max,
            2 => p[1] >= r.y_min,
            _ => p[1] <= r.y_max,
        }
    };
    let intersect = |a: [Scalar; 2], b: [Scalar; 2], edge: u8| -> [Scalar; 2] {
        match edge {
            0 => {
                let t = (r.x_min - a[0]) / (b[0] - a[0]);
                [r.x_min, a[1] + t * (b[1] - a[1])]
            }
            1 => {
                let t = (r.x_max - a[0]) / (b[0] - a[0]);
                [r.x_max, a[1] + t * (b[1] - a[1])]
            }
            2 => {
                let t = (r.y_min - a[1]) / (b[1] - a[1]);
                [a[0] + t * (b[0] - a[0]), r.y_min]
            }
            _ => {
                let t = (r.y_max - a[1]) / (b[1] - a[1]);
                [a[0] + t * (b[0] - a[0]), r.y_max]
            }
        }
    };

    let mut out = poly.to_vec();
    for edge in 0..4u8 {
        if out.len() < 3 {
            return None;
        }
        let input = std::mem::take(&mut out);
        let n = input.len();
        for i in 0..n {
            let cur = input[i];
            let prev = input[(i + n - 1) % n];
            let (cur_in, prev_in) = (inside(cur, edge), inside(prev, edge));
            if cur_in {
                if !prev_in {
                    out.push(intersect(prev, cur, edge));
                }
                out.push(cur);
            } else if prev_in {
                out.push(intersect(prev, cur, edge));
            }
        }
    }
    (out.len() >= 3).then_some(out)
}

/// Imports the container with `holes` as quality-0 keep-out zones, **clipping each hole to the
/// quadtree root** so an out-of-bounds hole cannot trip the CDE's constrict invariant. Returns `None`
/// if the container — or any in-bounds hole — is malformed (⇒ the caller places nothing). In-bounds
/// holes pass through unchanged (so the determinism golden is untouched); an enclosing hole clips to
/// a full-sheet keep-out (⇒ nothing fits); an entirely-outside hole is dropped (unreachable space).
fn import_container_with_holes(
    importer: &Importer,
    container: &[[Scalar; 2]],
    holes: &[Vec<[Scalar; 2]>],
) -> Option<Container> {
    let bare_ext = ExtContainer {
        id: 0,
        shape: ExtShape::SimplePolygon(ext_spolygon(container)),
        zones: vec![],
    };
    let bare = importer.import_container(&bare_ext).ok()?;
    if holes.is_empty() {
        return Some(bare);
    }

    let root = bare.base_cde.bbox();
    let zones: Vec<ExtQualityZone> = holes
        .iter()
        .filter_map(|hole| {
            let in_bounds = hole.iter().all(|p| {
                p[0] >= root.x_min && p[0] <= root.x_max && p[1] >= root.y_min && p[1] <= root.y_max
            });
            let shape = if in_bounds {
                ext_spolygon(hole)
            } else {
                ext_spolygon(&clip_polygon_to_rect(hole, root)?)
            };
            Some(ExtQualityZone {
                quality: 0,
                shape: ExtShape::SimplePolygon(shape),
            })
        })
        .collect();

    let ext = ExtContainer {
        id: 0,
        shape: ExtShape::SimplePolygon(ext_spolygon(container)),
        zones,
    };
    importer.import_container(&ext).ok()
}

/// Nests `items` (each with a demand `qty`) into a single irregular `container` with optional
/// `holes` (keep-out zones the parts must avoid), applying **one** allowed-rotation set to every item.
///
/// * `items` — one outline per item *type*, in item-local coordinates.
/// * `qty` — demand per item type (`qty.len() == items.len()`).
/// * `container` — the container boundary outline.
/// * `holes` — keep-out polygons inside the container that no part may overlap (interior voids,
///   sheet defects, or — to "nest inside a part" — the solid region of an already-placed part,
///   leaving its void nestable). Empty ⇒ no holes. Imported as quality-0 zones.
/// * `min_sep` — minimum separation between any two parts (and part↔boundary↔hole); `0.0` to disable.
/// * `rotations_deg` — allowed discrete orientations in degrees (e.g. `[0.0, 90.0, 180.0, 270.0]`),
///   applied to **every** item; empty ⇒ no rotation. For a distinct set per item type use
///   [`nest_per_item`].
/// * `seed` — explicit PRNG seed (no implicit/entropy fallback, ever).
/// * `budget` — samples per item placement (fixed; never a wall clock).
///
/// Determinism: the same arguments always produce a byte-identical [`NestSolution`].
#[must_use]
#[allow(clippy::too_many_arguments)]
pub fn nest(
    items: &[Vec<[Scalar; 2]>],
    qty: &[usize],
    container: &[[Scalar; 2]],
    holes: &[Vec<[Scalar; 2]>],
    min_sep: Scalar,
    rotations_deg: &[Scalar],
    seed: u64,
    budget: u64,
) -> NestSolution {
    // Broadcast the single rotation set to every item, then run the per-item path. Broadcasting is
    // byte-for-byte identical to threading one global set through the search — every item sees the
    // same orientations and so consumes the same PRNG draws — which is what keeps the existing
    // determinism golden valid through this refactor.
    let rotations_per_item = vec![rotations_deg.to_vec(); items.len()];
    nest_per_item(
        items,
        qty,
        container,
        holes,
        min_sep,
        &rotations_per_item,
        seed,
        budget,
    )
}

/// Like [`nest`], but with a **distinct allowed-rotation set per item type**: `rotations_deg[k]` is
/// the orientation set (degrees) for `items[k]`, so `rotations_deg.len() == items.len()`. An empty
/// inner set ⇒ that part is not rotated (the same `{0}` normalization [`nest`] applies, now per item).
/// Lets different shapes use different orientations in one nest — e.g. rectangles pinned axis-aligned
/// (`[0.0, 90.0]`), triangles free to interlock (`[0, 45, …, 315]`). Same determinism contract as
/// [`nest`]; arbitrary (non-cardinal) angles route through the same `libm` trig and stay byte-stable.
#[must_use]
#[allow(clippy::too_many_arguments)]
pub fn nest_per_item(
    items: &[Vec<[Scalar; 2]>],
    qty: &[usize],
    container: &[[Scalar; 2]],
    holes: &[Vec<[Scalar; 2]>],
    min_sep: Scalar,
    rotations_deg: &[Vec<Scalar>],
    seed: u64,
    budget: u64,
) -> NestSolution {
    assert_eq!(
        items.len(),
        qty.len(),
        "items and qty must be the same length"
    );
    assert_eq!(
        items.len(),
        rotations_deg.len(),
        "items and per-item rotations must be the same length"
    );

    let cde_config = default_cde_config();
    let min_item_separation = (min_sep > 0.0).then_some(min_sep);
    let mut importer = Importer::new(cde_config, None, min_item_separation, None);
    // Auto collision-footprint decimation: enabled whenever a separation is requested (`min_sep == 0`
    // needs no offset and so no superset margin). Applied symmetrically to every imported hazard —
    // items (Inflate), the container boundary and holes (Deflate/Inflate) — but only takes effect on
    // HIGH-VERTEX shapes; `convert_to_internal` gates per-shape on `DECIMATION_MIN_VERTICES`, so simple
    // parts and axis-aligned sheets keep the exact offset path (and the determinism golden) unchanged.
    importer.shape_modify_config.collision_decimation =
        (min_sep > 0.0).then_some(min_sep * DECIMATION_TOL_FRACTION);

    // Per-item rotations: degrees for the importer's metadata, radians for our sampler. An empty set
    // for an item ⇒ `{0}` (no rotation) — the same normalization the single-set path applies, now
    // applied independently per item type.
    let rotations_deg_per_item: Vec<Vec<Scalar>> = rotations_deg
        .iter()
        .map(|r| if r.is_empty() { vec![0.0] } else { r.clone() })
        .collect();
    let rotations_rad_per_item: Vec<Vec<Scalar>> = rotations_deg_per_item
        .iter()
        .map(|r| r.iter().map(|d| d.to_radians()).collect())
        .collect();

    // Import the container (with its holes as quality-0 keep-out zones). A malformed container or
    // hole ⇒ nothing can be placed (conservative — never silently place into an intended keep-out).
    let Some(container_entity) = import_container_with_holes(&importer, container, holes) else {
        return NestSolution {
            placements: vec![],
            unplaced: all_unplaced(qty),
        };
    };

    // Import items. A malformed item ⇒ that whole type is unplaced. High-vertex curved parts get
    // their collision footprint decimated by the importer's `collision_decimation` (gated per-shape in
    // `convert_to_internal`); `shape_orig` — which drives the reported placement frame — is always the
    // untouched original outline. Each item carries its own `allowed_orientations`, so the separation
    // search (which reads `item.allowed_rotation`) honours the per-item set automatically.
    let mut entities: Vec<Option<Item>> = Vec::with_capacity(items.len());
    for (i, outline) in items.iter().enumerate() {
        let ext_item = ExtItem {
            id: i as u64,
            allowed_orientations: Some(rotations_deg_per_item[i].clone()),
            shape: ExtShape::SimplePolygon(ext_spolygon(outline)),
            min_quality: None,
        };
        entities.push(importer.import_item(&ext_item).ok());
    }

    let mut layout = Layout::new(container_entity);
    let mut prng = Prng::seed_from_u64(seed);

    // Placement order: largest items first (descending CD-shape diameter); stable on ties.
    let mut order: Vec<usize> = (0..items.len())
        .filter(|&i| entities[i].is_some())
        .collect();
    order.sort_by(|&a, &b| {
        let da = entities[a].as_ref().unwrap().shape_cd.diameter;
        let db = entities[b].as_ref().unwrap().shape_cd.diameter;
        // descending; diameters are finite (valid polygons) so partial_cmp never returns None.
        db.partial_cmp(&da).unwrap()
    });

    let mut placed_per_type = vec![0usize; items.len()];
    constructive_fill(
        &mut layout,
        &entities,
        &order,
        qty,
        &rotations_rad_per_item,
        &mut prng,
        budget,
        &mut placed_per_type,
    );

    // Improvement: compact placed items toward the bottom-left, then fill freed space. Fixed number
    // of rounds (no wall clock); deterministic via the same seeded PRNG stream.
    improve::improve(
        &mut layout,
        &entities,
        &order,
        qty,
        &rotations_rad_per_item,
        &mut prng,
        budget,
        IMPROVE_ROUNDS,
        &mut placed_per_type,
    );

    // Separation search (Phase 2b): for any still-unplaced part, allow overlap then shove neighbours
    // apart (sparrow GLS) to discover interlocking arrangements greedy construction cannot reach. A
    // no-op when everything already placed, so the dense rectangular cases pay nothing.
    sep::run_separation(
        &mut layout,
        &entities,
        &order,
        qty,
        &mut prng,
        &mut placed_per_type,
    );

    let placements = extract_placements(&layout, &entities);

    let mut unplaced = vec![];
    for (i, (&want, &got)) in qty.iter().zip(&placed_per_type).enumerate() {
        for _ in got..want {
            unplaced.push(i);
        }
    }

    NestSolution {
        placements,
        unplaced,
    }
}

/// Runs the nest from `n_starts` decorrelated seeds and returns the **densest** result — the
/// deterministic best-of-K "multi-start". A single greedy construction lands in a *seed-dependent
/// basin*; sweeping several seeds and keeping the best lifts packing on heterogeneous parts (measured:
/// 13×7 bricks 91.9 % → 95.5 % at K=16) at no determinism cost. Applies **one** rotation set to every
/// item; for a per-item set use [`nest_multistart_per_item`].
///
/// * `n_starts` — number of independent constructions (clamped to ≥ 1). **`n_starts == 1` is
///   byte-identical to [`nest`]** with the same seed, so the existing determinism golden is untouched.
/// * every other argument carries its [`nest`] meaning. Start *k* uses `seed.wrapping_add(k)`; the
///   PRNG's SplitMix64 expansion decorrelates adjacent seeds into well-separated streams.
///
/// Determinism: each start is the byte-stable [`nest`] pipeline; the keep-best reduction maximises the
/// total placed **area** (a deterministic, fixed-order float sum compared via `total_cmp`) and keeps
/// the earliest *k* on a tie — the chosen layout is byte-identical for the same arguments.
#[must_use]
#[allow(clippy::too_many_arguments)]
pub fn nest_multistart(
    items: &[Vec<[Scalar; 2]>],
    qty: &[usize],
    container: &[[Scalar; 2]],
    holes: &[Vec<[Scalar; 2]>],
    min_sep: Scalar,
    rotations_deg: &[Scalar],
    seed: u64,
    budget: u64,
    n_starts: usize,
) -> NestSolution {
    let rotations_per_item = vec![rotations_deg.to_vec(); items.len()];
    nest_multistart_per_item(
        items,
        qty,
        container,
        holes,
        min_sep,
        &rotations_per_item,
        seed,
        budget,
        n_starts,
    )
}

/// [`nest_multistart`] with a **distinct allowed-rotation set per item type** (the [`nest_per_item`]
/// semantics): `rotations_deg[k]` applies to `items[k]`. Same best-of-K determinism contract.
#[must_use]
#[allow(clippy::too_many_arguments)]
pub fn nest_multistart_per_item(
    items: &[Vec<[Scalar; 2]>],
    qty: &[usize],
    container: &[[Scalar; 2]],
    holes: &[Vec<[Scalar; 2]>],
    min_sep: Scalar,
    rotations_deg: &[Vec<Scalar>],
    seed: u64,
    budget: u64,
    n_starts: usize,
) -> NestSolution {
    let starts = n_starts.max(1);
    let areas: Vec<Scalar> = items.iter().map(|o| polygon_area(o)).collect();

    // Run the K independent starts (sequentially by default; on threads under the `parallel` feature —
    // byte-identical either way). Returned as `(solution, placed_area)` in k = 0..starts order.
    let runs = run_starts(
        items,
        qty,
        container,
        holes,
        min_sep,
        rotations_deg,
        seed,
        budget,
        starts,
        &areas,
    );

    // Keep-best reduction in canonical k order: strictly-greater placed area wins, an exact tie keeps
    // the earliest k (lowest seed offset). A single `total_cmp` per step on each solution's own
    // canonical area sum — no cross-solution/cross-thread float reduction — so the chosen layout is
    // byte-identical for the same arguments, sequential or parallel.
    let mut best: Option<(NestSolution, Scalar)> = None;
    for (sol, area) in runs {
        let better = best
            .as_ref()
            .is_none_or(|(_, best_area)| area.total_cmp(best_area) == Ordering::Greater);
        if better {
            best = Some((sol, area));
        }
    }
    best.expect("n_starts is clamped to >= 1, so at least one solution is produced")
        .0
}

/// Runs `starts` independent constructions, start *k* at `seed.wrapping_add(k)`, returning
/// `(solution, placed_area)` in `0..starts` order. **Sequential** build: a plain loop.
#[cfg(not(feature = "parallel"))]
#[allow(clippy::too_many_arguments)]
fn run_starts(
    items: &[Vec<[Scalar; 2]>],
    qty: &[usize],
    container: &[[Scalar; 2]],
    holes: &[Vec<[Scalar; 2]>],
    min_sep: Scalar,
    rotations_deg: &[Vec<Scalar>],
    seed: u64,
    budget: u64,
    starts: usize,
    areas: &[Scalar],
) -> Vec<(NestSolution, Scalar)> {
    (0..starts as u64)
        .map(|k| {
            let sol = nest_per_item(
                items,
                qty,
                container,
                holes,
                min_sep,
                rotations_deg,
                seed.wrapping_add(k),
                budget,
            );
            let area = placed_area(&sol, areas);
            (sol, area)
        })
        .collect()
}

/// Runs `starts` independent constructions concurrently on scoped threads (one per start), returning
/// `(solution, placed_area)` in `0..starts` order — **byte-identical** to the sequential build: each
/// start is the self-contained, golden-stable [`nest_per_item`] (own PRNG, own `Layout`, no shared
/// mutable state), the results are collected in `k` order via ordered join handles, and the caller's
/// keep-best reduction runs afterwards in that same fixed order, so completion order is irrelevant and
/// no float sum ever crosses a thread. This is the one sanctioned use of threads (the multi-start
/// meta-loop only — never inside a placement search); the cross-platform golden run under
/// `--features parallel` is the standing proof it matches the sequential snapshot.
#[cfg(feature = "parallel")]
#[allow(clippy::too_many_arguments)]
fn run_starts(
    items: &[Vec<[Scalar; 2]>],
    qty: &[usize],
    container: &[[Scalar; 2]],
    holes: &[Vec<[Scalar; 2]>],
    min_sep: Scalar,
    rotations_deg: &[Vec<Scalar>],
    seed: u64,
    budget: u64,
    starts: usize,
    areas: &[Scalar],
) -> Vec<(NestSolution, Scalar)> {
    std::thread::scope(|scope| {
        let handles: Vec<_> = (0..starts as u64)
            .map(|k| {
                scope.spawn(move || {
                    let sol = nest_per_item(
                        items,
                        qty,
                        container,
                        holes,
                        min_sep,
                        rotations_deg,
                        seed.wrapping_add(k),
                        budget,
                    );
                    let area = placed_area(&sol, areas);
                    (sol, area)
                })
            })
            .collect();
        // Join in spawn (k) order — the returned Vec is ordered by k regardless of finish order.
        handles
            .into_iter()
            .map(|h| h.join().expect("a multi-start nest thread panicked"))
            .collect()
    })
}

/// Nests `items` (demand `qty`) across several `sheets` in order: each sheet is filled with whatever
/// demand remains, then the rest spills to the next. Returns the placements per sheet plus the global
/// unplaced. Applies **one** allowed-rotation set to every item; for a distinct set per item type use
/// [`nest_multi_per_item`].
///
/// Determinism: each sheet is nested with a seed derived deterministically from `seed` and the sheet
/// index, so the whole result is byte-identical for the same arguments. `min_sep`, `rotations_deg`,
/// and `budget` carry the same meaning as in [`nest`].
#[must_use]
#[allow(clippy::too_many_arguments)]
pub fn nest_multi(
    items: &[Vec<[Scalar; 2]>],
    qty: &[usize],
    sheets: &[Sheet],
    min_sep: Scalar,
    rotations_deg: &[Scalar],
    seed: u64,
    budget: u64,
) -> MultiSheetSolution {
    // Broadcast the single rotation set to every item, then run the per-item path (byte-identical —
    // see [`nest`]).
    let rotations_per_item = vec![rotations_deg.to_vec(); items.len()];
    nest_multi_per_item(
        items,
        qty,
        sheets,
        min_sep,
        &rotations_per_item,
        seed,
        budget,
    )
}

/// Like [`nest_multi`], but with a **distinct allowed-rotation set per item type** — `rotations_deg[k]`
/// applies to `items[k]` (`rotations_deg.len() == items.len()`). Carries the per-item semantics of
/// [`nest_per_item`] across the multi-sheet spill. Same determinism contract as [`nest_multi`].
#[must_use]
#[allow(clippy::too_many_arguments)]
pub fn nest_multi_per_item(
    items: &[Vec<[Scalar; 2]>],
    qty: &[usize],
    sheets: &[Sheet],
    min_sep: Scalar,
    rotations_deg: &[Vec<Scalar>],
    seed: u64,
    budget: u64,
) -> MultiSheetSolution {
    assert_eq!(
        items.len(),
        qty.len(),
        "items and qty must be the same length"
    );
    assert_eq!(
        items.len(),
        rotations_deg.len(),
        "items and per-item rotations must be the same length"
    );

    let mut remaining = qty.to_vec();
    let mut per_sheet = Vec::with_capacity(sheets.len());

    for (i, sheet) in sheets.iter().enumerate() {
        // A distinct, deterministic per-sheet seed (SplitMix64 in the PRNG de-correlates adjacent
        // seeds, so `seed + i` gives well-separated streams).
        let sheet_seed = seed.wrapping_add(i as u64);
        let sol = nest_per_item(
            items,
            &remaining,
            &sheet.outline,
            &sheet.holes,
            min_sep,
            rotations_deg,
            sheet_seed,
            budget,
        );

        for p in &sol.placements {
            remaining[p.item] -= 1;
        }
        per_sheet.push(sol.placements);

        if remaining.iter().all(|&r| r == 0) {
            break; // everything placed — no need to touch the remaining sheets
        }
    }

    // Keep `per_sheet` index-parallel with `sheets`: any trailing sheets we broke out of (because
    // demand ran out) get an empty placement list, so `per_sheet[i]` is always valid for every sheet.
    per_sheet.resize_with(sheets.len(), Vec::new);

    let unplaced = remaining
        .iter()
        .enumerate()
        .flat_map(|(i, &n)| std::iter::repeat_n(i, n))
        .collect();

    MultiSheetSolution {
        per_sheet,
        unplaced,
    }
}

/// Greedily places every requested instance (largest-first) at its lowest-loss feasible pose.
/// `rotations_rad` is indexed by item id, so each type samples from its own orientation set.
#[allow(clippy::too_many_arguments)]
fn constructive_fill(
    layout: &mut Layout,
    entities: &[Option<Item>],
    order: &[usize],
    qty: &[usize],
    rotations_rad: &[Vec<Scalar>],
    prng: &mut Prng,
    budget: u64,
    placed_per_type: &mut [usize],
) {
    for &item_id in order {
        let item = entities[item_id].as_ref().unwrap();
        while placed_per_type[item_id] < qty[item_id] {
            match search::search(layout.cde(), item, &rotations_rad[item_id], prng, budget) {
                Some(d_transf) => {
                    improve::place_dropped(layout, item, d_transf);
                    placed_per_type[item_id] += 1;
                }
                None => break, // no remaining instance of this type fits anywhere
            }
        }
    }
}

/// Converts the layout's placed items into anchor-free [`Placement`]s.
fn extract_placements(layout: &Layout, entities: &[Option<Item>]) -> Vec<Placement> {
    layout
        .placed_items
        .values()
        .map(|pi| {
            let item = entities[pi.item_id].as_ref().unwrap();
            // The original outline's centroid = -(centering pre-transform translation).
            let (px, py) = item.shape_orig.pre_transform.translation();
            let centroid = (-px, -py);
            let (x, y, rotation_deg) = search::original_to_placed(&pi.d_transf, centroid);
            Placement {
                item: pi.item_id,
                x,
                y,
                rotation_deg,
            }
        })
        .collect()
}

/// Absolute polygon area (shoelace) of an item-local `outline`. Pure `+ − ×` ⇒ deterministic and
/// byte-identical cross-platform; used only to rank multi-start results by total placed area.
fn polygon_area(outline: &[[Scalar; 2]]) -> Scalar {
    let n = outline.len();
    if n < 3 {
        return 0.0;
    }
    let mut acc: Scalar = 0.0;
    for i in 0..n {
        let j = (i + 1) % n;
        acc += outline[i][0] * outline[j][1] - outline[j][0] * outline[i][1];
    }
    (0.5 * acc).abs()
}

/// Total placed area of a solution = Σ area(item) over its placements, summed in the solution's own
/// (deterministic) placement order. The multi-start objective — higher = denser packing / utilization.
fn placed_area(sol: &NestSolution, areas: &[Scalar]) -> Scalar {
    sol.placements.iter().map(|p| areas[p.item]).sum()
}

/// Every requested instance, marked unplaced (used when the container itself fails to import).
fn all_unplaced(qty: &[usize]) -> Vec<usize> {
    qty.iter()
        .enumerate()
        .flat_map(|(i, &n)| std::iter::repeat_n(i, n))
        .collect()
}

/// Re-export for callers that want to drive the per-item search directly.
pub use search::search;

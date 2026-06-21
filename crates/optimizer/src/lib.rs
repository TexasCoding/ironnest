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

use ironnest_cde::collision_detection::CDEConfig;
use ironnest_cde::entities::{Item, Layout};
use ironnest_cde::geometry::fail_fast::SPSurrogateConfig;
use ironnest_cde::io::ext_repr::{ExtContainer, ExtItem, ExtSPolygon, ExtShape};
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

/// Nests `items` (each with a demand `qty`) into a single irregular `container`.
///
/// * `items` — one outline per item *type*, in item-local coordinates.
/// * `qty` — demand per item type (`qty.len() == items.len()`).
/// * `container` — the container boundary outline (holes/keepouts: a later milestone).
/// * `min_sep` — minimum separation between any two parts (and part↔boundary); `0.0` to disable.
/// * `rotations_deg` — allowed discrete orientations in degrees (e.g. `[0.0, 90.0, 180.0, 270.0]`);
///   empty ⇒ no rotation.
/// * `seed` — explicit PRNG seed (no implicit/entropy fallback, ever).
/// * `budget` — samples per item placement (fixed; never a wall clock).
///
/// Determinism: the same arguments always produce a byte-identical [`NestSolution`].
#[must_use]
pub fn nest(
    items: &[Vec<[Scalar; 2]>],
    qty: &[usize],
    container: &[[Scalar; 2]],
    min_sep: Scalar,
    rotations_deg: &[Scalar],
    seed: u64,
    budget: u64,
) -> NestSolution {
    assert_eq!(
        items.len(),
        qty.len(),
        "items and qty must be the same length"
    );

    let cde_config = default_cde_config();
    let min_item_separation = (min_sep > 0.0).then_some(min_sep);
    let importer = Importer::new(cde_config, None, min_item_separation, None);

    // Rotations: degrees for the importer's metadata, radians for our sampler. Empty ⇒ {0}.
    let rotations_deg_vec: Vec<Scalar> = if rotations_deg.is_empty() {
        vec![0.0]
    } else {
        rotations_deg.to_vec()
    };
    let rotations_rad: Vec<Scalar> = rotations_deg_vec.iter().map(|d| d.to_radians()).collect();

    // Import the container; a malformed container ⇒ nothing can be placed.
    let ext_container = ExtContainer {
        id: 0,
        shape: ExtShape::SimplePolygon(ext_spolygon(container)),
        zones: vec![],
    };
    let Ok(container_entity) = importer.import_container(&ext_container) else {
        return NestSolution {
            placements: vec![],
            unplaced: all_unplaced(qty),
        };
    };

    // Import items. A malformed item ⇒ that whole type is unplaced.
    let mut entities: Vec<Option<Item>> = Vec::with_capacity(items.len());
    for (i, outline) in items.iter().enumerate() {
        let ext_item = ExtItem {
            id: i as u64,
            allowed_orientations: Some(rotations_deg_vec.clone()),
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
        &rotations_rad,
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
        &rotations_rad,
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

/// Greedily places every requested instance (largest-first) at its lowest-loss feasible pose.
#[allow(clippy::too_many_arguments)]
fn constructive_fill(
    layout: &mut Layout,
    entities: &[Option<Item>],
    order: &[usize],
    qty: &[usize],
    rotations_rad: &[Scalar],
    prng: &mut Prng,
    budget: u64,
    placed_per_type: &mut [usize],
) {
    for &item_id in order {
        let item = entities[item_id].as_ref().unwrap();
        while placed_per_type[item_id] < qty[item_id] {
            match search::search(layout.cde(), item, rotations_rad, prng, budget) {
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

/// Every requested instance, marked unplaced (used when the container itself fails to import).
fn all_unplaced(qty: &[usize]) -> Vec<usize> {
    qty.iter()
        .enumerate()
        .flat_map(|(i, &n)| std::iter::repeat_n(i, n))
        .collect()
}

/// Re-export for callers that want to drive the per-item search directly.
pub use search::search;

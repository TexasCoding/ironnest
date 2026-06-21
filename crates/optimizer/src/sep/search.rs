// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Per-move placement search: sample candidate poses uniformly, then refine the best few with an
//! adaptive **coordinate descent**. Ported from sparrow (`src/sample/{uniform_sampler,best_samples,
//! coord_descent,search}.rs`, MIT) to our deterministic loop.
//!
//! ironnest adaptations (CLAUDE.md determinism rules):
//! - all randomness is the portable [`Prng`] (PCG64) — rotation pick, x/y draw, descent axis pick;
//! - **no Wiggle axis** — rotations stay discrete, so the descent only moves in x/y (the four
//!   translation axes) and never touches continuous trig;
//! - candidate ties break deterministically ([`SampleEval`]'s total order; first-min on equal eval).

use super::evaluator::{SampleEval, SeparationEvaluator, unweighted_overlap};
use crate::prng::Prng;
use ironnest_cde::entities::{Item, Layout};
use ironnest_cde::geometry::geo_enums::RotationRange;
use ironnest_cde::geometry::geo_traits::TransformableFrom;
use ironnest_cde::geometry::primitives::Rect;
use ironnest_geo::{DTransformation, Scalar, Transformation};

/// Coordinate-descent step multiplier on improvement. sparrow `CD_STEP_SUCCESS`.
const CD_STEP_SUCCESS: Scalar = 1.1;
/// Coordinate-descent step multiplier on failure. sparrow `CD_STEP_FAIL`.
const CD_STEP_FAIL: Scalar = 0.5;
/// (init, limit) translation step as a ratio of the item's min dimension — first refinement.
const PRE_REFINE_TL_RATIOS: (Scalar, Scalar) = (0.25, 0.02);
/// (init, limit) translation step as a ratio of the item's min dimension — final refinement.
const SND_REFINE_TL_RATIOS: (Scalar, Scalar) = (0.01, 0.001);
/// Samples closer than this ratio of the item's min dimension are treated as duplicates. sparrow
/// `UNIQUE_SAMPLE_THRESHOLD`.
const UNIQUE_SAMPLE_THRESHOLD: Scalar = 0.05;

/// How many poses to draw and how many to refine, per move. sparrow `SampleConfig`.
#[derive(Clone, Copy, Debug)]
#[allow(clippy::struct_field_names)] // the `n_` prefix reads as "count of"; matches sparrow
pub struct SampleConfig {
    pub n_container_samples: usize,
    pub n_focussed_samples: usize,
    pub n_coord_descents: usize,
}

/// Searches for a low-overlap pose for `item`, starting from `ref_dt` (its pre-move pose) and a
/// focused window `focus_bbox` around it, plus container-wide sampling. Returns the best
/// `(pose, eval)` found, or `None` if no pose could be sampled at all.
#[allow(clippy::too_many_arguments)]
pub fn search_placement(
    item: &Item,
    container_bbox: Rect,
    ref_dt: DTransformation,
    focus_bbox: Rect,
    evaluator: &mut SeparationEvaluator,
    cfg: SampleConfig,
    prng: &mut Prng,
) -> Option<(DTransformation, SampleEval)> {
    let item_min_dim = Scalar::min(item.shape_cd.bbox.width(), item.shape_cd.bbox.height());
    let mut best = BestSamples::new(cfg.n_coord_descents, item_min_dim * UNIQUE_SAMPLE_THRESHOLD);

    // Always evaluate the current pose — separation must never make an item worse than staying put.
    let ref_eval = evaluator.evaluate_sample(ref_dt);
    best.report(ref_dt, ref_eval);

    // Focused sampling around the item's pre-move footprint.
    if let Some(focused) = UniformBBoxSampler::new(focus_bbox, item, container_bbox) {
        for _ in 0..cfg.n_focussed_samples {
            let dt = focused.sample(prng);
            let eval = evaluator.evaluate_sample(dt);
            best.report(dt, eval);
        }
    }

    // Container-wide sampling.
    if let Some(container) = UniformBBoxSampler::new(container_bbox, item, container_bbox) {
        for _ in 0..cfg.n_container_samples {
            let dt = container.sample(prng);
            let eval = evaluator.evaluate_sample(dt);
            best.report(dt, eval);
        }
    }

    // 1. Refine each retained best sample with a coarse coordinate descent.
    let pre_cfg = CDConfig {
        t_step_init: item_min_dim * PRE_REFINE_TL_RATIOS.0,
        t_step_limit: item_min_dim * PRE_REFINE_TL_RATIOS.1,
    };
    for start in best.samples.clone() {
        let descended = refine_coord_desc(start, evaluator, pre_cfg, prng);
        best.report(descended.0, descended.1);
    }

    // 2. Refine the single best with a finer coordinate descent.
    let snd_cfg = CDConfig {
        t_step_init: item_min_dim * SND_REFINE_TL_RATIOS.0,
        t_step_limit: item_min_dim * SND_REFINE_TL_RATIOS.1,
    };
    best.best()
        .map(|s| refine_coord_desc(s, evaluator, snd_cfg, prng))
}

// ---------------------------------------------------------------------------------------------
// Uniform bbox sampler
// ---------------------------------------------------------------------------------------------

/// Draws uniform poses (rotation + x + y) that keep the item fully inside the container bbox.
#[derive(Clone, Debug)]
struct UniformBBoxSampler {
    rot_entries: Vec<RotEntry>,
}

#[derive(Clone, Debug)]
struct RotEntry {
    r: Scalar,
    x_range: (Scalar, Scalar),
    y_range: (Scalar, Scalar),
}

impl UniformBBoxSampler {
    /// Builds a sampler over `sample_bbox`, restricted so the (rotated) item stays inside
    /// `container_bbox`. Returns `None` if no rotation admits any in-bounds position.
    fn new(sample_bbox: Rect, item: &Item, container_bbox: Rect) -> Option<Self> {
        let rotations: Vec<Scalar> = match &item.allowed_rotation {
            // `Continuous` is intentionally collapsed to `{0°}`: the separation search is a
            // discrete-rotation path (no Wiggle axis), so continuous rotation is not supported here.
            // The public `nest()` always imports a `Discrete` set, so this arm is unreachable today.
            RotationRange::None | RotationRange::Continuous => vec![0.0],
            RotationRange::Discrete(r) => r.clone(),
        };

        let mut shape_buffer = (*item.shape_cd).clone();

        let rot_entries: Vec<RotEntry> = rotations
            .iter()
            .filter_map(|&r| {
                let r_bbox = shape_buffer
                    .transform_from(&item.shape_cd, &Transformation::from_rotation(r))
                    .bbox;

                // The container range that keeps the rotated shape fully inside.
                let cont_x = (
                    container_bbox.x_min - r_bbox.x_min,
                    container_bbox.x_max - r_bbox.x_max,
                );
                let cont_y = (
                    container_bbox.y_min - r_bbox.y_min,
                    container_bbox.y_max - r_bbox.y_max,
                );

                let x_range = intersect_range(cont_x, (sample_bbox.x_min, sample_bbox.x_max));
                let y_range = intersect_range(cont_y, (sample_bbox.y_min, sample_bbox.y_max));

                if x_range.0 >= x_range.1 || y_range.0 >= y_range.1 {
                    None
                } else {
                    Some(RotEntry {
                        r,
                        x_range,
                        y_range,
                    })
                }
            })
            .collect();

        if rot_entries.is_empty() {
            None
        } else {
            Some(Self { rot_entries })
        }
    }

    /// Draws one pose: pick a rotation entry, then x then y (fixed draw order — part of the contract).
    fn sample(&self, prng: &mut Prng) -> DTransformation {
        let entry = &self.rot_entries[prng.below(self.rot_entries.len())];
        let x = prng.range(entry.x_range.0, entry.x_range.1);
        let y = prng.range(entry.y_range.0, entry.y_range.1);
        DTransformation::new(entry.r, (x, y))
    }
}

fn intersect_range(a: (Scalar, Scalar), b: (Scalar, Scalar)) -> (Scalar, Scalar) {
    (Scalar::max(a.0, b.0), Scalar::min(a.1, b.1))
}

/// Finds the lowest-(unweighted-)overlap pose for a **not-yet-placed** `item` by sampling
/// `n_samples` in-bounds poses over the whole container. Used by the insertion driver to seed a new
/// part (it will usually overlap; the separator then makes room). Returns `None` only if no in-bounds
/// pose exists. Ties keep the first pose found; a zero-overlap (feasible) pose ends the search early.
pub fn lowest_overlap_pose(
    layout: &Layout,
    item: &Item,
    prng: &mut Prng,
    n_samples: usize,
) -> Option<DTransformation> {
    let container_bbox = layout.container.outer_cd.bbox;
    let sampler = UniformBBoxSampler::new(container_bbox, item, container_bbox)?;
    let mut buff = (*item.shape_cd).clone();

    let mut best: Option<(DTransformation, Scalar)> = None;
    for _ in 0..n_samples {
        let dt = sampler.sample(prng);
        buff.transform_from(&item.shape_cd, &dt.compose());
        let loss = unweighted_overlap(layout, &buff);
        if best.as_ref().is_none_or(|(_, b)| loss < *b) {
            best = Some((dt, loss));
        }
        if loss == 0.0 {
            break; // a feasible seed — cannot beat zero overlap
        }
    }
    best.map(|(dt, _)| dt)
}

// ---------------------------------------------------------------------------------------------
// Best-samples set
// ---------------------------------------------------------------------------------------------

/// Keeps the `size` best (lowest-eval) samples seen, deduplicated by spatial similarity, sorted best
/// first. Ported from sparrow `BestSamples`.
#[derive(Debug, Clone)]
struct BestSamples {
    size: usize,
    samples: Vec<(DTransformation, SampleEval)>,
    unique_thresh: Scalar,
}

impl BestSamples {
    fn new(size: usize, unique_thresh: Scalar) -> Self {
        Self {
            size: size.max(1),
            samples: vec![],
            unique_thresh,
        }
    }

    fn report(&mut self, dt: DTransformation, eval: SampleEval) -> bool {
        let accept = if eval < self.upper_bound() {
            let similar_idx: Vec<usize> = self
                .samples
                .iter()
                .enumerate()
                .filter(|(_, (d, _))| dtransfs_are_similar(*d, dt, self.unique_thresh))
                .map(|(i, _)| i)
                .collect();

            if similar_idx.is_empty() {
                if self.samples.len() == self.size {
                    self.samples.pop();
                }
                true
            } else if similar_idx.iter().all(|&i| eval < self.samples[i].1) {
                // Strictly better than every similar incumbent: evict them.
                self.samples
                    .retain(|(d, _)| !dtransfs_are_similar(*d, dt, self.unique_thresh));
                true
            } else {
                false
            }
        } else {
            false
        };

        if accept {
            self.samples.push((dt, eval));
            // Stable sort keeps insertion order on equal evals → deterministic.
            self.samples.sort_by_key(|&(_, eval)| eval);
        }
        accept
    }

    fn best(&self) -> Option<(DTransformation, SampleEval)> {
        self.samples.first().copied()
    }

    /// The acceptance ceiling: the worst retained sample once full, else [`SampleEval::Invalid`]
    /// (so any non-invalid sample is accepted until the set fills).
    fn upper_bound(&self) -> SampleEval {
        self.samples
            .get(self.size - 1)
            .map_or(SampleEval::Invalid, |(_, e)| *e)
    }
}

/// Two poses are "similar" if their translations are within `thresh` on both axes and their
/// rotations within ~1°. Ported from sparrow `dtransfs_are_similar`.
///
/// Unlike sparrow we do *not* reduce the rotations mod 2π before comparing: our rotations are the
/// exact discrete literals from the allowed set (0, π/2, π, 3π/2), never aliased copies 2π apart, so
/// raw subtraction is correct. (If continuous rotation were ever added, restore the mod-2π reduction.)
fn dtransfs_are_similar(dt1: DTransformation, dt2: DTransformation, thresh: Scalar) -> bool {
    let (x1, y1) = dt1.translation();
    let (x2, y2) = dt2.translation();
    if (x1 - x2).abs() < thresh && (y1 - y2).abs() < thresh {
        let angle_diff = (dt1.rotation() - dt2.rotation()).abs();
        angle_diff < (1.0 as Scalar).to_radians()
    } else {
        false
    }
}

// ---------------------------------------------------------------------------------------------
// Coordinate descent (no Wiggle: x/y axes only)
// ---------------------------------------------------------------------------------------------

#[derive(Clone, Copy, Debug)]
struct CDConfig {
    t_step_init: Scalar,
    t_step_limit: Scalar,
}

/// Refines `(init_dt, init_eval)` to a local overlap minimum by adaptive coordinate descent: try two
/// candidates either side of the current pose along the active axis, keep the better, grow the step
/// on success / shrink and re-pick the axis on failure; stop when the steps fall below the limit.
fn refine_coord_desc(
    (init_dt, init_eval): (DTransformation, SampleEval),
    evaluator: &mut SeparationEvaluator,
    cfg: CDConfig,
    prng: &mut Prng,
) -> (DTransformation, SampleEval) {
    let mut cd = CoordinateDescent {
        pos: init_dt,
        eval: init_eval,
        axis: CDAxis::pick(prng),
        steps: (cfg.t_step_init, cfg.t_step_init),
        limit: cfg.t_step_limit,
    };

    // Convergence is guaranteed in principle (every non-improving step halves a coordinate's step,
    // so the steps fall below the limit in finitely many moves), but a fixed hard cap is cheap
    // insurance against a pathological landscape and matches sparrow's runaway debug guard. The cap
    // is a fixed integer (never a clock) — determinism-safe.
    let mut iters = 0u32;
    while let Some(candidates) = cd.ask() {
        let evals = candidates.map(|c| evaluator.evaluate_sample(c));
        // First-min on ties → deterministic.
        let best = if evals[0] <= evals[1] {
            (candidates[0], evals[0])
        } else {
            (candidates[1], evals[1])
        };
        cd.tell(best, prng);

        iters += 1;
        debug_assert!(
            iters < CD_MAX_ITERS,
            "coordinate descent exceeded its iteration cap"
        );
        if iters >= CD_MAX_ITERS {
            break;
        }
    }
    (cd.pos, cd.eval)
}

/// Hard cap on coordinate-descent iterations per refinement (a runaway backstop; convergence
/// normally happens in far fewer). Fixed integer — never a wall clock.
const CD_MAX_ITERS: u32 = 500;

#[derive(Debug)]
struct CoordinateDescent {
    pos: DTransformation,
    eval: SampleEval,
    axis: CDAxis,
    steps: (Scalar, Scalar),
    limit: Scalar,
}

impl CoordinateDescent {
    /// Two candidate poses either side of the current one along the active axis, or `None` once both
    /// steps have shrunk below the limit.
    fn ask(&self) -> Option<[DTransformation; 2]> {
        let (sx, sy) = self.steps;
        if sx < self.limit && sy < self.limit {
            return None;
        }
        let (tx, ty) = self.pos.translation();
        let r = self.pos.rotation();
        let pair = match self.axis {
            CDAxis::Horizontal => [(tx + sx, ty), (tx - sx, ty)],
            CDAxis::Vertical => [(tx, ty + sy), (tx, ty - sy)],
            CDAxis::ForwardDiag => [(tx + sx, ty + sy), (tx - sx, ty - sy)],
            CDAxis::BackwardDiag => [(tx - sx, ty + sy), (tx + sx, ty - sy)],
        };
        Some(pair.map(|(x, y)| DTransformation::new(r, (x, y))))
    }

    fn tell(&mut self, (pos, eval): (DTransformation, SampleEval), prng: &mut Prng) {
        let better = eval < self.eval;
        let worse = eval > self.eval;
        if !worse {
            self.pos = pos;
            self.eval = eval;
        }
        let m = if better {
            CD_STEP_SUCCESS
        } else {
            CD_STEP_FAIL
        };
        match self.axis {
            CDAxis::Horizontal => self.steps.0 *= m,
            CDAxis::Vertical => self.steps.1 *= m,
            CDAxis::ForwardDiag | CDAxis::BackwardDiag => {
                let ms = m.sqrt();
                self.steps.0 *= ms;
                self.steps.1 *= ms;
            }
        }
        if !better {
            self.axis = CDAxis::pick(prng);
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum CDAxis {
    Horizontal,
    Vertical,
    ForwardDiag,
    BackwardDiag,
}

impl CDAxis {
    fn pick(prng: &mut Prng) -> Self {
        match prng.below(4) {
            0 => CDAxis::Horizontal,
            1 => CDAxis::Vertical,
            2 => CDAxis::ForwardDiag,
            _ => CDAxis::BackwardDiag,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dt(x: Scalar, y: Scalar) -> DTransformation {
        DTransformation::new(0.0, (x, y))
    }
    fn coll(loss: Scalar) -> SampleEval {
        SampleEval::Collision { loss }
    }

    #[test]
    fn best_samples_keeps_size_best_and_sorted() {
        let mut bs = BestSamples::new(2, 0.001);
        // Three well-separated poses; only the two lowest-loss survive, sorted best-first.
        assert!(bs.report(dt(0.0, 0.0), coll(5.0)));
        assert!(bs.report(dt(10.0, 0.0), coll(1.0)));
        assert!(bs.report(dt(20.0, 0.0), coll(3.0)));
        assert_eq!(bs.samples.len(), 2);
        assert_eq!(bs.best().map(|(_, e)| e), Some(coll(1.0)));
        // A pose worse than both retained (and above the upper bound) is rejected.
        assert!(!bs.report(dt(30.0, 0.0), coll(9.0)));
    }

    #[test]
    fn best_samples_dedups_similar_keeping_the_better() {
        let mut bs = BestSamples::new(3, 1.0); // similarity threshold 1.0 unit
        assert!(bs.report(dt(0.0, 0.0), coll(5.0)));
        // Within threshold of the first → replaces it only because it is strictly better.
        assert!(bs.report(dt(0.5, 0.0), coll(2.0)));
        assert_eq!(
            bs.samples.len(),
            1,
            "the similar worse one was evicted, not added"
        );
        assert_eq!(bs.best().map(|(_, e)| e), Some(coll(2.0)));
        // A similar but worse pose is not accepted.
        assert!(!bs.report(dt(0.4, 0.0), coll(8.0)));
        assert_eq!(bs.samples.len(), 1);
    }

    #[test]
    fn dtransfs_similarity_respects_threshold() {
        assert!(dtransfs_are_similar(dt(0.0, 0.0), dt(0.05, 0.05), 0.1));
        assert!(!dtransfs_are_similar(dt(0.0, 0.0), dt(0.2, 0.0), 0.1));
    }
}

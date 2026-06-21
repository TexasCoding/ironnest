// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! The overlap *proxy* — a smooth, differentiable-everywhere stand-in for the true overlap area
//! between two shapes, computed from their inscribed surrogate poles (circles).
//!
//! Ported from sparrow (`src/quantify/overlap_proxy.rs` + `src/quantify/mod.rs`, MIT) to [`Scalar`]
//! = f64. It is **Algorithm 3/4** of the sparrow paper (arXiv 2509.13329).
//!
//! DETERMINISM(ironnest): the proxy is the search's *ranking* signal only — the CDE remains the
//! feasibility arbiter (see [`super::evaluator`]). Arithmetic is `+ − × ÷ sqrt powi` exclusively
//! (all IEEE-deterministic); no transcendentals. The pole *set* is byte-identical cross-platform
//! because it comes from the fork's ordered [`SPSurrogate`] generation. For a fixed nested-loop
//! order the pair sum is byte-identical on a given platform; pairs are always evaluated in the same
//! `(outer, inner)` order so the result is reproducible.

use ironnest_cde::geometry::fail_fast::SPSurrogate;
use ironnest_cde::geometry::geo_traits::DistanceTo;
use ironnest_cde::geometry::primitives::{Rect, SPolygon};
use ironnest_geo::Scalar;
use std::f64::consts::PI;

/// `epsilon = max(diam_a, diam_b) * OVERLAP_PROXY_EPSILON_DIAM_RATIO` — the decay knee, scaled to
/// the parts' size so the proxy is smooth at the resolution that matters. sparrow `consts.rs`.
const OVERLAP_PROXY_EPSILON_DIAM_RATIO: Scalar = 0.01;

/// A proxy for the overlap *area* between two simple polygons, computed from their surrogate poles.
///
/// For every pole pair `(p, q)` across the two shapes, the circle–circle penetration depth
/// `pd = (r_p + r_q) − dist(c_p, c_q)` is passed through a **hyperbolic decay** so the signal stays
/// strictly positive and smooth even once the circles separate (`pd < epsilon`) — this is the fix
/// for the "motion dies right before resolving" failure of a hard `max(0, pd)` cutoff. Contributions
/// are weighted by `min(r_p, r_q)` and the whole sum is scaled by `π`.
///
/// sparrow `overlap_proxy::overlap_area_proxy` (Algorithm 3).
#[must_use]
pub fn overlap_area_proxy(sp1: &SPSurrogate, sp2: &SPSurrogate, epsilon: Scalar) -> Scalar {
    let mut total_overlap: Scalar = 0.0;
    for p1 in &sp1.poles {
        for p2 in &sp2.poles {
            // Penetration depth between the two poles (circles).
            let pd = (p1.radius + p2.radius) - p1.center.distance_to(&p2.center);

            let pd_decay = if pd >= epsilon {
                pd
            } else {
                // Smooth, strictly-positive continuation for pd < epsilon (incl. negative pd):
                // -pd + 2ε > 0 always here, so this never divides by zero and never goes negative.
                epsilon.powi(2) / (-pd + 2.0 * epsilon)
            };

            total_overlap += pd_decay * Scalar::min(p1.radius, p2.radius);
        }
    }
    total_overlap *= PI;
    debug_assert!(total_overlap.is_finite() && total_overlap >= 0.0);

    total_overlap
}

/// The shape-difficulty penalty between two shapes: the geometric mean of the square roots of their
/// convex-hull areas, so big / concave parts cost more to leave overlapping. sparrow
/// `quantify::calc_shape_penalty`.
#[must_use]
pub fn calc_shape_penalty(s1: &SPolygon, s2: &SPolygon) -> Scalar {
    let p1 = Scalar::sqrt(s1.surrogate().convex_hull_area);
    let p2 = Scalar::sqrt(s2.surrogate().convex_hull_area);
    (p1 * p2).sqrt()
}

/// Quantifies a collision between two simple polygons (the per-pair loss stored in the tracker and
/// summed by the evaluator). sparrow `quantify::quantify_collision_poly_poly` (Algorithm 4).
///
/// Symmetric in its arguments up to the floating-point summation order of [`overlap_area_proxy`]
/// (`epsilon` and `penalty` are symmetric); callers always pass the pair in a fixed order so the
/// stored value is reproducible.
#[must_use]
pub fn quantify_collision_poly_poly(s1: &SPolygon, s2: &SPolygon) -> Scalar {
    let epsilon = Scalar::max(s1.diameter, s2.diameter) * OVERLAP_PROXY_EPSILON_DIAM_RATIO;

    let overlap_proxy =
        overlap_area_proxy(s1.surrogate(), s2.surrogate(), epsilon) + epsilon.powi(2);
    debug_assert!(overlap_proxy.is_finite() && overlap_proxy > 0.0);

    let penalty = calc_shape_penalty(s1, s2);

    overlap_proxy.sqrt() * penalty
}

/// Quantifies a collision between a simple polygon and the *exterior* of the container, approximated
/// by the container bounding box. sparrow `quantify::quantify_collision_poly_container`.
///
/// The bbox is an approximation for an irregular container (it under-penalizes poking out through a
/// concave boundary) — acceptable because this is only the ranking signal; the CDE's `Exterior`
/// hazard is the real arbiter.
#[must_use]
pub fn quantify_collision_poly_container(s: &SPolygon, c_bbox: Rect) -> Scalar {
    let s_bbox = s.bbox;
    let overlap = match Rect::intersection(s_bbox, c_bbox) {
        Some(r) => {
            // Intersection exists: the area sticking out, plus a small floor so it is never zero.
            (s_bbox.area() - r.area()) + 0.0001 * s_bbox.area()
        }
        None => {
            // No intersection: guide the shape back toward the container.
            s_bbox.area() + s_bbox.centroid().distance_to(&c_bbox.centroid())
        }
    };
    debug_assert!(overlap.is_finite() && overlap >= 0.0);

    let penalty = calc_shape_penalty(s, s);

    2.0 * overlap.sqrt() * penalty
}

#[cfg(test)]
mod tests {
    use super::*;
    use ironnest_cde::geometry::fail_fast::SPSurrogateConfig;
    use ironnest_cde::geometry::primitives::{Point, SPolygon};
    use ironnest_geo::Transformation;
    use ironnest_geo::geo_traits::Transformable;

    fn square(cx: Scalar, cy: Scalar, half: Scalar) -> SPolygon {
        let mut sp = SPolygon::new(vec![
            Point(cx - half, cy - half),
            Point(cx + half, cy - half),
            Point(cx + half, cy + half),
            Point(cx - half, cy + half),
        ])
        .unwrap();
        sp.generate_surrogate(SPSurrogateConfig {
            n_pole_limits: [(100, 0.0), (20, 0.75), (10, 0.90)],
            n_ff_poles: 2,
            n_ff_piers: 0,
        })
        .unwrap();
        sp
    }

    #[test]
    fn proxy_is_monotone_decreasing_with_separation() {
        // Two identical unit squares; slide the second away along +x. As they separate the proxy
        // must decrease monotonically and stay strictly positive (the hyperbolic-decay guarantee).
        let a = square(0.0, 0.0, 1.0);
        let mut prev = Scalar::INFINITY;
        for step in 0..40 {
            let dx = Scalar::from(step) * 0.1;
            let mut b = a.clone();
            b.transform(&Transformation::from_translation((dx, 0.0)));
            let eps = Scalar::max(a.diameter, b.diameter) * OVERLAP_PROXY_EPSILON_DIAM_RATIO;
            let v = overlap_area_proxy(a.surrogate(), b.surrogate(), eps);
            assert!(
                v > 0.0,
                "proxy must be strictly positive everywhere (dx={dx})"
            );
            assert!(
                v <= prev + 1e-12,
                "proxy must not increase as parts separate (dx={dx})"
            );
            prev = v;
        }
    }

    #[test]
    fn proxy_is_symmetric_for_fixed_order() {
        let a = square(0.0, 0.0, 1.0);
        let b = square(0.5, 0.3, 1.5);
        // quantify_collision_poly_poly(a,b) vs (b,a): epsilon & penalty are symmetric; only the
        // pole-loop order differs. They should agree to within floating-point summation slack.
        let ab = quantify_collision_poly_poly(&a, &b);
        let ba = quantify_collision_poly_poly(&b, &a);
        assert!((ab - ba).abs() <= 1e-9 * ab.max(ba), "ab={ab} ba={ba}");
    }

    #[test]
    #[allow(clippy::float_cmp)] // byte-identity IS the property under test (determinism)
    fn identical_calls_are_byte_identical() {
        let a = square(0.0, 0.0, 1.0);
        let b = square(0.5, 0.5, 1.0);
        assert_eq!(
            quantify_collision_poly_poly(&a, &b),
            quantify_collision_poly_poly(&a, &b)
        );
    }
}

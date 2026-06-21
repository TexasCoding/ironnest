// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use crate::Scalar;
use crate::primitives::Point;
use crate::primitives::SPolygon;
use ordered_float::OrderedFloat;

use anyhow::{Result, bail};

/// Returns the indices of the points in the [`SPolygon`] that form the convex hull
#[must_use]
pub fn convex_hull_indices(shape: &SPolygon) -> Vec<usize> {
    let c_hull = convex_hull_from_points(shape.vertices.clone());
    let mut indices = vec![];
    for p in &c_hull {
        indices.push(shape.vertices.iter().position(|x| x == p).unwrap());
    }
    indices
}

/// Reconstitutes the convex hull of a [`SPolygon`] using its surrogate
pub fn convex_hull_from_surrogate(s: &SPolygon) -> Result<Vec<Point>> {
    if let Some(surr) = s.surrogate.as_ref() {
        Ok(surr
            .convex_hull_indices
            .iter()
            .map(|&i| s.vertices[i])
            .collect())
    } else {
        bail!("no surrogate present")
    }
}

/// Filters a set of points to only include those that are part of the convex hull
#[must_use]
pub fn convex_hull_from_points(mut points: Vec<Point>) -> Vec<Point> {
    //https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain

    //sort the points by x coordinate.
    // DETERMINISM(ironnest): this MUST stay a *stable* sort (`sort_by_key`, not `sort_unstable_*`).
    // Points sharing an x (common in CAD parts with vertical edges) keep their input order, which —
    // together with deterministic import vertex order — makes the monotone-chain hull (and the
    // surrogate poles/piers it feeds into the CDE) byte-identical across platforms.
    points.sort_by_key(|p| OrderedFloat(p.0));

    let mut lower_hull = points
        .iter()
        .fold(vec![], |hull, p| grow_convex_hull(hull, *p));
    let mut upper_hull = points
        .iter()
        .rev()
        .fold(vec![], |hull, p| grow_convex_hull(hull, *p));

    //First and last element of both hull parts are the same point
    upper_hull.pop();
    lower_hull.pop();

    lower_hull.append(&mut upper_hull);
    lower_hull
}

fn grow_convex_hull(mut h: Vec<Point>, next: Point) -> Vec<Point> {
    //pop all points from the hull which will be made irrelevant due to the new point
    while h.len() >= 2 && cross(h[h.len() - 2], h[h.len() - 1], next) <= 0.0 {
        h.pop();
    }
    h.push(next);
    h
}

fn cross(a: Point, b: Point, c: Point) -> Scalar {
    (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
}

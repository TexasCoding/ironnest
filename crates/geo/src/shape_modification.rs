// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use crate::Scalar;
use itertools::Itertools;
use log::{debug, error, info, warn};
use ordered_float::OrderedFloat;
use serde::{Deserialize, Serialize};
use std::cmp::Ordering;

use crate::fpa::FPA;
use crate::geo_traits::{CollidesWith, DistanceTo};
use crate::primitives::Edge;
use crate::primitives::Point;
use crate::primitives::SPolygon;

use anyhow::{Result, bail};

/// Whether to strictly inflate or deflate when making any modifications to shape.
/// Depends on the [`position`](crate::collision_detection::hazards::HazardEntity::scope) of the [`HazardEntity`](crate::collision_detection::hazards::HazardEntity) that the shape represents.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ShapeModifyMode {
    /// Modify the shape to be strictly larger than the original (superset).
    Inflate,
    /// Modify the shape to be strictly smaller than the original (subset).
    Deflate,
}

#[derive(Serialize, Deserialize, Clone, Copy, Debug, Default, PartialEq)]
pub struct ShapeModifyConfig {
    /// Maximum deviation of the simplified polygon with respect to the original polygon area as a ratio.
    /// If undefined, no simplification is performed.
    /// See [`simplify_shape`]
    pub simplify_tolerance: Option<Scalar>,
    /// Offset by which to inflate or deflate the polygon.
    /// If undefined, no offset is applied.
    /// See [`offset_shape`]
    pub offset: Option<Scalar>,
    /// FORK(ironnest): Douglas–Peucker tolerance for decimating a **collision footprint** (curved,
    /// high-vertex parts). When `Some(tol)`, [`OriginalShape::convert_to_internal`](crate::original_shape::OriginalShape::convert_to_internal)
    /// takes a dedicated path — DP-simplify by `tol` FIRST, then offset by `offset + tol` (instead of
    /// the default offset-then-area-simplify) — so the footprint stays a superset of
    /// `original ⊕ disk(offset)` while shedding most vertices. Set per-item by the optimizer; `None`
    /// (the default) leaves the upstream pipeline untouched. See [`simplify_dp`].
    #[serde(default)]
    pub collision_decimation: Option<Scalar>,
    /// Definition for narrow concavities that can be closed by a straight edge.
    /// Defined as a tuple of (`max_distance_ratio`, `max_area_ratio`) where:
    /// - `max_distance_ratio`: maximum distance between two vertices of a polygon to consider it a narrow concavity, defined as a fraction of the item's diameter.
    /// - `max_area_ratio`: maximum area of the sub-shape formed by the vertices between the two vertices, defined as a fraction of the item's area.
    ///
    /// If undefined, no narrow concavities will be closed.
    /// See [`close_narrow_concavities`]
    pub narrow_concavity_cutoff: Option<(Scalar, Scalar)>,
}

/// Simplifies a [`SPolygon`] by reducing the number of edges.
///
/// The simplified shape will either be a subset or a superset of the original shape, depending on the [`ShapeModifyMode`].
/// The procedure sequentially eliminates edges until either the change in area (ratio)
/// exceeds `max_area_delta` or the number of edges < 4.
pub fn simplify_shape(
    shape: &SPolygon,
    mode: ShapeModifyMode,
    max_area_change_ratio: Scalar,
) -> SPolygon {
    let original_area = shape.area;

    let mut ref_points = shape.vertices.clone();

    for _ in 0..shape.n_vertices() {
        let n_points = ref_points.len().cast_signed();
        if n_points < 4 {
            //can't simplify further
            break;
        }

        let mut corners = (0..n_points)
            .map(|i| {
                let i_prev = (i - 1).rem_euclid(n_points);
                let i_next = (i + 1).rem_euclid(n_points);
                Corner(
                    i_prev.cast_unsigned(),
                    i.cast_unsigned(),
                    i_next.cast_unsigned(),
                )
            })
            .collect_vec();

        if mode == ShapeModifyMode::Deflate {
            //default mode is to inflate, so we need to reverse the order of the corners and flip the corners for deflate mode
            //reverse the order of the corners
            corners.reverse();
            //reverse each corner
            corners.iter_mut().for_each(Corner::flip);
        }

        let mut candidates = vec![];

        let mut prev_corner = corners.last().expect("corners is empty");
        let mut prev_corner_type = CornerType::from(prev_corner.to_points(&ref_points));

        //Go over all corners and generate candidates
        for corner in &corners {
            let corner_type = CornerType::from(corner.to_points(&ref_points));

            //Generate a removal candidate (or not)
            match (&corner_type, &prev_corner_type) {
                (CornerType::Concave, _) => candidates.push(Candidate::Concave(*corner)),
                (CornerType::Collinear, _) => candidates.push(Candidate::Collinear(*corner)),
                (CornerType::Convex, CornerType::Convex) => {
                    candidates.push(Candidate::ConvexConvex(*prev_corner, *corner));
                }
                (_, _) => {}
            }
            (prev_corner, prev_corner_type) = (corner, corner_type);
        }

        //search the candidate with the smallest change in area that is valid
        let best_candidate = candidates
            .iter()
            .sorted_by_cached_key(|c| {
                OrderedFloat(calculate_area_delta(&ref_points, c).unwrap_or(Scalar::INFINITY))
            })
            .find(|c| candidate_is_valid(&ref_points, c));

        //if it is within the area change constraints, execute the candidate
        if let Some(best_candidate) = best_candidate {
            let new_shape = execute_candidate(&ref_points, best_candidate);
            let new_shape_area = SPolygon::calculate_area(&new_shape);
            let area_delta = (new_shape_area - original_area).abs() / original_area;
            if area_delta <= max_area_change_ratio {
                debug!(
                    "[PS] executed {:?} simplification causing {:.2}% area change",
                    best_candidate,
                    area_delta * 100.0
                );
                ref_points = new_shape;
            } else {
                break; //area change too significant
            }
        } else {
            break; //no candidate found
        }
    }

    //Convert it back to a simple polygon
    let simpl_shape = SPolygon::new(ref_points).unwrap();

    if simpl_shape.n_vertices() < shape.n_vertices() {
        info!(
            "[PS] simplified from {} to {} edges with {:.3}% area difference",
            shape.n_vertices(),
            simpl_shape.n_vertices(),
            (simpl_shape.area - shape.area) / shape.area * 100.0
        );
    } else {
        info!("[PS] no simplification possible within area change constraints");
    }

    simpl_shape
}

/// FORK(ironnest): vertex count above which a shape's *collision footprint* is decimated (when
/// `ShapeModifyConfig::collision_decimation` is set) instead of taking the exact offset path. Below
/// it, [`OriginalShape::convert_to_internal`](crate::original_shape::OriginalShape::convert_to_internal)
/// offsets the original exactly — so simple parts and low-vertex containers (incl. axis-aligned sheets,
/// where the offset is already exact) keep byte-for-byte behavior and the determinism golden is
/// untouched. 32 sits well above any hand-defined polygon yet far below the hundreds-of-vertex curved
/// shells (e.g. developed cone/reducer outlines) that are both slow and slightly under-reserved on
/// curves by the polygonal offsetter.
pub const DECIMATION_MIN_VERTICES: usize = 32;

/// FORK(ironnest): Deterministic Douglas–Peucker simplification of a **closed** polygon, used to
/// decimate a *collision footprint* — never the returned/original outline (the engine still
/// reports placements in the original frame; see
/// [`OriginalShape::convert_to_internal`](crate::original_shape::OriginalShape::convert_to_internal)).
///
/// Every retained vertex is an original vertex and DP guarantees no point of the original boundary
/// lies more than `tol` from the simplified boundary. The caller therefore offsets the result
/// outward by **`offset + tol`**: the `+tol` exactly compensates DP's bounded inward deviation so the
/// offset footprint is a superset of `original ⊕ disk(offset)` (and the same margin swamps the
/// straight-skeleton offsetter's sub-mil polygonal curve deficit). This is what keeps two placed
/// *original* outlines ≥ `2·offset` apart on curved parts where exact offsetting under-reserved.
///
/// DETERMINISM(ironnest): pure `+ − × ÷` and comparisons — no transcendentals, no `mul_add`, no
/// hashing, and the kept-vertex set is independent of evaluation order — so it is byte-identical on
/// every target. The perpendicular-distance test is squared (`cross² > tol²·len²`) to avoid `sqrt`
/// and any sign/zero edge case. Closed-ring DP splits at vertex 0 and the vertex farthest from it
/// (lowest index breaks ties), then runs standard open-polyline DP on the two arcs.
#[must_use]
pub fn simplify_dp(shape: &SPolygon, tol: Scalar) -> SPolygon {
    let pts = &shape.vertices;
    let n = pts.len();
    if n <= 4 || tol <= 0.0 {
        return shape.clone(); // nothing to gain on a quad/tri (or a non-positive tolerance)
    }
    let tol2 = tol * tol;

    // Split anchors: vertex 0 and the vertex farthest from it (lowest index wins ties).
    let anchor = 0usize;
    let mut far = anchor;
    let mut far_d2 = -1.0;
    for (i, p) in pts.iter().enumerate() {
        let (dx, dy) = (p.0 - pts[anchor].0, p.1 - pts[anchor].1);
        let d2 = dx * dx + dy * dy;
        if d2 > far_d2 {
            far_d2 = d2;
            far = i;
        }
    }
    if far == anchor {
        return shape.clone(); // degenerate: all vertices coincident
    }

    let mut keep = vec![false; n];
    keep[anchor] = true;
    keep[far] = true;
    dp_mark(pts, anchor, far, n, tol2, &mut keep);
    dp_mark(pts, far, anchor, n, tol2, &mut keep);

    let kept: Vec<Point> = (0..n).filter(|&i| keep[i]).map(|i| pts[i]).collect();

    // A simple polygon needs ≥ 3 vertices; if DP collapsed it (only possible when `tol` approaches
    // the shape's own size) keep the original rather than emit something degenerate.
    match SPolygon::new(kept) {
        Ok(simplified) if simplified.n_vertices() >= 3 => simplified,
        _ => shape.clone(),
    }
}

/// Marks (`keep[idx] = true`) the Douglas–Peucker vertices on the open arc running from `from` to
/// `to` in increasing index order modulo `n` (endpoints excluded). Iterative (explicit stack) so a
/// high-vertex part cannot overflow the call stack; the kept-set is independent of stack ordering.
fn dp_mark(pts: &[Point], from: usize, to: usize, n: usize, tol2: Scalar, keep: &mut [bool]) {
    let mut stack = vec![(from, to)];
    while let Some((from, to)) = stack.pop() {
        let (start, end) = (pts[from], pts[to]);
        let (dx, dy) = (end.0 - start.0, end.1 - start.1);
        let seg2 = dx * dx + dy * dy;

        // Farthest vertex on the arc from the chord start→end. For a fixed chord, perpendicular
        // distance is monotone in |cross|, so track the max of `cross²` (or point-distance² if the
        // chord is degenerate, i.e. start == end).
        let mut farthest: Option<usize> = None;
        let mut best_metric2 = 0.0;
        let mut idx = (from + 1) % n;
        while idx != to {
            let p = pts[idx];
            let metric2 = if seg2 > 0.0 {
                let cross = dx * (p.1 - start.1) - dy * (p.0 - start.0);
                cross * cross
            } else {
                let (ex, ey) = (p.0 - start.0, p.1 - start.1);
                ex * ex + ey * ey
            };
            if metric2 > best_metric2 {
                best_metric2 = metric2;
                farthest = Some(idx);
            }
            idx = (idx + 1) % n;
        }

        // Keep the farthest vertex iff it deviates by more than `tol`. Squared, no sqrt:
        //   dist > tol  ⟺  cross²/seg² > tol²  ⟺  cross² > tol²·seg²   (or pt² > tol² when seg² == 0).
        let threshold2 = if seg2 > 0.0 { tol2 * seg2 } else { tol2 };
        if let Some(mid) = farthest
            && best_metric2 > threshold2
        {
            keep[mid] = true;
            stack.push((from, mid));
            stack.push((mid, to));
        }
    }
}

fn calculate_area_delta(
    shape: &[Point],
    candidate: &Candidate,
) -> Result<Scalar, InvalidCandidate> {
    //calculate the difference in area of the shape if the candidate were to be executed
    let area = match candidate {
        Candidate::Collinear(_) => 0.0,
        Candidate::Concave(c) => {
            //Triangle formed by i_prev, i and i_next will correspond to the change area
            let Point(x0, y0) = shape[c.0];
            let Point(x1, y1) = shape[c.1];
            let Point(x2, y2) = shape[c.2];

            let area = (x0 * y1 + x1 * y2 + x2 * y0 - x0 * y2 - x1 * y0 - x2 * y1) / 2.0;

            area.abs()
        }
        Candidate::ConvexConvex(c1, c2) => {
            let replacing_vertex = replacing_vertex_convex_convex_candidate(shape, (*c1, *c2))?;

            //the triangle formed by corner c1, c2, and replacing vertex will correspond to the change in area
            let Point(x0, y0) = shape[c1.1];
            let Point(x1, y1) = replacing_vertex;
            let Point(x2, y2) = shape[c2.1];

            let area = (x0 * y1 + x1 * y2 + x2 * y0 - x0 * y2 - x1 * y0 - x2 * y1) / 2.0;

            area.abs()
        }
    };
    Ok(area)
}

fn candidate_is_valid(shape: &[Point], candidate: &Candidate) -> bool {
    //ensure the removal/replacement does not create any self intersections
    match candidate {
        Candidate::Collinear(_) => true,
        Candidate::Concave(c) => {
            let new_edge = Edge::try_new(shape[c.0], shape[c.2]).unwrap();
            let affected_points = [shape[c.0], shape[c.1], shape[c.2]];

            //check for self-intersections
            edge_iter(shape)
                .filter(|l| !affected_points.contains(&l.start))
                .filter(|l| !affected_points.contains(&l.end))
                .all(|l| !l.collides_with(&new_edge))
        }
        Candidate::ConvexConvex(c1, c2) => {
            match replacing_vertex_convex_convex_candidate(shape, (*c1, *c2)) {
                Err(_) => false,
                Ok(new_vertex) => {
                    let new_edge_1 = Edge::try_new(shape[c1.0], new_vertex).unwrap();
                    let new_edge_2 = Edge::try_new(new_vertex, shape[c2.2]).unwrap();

                    let affected_points = [shape[c1.1], shape[c1.0], shape[c2.1], shape[c2.2]];

                    //check for self-intersections
                    edge_iter(shape)
                        .filter(|l| !affected_points.contains(&l.start))
                        .filter(|l| !affected_points.contains(&l.end))
                        .all(|l| !l.collides_with(&new_edge_1) && !l.collides_with(&new_edge_2))
                }
            }
        }
    }
}

fn edge_iter(points: &[Point]) -> impl Iterator<Item = Edge> + '_ {
    let n_points = points.len();
    (0..n_points).map(move |i| {
        let j = (i + 1) % n_points;
        Edge::try_new(points[i], points[j]).unwrap()
    })
}

fn execute_candidate(shape: &[Point], candidate: &Candidate) -> Vec<Point> {
    let mut points = shape.iter().copied().collect_vec();
    match candidate {
        Candidate::Collinear(c) | Candidate::Concave(c) => {
            points.remove(c.1);
        }
        Candidate::ConvexConvex(c1, c2) => {
            let replacing_vertex = replacing_vertex_convex_convex_candidate(shape, (*c1, *c2))
                .expect("invalid candidate cannot be executed");
            points.remove(c1.1);
            let other_index = if c1.1 < c2.1 { c2.1 - 1 } else { c2.1 };
            points.remove(other_index);
            points.insert(other_index, replacing_vertex);
        }
    }
    points
}

fn replacing_vertex_convex_convex_candidate(
    shape: &[Point],
    (c1, c2): (Corner, Corner),
) -> Result<Point, InvalidCandidate> {
    assert_eq!(c1.2, c2.1, "non-consecutive corners {c1:?},{c2:?}");
    assert_eq!(c1.1, c2.0, "non-consecutive corners {c1:?},{c2:?}");

    let edge_prev = Edge::try_new(shape[c1.0], shape[c1.1]).unwrap();
    let edge_next = Edge::try_new(shape[c2.2], shape[c2.1]).unwrap();

    calculate_intersection_in_front(&edge_prev, &edge_next).ok_or(InvalidCandidate)
}

fn calculate_intersection_in_front(l1: &Edge, l2: &Edge) -> Option<Point> {
    //Calculates the intersection point between l1 and l2 if both were extended in front to infinity.

    //https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line_segment
    //vector 1 = [(x1,y1),(x2,y2)[ and vector 2 = [(x3,y3),(x4,y4)[
    let Point(x1, y1) = l1.start;
    let Point(x2, y2) = l1.end;
    let Point(x3, y3) = l2.start;
    let Point(x4, y4) = l2.end;

    //used formula is slightly different to the one on wikipedia. The orientation of the line segments are flipped
    //We consider an intersection if t == ]0,1] && u == ]0,1]

    let t_nom = (x2 - x4) * (y4 - y3) - (y2 - y4) * (x4 - x3);
    let t_denom = (x2 - x1) * (y4 - y3) - (y2 - y1) * (x4 - x3);

    let u_nom = (x2 - x4) * (y2 - y1) - (y2 - y4) * (x2 - x1);
    let u_denom = (x2 - x1) * (y4 - y3) - (y2 - y1) * (x4 - x3);

    let t = if t_denom == 0.0 { 0.0 } else { t_nom / t_denom };

    let u = if u_denom == 0.0 { 0.0 } else { u_nom / u_denom };

    if t < 0.0 && u < 0.0 {
        //intersection is in front both vectors
        Some(Point(x2 + t * (x1 - x2), y2 + t * (y1 - y2)))
    } else {
        //no intersection (parallel or not in front)
        None
    }
}

#[derive(Debug, Clone)]
struct InvalidCandidate;

#[derive(Clone, Debug, PartialEq)]
enum Candidate {
    Concave(Corner),
    ConvexConvex(Corner, Corner),
    Collinear(Corner),
}

#[derive(Clone, Copy, Debug, PartialEq)]
///Corner is defined as the left hand side of points 0-1-2
struct Corner(pub usize, pub usize, pub usize);

impl Corner {
    pub fn flip(&mut self) {
        std::mem::swap(&mut self.0, &mut self.2);
    }

    pub fn to_points(self, points: &[Point]) -> [Point; 3] {
        [points[self.0], points[self.1], points[self.2]]
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum CornerType {
    Concave,
    Convex,
    Collinear,
}

impl CornerType {
    pub fn from([p1, p2, p3]: [Point; 3]) -> Self {
        //returns the corner type on the left-hand side p1->p2->p3
        //From: https://algorithmtutor.com/Computational-Geometry/Determining-if-two-consecutive-segments-turn-left-or-right/

        let p1p2 = (p2.0 - p1.0, p2.1 - p1.1);
        let p1p3 = (p3.0 - p1.0, p3.1 - p1.1);
        let cross_prod = p1p2.0 * p1p3.1 - p1p2.1 * p1p3.0;

        //a positive cross product indicates that p2p3 turns to the left with respect to p1p2
        match cross_prod.partial_cmp(&0.0).expect("cross product is NaN") {
            Ordering::Less => CornerType::Concave,
            Ordering::Equal => CornerType::Collinear,
            Ordering::Greater => CornerType::Convex,
        }
    }
}

/// Offsets a [`SPolygon`] by a certain `distance` either inwards or outwards depending on the [`ShapeModifyMode`].
/// Relies on the **vendored** straight-skeleton offsetter ([`crate::buffer`], ex-`geo-buffer 0.2`).
///
/// DETERMINISM(ironnest) — this was docs/00 hazard #2 / docs/01 plan-risk #2 (the lone cross-platform
/// residual): upstream `geo-buffer`'s rounded arc-joins called **std** `f64::sin`/`cos` (platform C
/// libm, ULP-divergent). We vendored the offsetter and routed those two lines through the pure-Rust
/// `libm` crate, so this output — which feeds `OriginalShape::convert_to_internal` → the item's
/// collision shape whenever `min_item_separation != 0` — is now **byte-identical on every target**.
/// The x-platform golden covers a nonzero-`min_sep` case as proof.
pub fn offset_shape(sp: &SPolygon, mode: ShapeModifyMode, distance: Scalar) -> Result<SPolygon> {
    let offset = match mode {
        ShapeModifyMode::Deflate => -distance,
        ShapeModifyMode::Inflate => distance,
    };

    // Convert the SPolygon to a geo_types::Polygon. f64-native throughout — no f32 round-trip.
    let geo_poly =
        geo_types::Polygon::new(sp.vertices.iter().map(|p| (p.0, p.1)).collect(), vec![]);

    // Create the offset polygon (vendored, libm-deterministic).
    let geo_poly_offsets = crate::buffer::buffer_polygon_rounded(&geo_poly, offset).0;

    let geo_poly_offset = match geo_poly_offsets.len() {
        0 => bail!("Offset resulted in an empty polygon"),
        1 => &geo_poly_offsets[0],
        _ => {
            // If there are multiple polygons, we take the first one.
            // This can happen if the offset creates multiple disconnected parts.
            warn!("Offset resulted in multiple polygons, taking the first one.");
            &geo_poly_offsets[0]
        }
    };

    // Convert back to an internal `SPolygon`.
    let points = geo_poly_offset
        .exterior()
        .points()
        .map(|p| Point(p.x(), p.y()))
        .collect_vec();

    spolygon_from_ring(points)
}

/// Builds an [`SPolygon`] from the exterior ring produced by the offsetter.
///
/// FORK(ironnest): jagua's `offset_shape` round-tripped through
/// `crate::io::import::import_simple_polygon`, but `io` lives in the `cde` crate — a `geo → cde`
/// cycle. We inline the identical cleanup here so `geo` stays a leaf crate: strip a duplicate
/// closing vertex, drop consecutive (near-)duplicates, reject non-consecutive duplicates, then
/// validate via [`SPolygon::new`]. `cde`'s `io::import` keeps its own faithful copy for item /
/// container import.
fn spolygon_from_ring(mut points: Vec<Point>) -> Result<SPolygon> {
    // Strip the last vertex if it is the same as the first one.
    if points.len() > 1 && points[0] == points[points.len() - 1] {
        points.pop();
    }
    // Remove consecutive (near-)duplicates (e.g. [1, 2, 2, 3] -> [1, 2, 3]).
    eliminate_degenerate_vertices(&mut points);
    // Bail if there are any non-consecutive duplicates.
    if points.len() != points.iter().unique().count() {
        bail!("Offset polygon has non-consecutive duplicate vertices");
    }
    SPolygon::new(points)
}

/// Removes consecutive (near-)duplicate vertices in place, comparing with [`FPA`] tolerance.
///
/// Keep behaviourally in sync with `ironnest_cde::io::import::eliminate_degenerate_vertices` (the
/// canonical copy used for item/container import; this one exists only to keep `geo` a leaf crate).
fn eliminate_degenerate_vertices(points: &mut Vec<Point>) {
    let mut indices_to_remove = vec![];
    let n_points = points.len();
    for i in 0..n_points {
        let j = (i + 1) % n_points;
        let (p_i, p_j) = (points[i], points[j]);
        if FPA(p_i.0) == FPA(p_j.0) && FPA(p_i.1) == FPA(p_j.1) {
            indices_to_remove.push(i);
        }
    }
    //remove points in reverse order to avoid shifting indices
    indices_to_remove.sort_unstable_by(|a, b| b.cmp(a));
    for index in indices_to_remove {
        if index < points.len() {
            points.remove(index);
        }
    }
}

#[allow(clippy::too_many_lines)]
/// Closes narrow concavities in a [`SPolygon`] by replacing them with a straight edge, eliminating the vertices in between.
#[must_use]
pub fn close_narrow_concavities(
    orig_shape: &SPolygon,
    mode: ShapeModifyMode,
    (cutoff_distance_ratio, cutoff_area_ratio): (Scalar, Scalar),
) -> SPolygon {
    let mut n_concav_closed = 0;
    let mut shape = orig_shape.clone();

    for _ in 0..shape.n_vertices() {
        let n_points = shape.n_vertices();

        let calc_vert_elim = |i, j| {
            if j > i {
                j - i - 1
            } else {
                n_points - i + j - 1
            }
        };

        let mut best_candidate = None;
        for i in 0..n_points {
            for j in 0..n_points {
                if i == j || (i + 1) % n_points == j || (j + 1) % n_points == i {
                    continue; //skip adjacent points
                }
                //Simulate the replacing edge
                let c_edge = Edge::try_new(shape.vertex(i), shape.vertex(j))
                    .expect("invalid edge in string candidate")
                    .scale(0.9999); //slightly shrink the edge to avoid self-intersections

                if c_edge.length() > cutoff_distance_ratio * shape.diameter {
                    //If the edge is too long, skip it
                    continue;
                }

                if mode == ShapeModifyMode::Inflate
                    && (shape.collides_with(&c_edge.start) || shape.collides_with(&c_edge.end))
                {
                    //If we are only allowed to inflate the shape and any end point is inside the shape, skip it
                    continue;
                }

                if mode == ShapeModifyMode::Deflate
                    && !(shape.collides_with(&c_edge.start) && shape.collides_with(&c_edge.end))
                {
                    //If we are only allowed to deflate the shape and both end points are not inside the shape, skip it
                    continue;
                }

                if shape.edge_iter().any(|e| e.collides_with(&c_edge)) {
                    //If the edge collides with any edge of the shape, reject always
                    continue;
                }
                //the eliminated vertices should form a negative area (in inflation mode) or positive area (in deflation mode)
                let sub_shape_area = {
                    let sub_shape_points = if j > i {
                        shape.vertices[i..j].to_vec()
                    } else {
                        [&shape.vertices[i..], &shape.vertices[..j]].concat()
                    };
                    SPolygon::calculate_area(&sub_shape_points)
                };
                if sub_shape_area >= 0.0 {
                    //if the area is not negative, skip it
                    continue;
                }
                if sub_shape_area.abs() > cutoff_area_ratio * shape.area {
                    //if the area is too large, skip it
                    continue;
                }

                //Valid candidate found...
                match best_candidate {
                    None => {
                        //first candidate found
                        best_candidate = Some((i, j));
                    }
                    Some((best_i, best_j)) => {
                        //check the number of points that would be removed
                        if calc_vert_elim(i, j) > calc_vert_elim(best_i, best_j) {
                            best_candidate = Some((i, j));
                        }
                    }
                }
            }
        }
        if let Some((i, j)) = best_candidate {
            let mut ref_points = shape.vertices.clone();
            let start = i.cast_signed() + 1;
            let end = j.cast_signed() - 1;
            debug!(
                "[PS] closing concavity between points (idx: {}, {:?}) and (idx: {}, {:?}) with edge length {:.3} ({} vertices eliminated)",
                i,
                shape.vertex(i),
                j,
                shape.vertex(j),
                Edge::try_new(shape.vertex(i), shape.vertex(j))
                    .expect("invalid edge in string candidate")
                    .length(),
                calc_vert_elim(i, j)
            );
            if start <= end {
                // if j does not wrap around the shape
                ref_points.drain(start.cast_unsigned()..=end.cast_unsigned());
            } else {
                // if j wraps around the shape
                if start.cast_unsigned() < n_points {
                    //remove from `start` to back
                    ref_points.drain(start.cast_unsigned()..);
                }
                if end >= 0 {
                    //remove from front to `end`
                    ref_points.drain(0..=end.cast_unsigned());
                }
            }
            shape = SPolygon::new(ref_points).expect("invalid shape after closing concavity");
            n_concav_closed += 1;
        } else {
            //no more candidates found, break the loop
            break;
        }
    }

    if n_concav_closed > 0 {
        info!(
            "[PS] [EXPERIMENTAL] closed {} concavities closer than {:.3}% of diameter and less than {:.3}% of area, reducing vertices from {} to {}",
            n_concav_closed,
            cutoff_distance_ratio * 100.0,
            cutoff_area_ratio * 100.0,
            orig_shape.n_vertices(),
            shape.n_vertices()
        );
    }

    shape
}

#[must_use]
pub fn shape_modification_valid(orig: &SPolygon, simpl: &SPolygon, mode: ShapeModifyMode) -> bool {
    //make sure each point of the original shape is either in the new shape or included (in case of inflation)/excluded (in case of deflation) in the new shape
    let on_edge = |p: &Point| {
        simpl
            .edge_iter()
            .any(|e| e.distance_to(p) < simpl.diameter * 1e-6)
    };

    for p in orig.vertices.iter().filter(|p| !simpl.vertices.contains(p)) {
        let vertex_on_edge = on_edge(p);
        let vertex_in_simpl = simpl.collides_with(p);

        let error = match mode {
            ShapeModifyMode::Inflate => !vertex_in_simpl && !vertex_on_edge,
            ShapeModifyMode::Deflate => vertex_in_simpl && !vertex_on_edge,
        };

        if error {
            error!(
                "[PS] point {:?} from original shape is incorrect in simplified shape (original vertices: {:?}, simplified vertices: {:?})",
                p,
                orig.vertices.iter().map(|p| (p.0, p.1)).collect_vec(),
                simpl.vertices.iter().map(|p| (p.0, p.1)).collect_vec()
            );
            return false; //point is not in the new shape and does not collide with it
        }
    }
    true
}

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use crate::Scalar;
use crate::fpa::FPA;
use crate::geo_enums::{GeoPosition, GeoRelation};
use crate::geo_traits::{AlmostCollidesWith, CollidesWith, DistanceTo, SeparationDistance};
use crate::primitives::Edge;
use crate::primitives::Point;
use anyhow::Result;
use anyhow::ensure;
use ordered_float::OrderedFloat;

///Axis-aligned rectangle
#[derive(Clone, Debug, PartialEq, Copy)]
pub struct Rect {
    pub x_min: Scalar,
    pub y_min: Scalar,
    pub x_max: Scalar,
    pub y_max: Scalar,
}

impl Rect {
    pub fn try_new(x_min: Scalar, y_min: Scalar, x_max: Scalar, y_max: Scalar) -> Result<Self> {
        ensure!(
            x_min < x_max && y_min < y_max,
            "invalid rectangle, x_min: {x_min}, x_max: {x_max}, y_min: {y_min}, y_max: {y_max}"
        );
        Ok(Rect {
            x_min,
            y_min,
            x_max,
            y_max,
        })
    }

    pub fn from_diagonal_corners(c1: Point, c2: Point) -> Result<Self> {
        let x_min = Scalar::min(c1.x(), c2.x());
        let y_min = Scalar::min(c1.y(), c2.y());
        let x_max = Scalar::max(c1.x(), c2.x());
        let y_max = Scalar::max(c1.y(), c2.y());
        Rect::try_new(x_min, y_min, x_max, y_max)
    }

    /// Returns the geometric relation between `self` and another [`Rect`].
    /// Optimized for `GeoRelation::Disjoint`
    #[inline(always)]
    #[must_use]
    pub fn relation_to(&self, other: Rect) -> GeoRelation {
        if !self.collides_with(&other) {
            return GeoRelation::Disjoint;
        }
        if self.x_min <= other.x_min
            && self.y_min <= other.y_min
            && self.x_max >= other.x_max
            && self.y_max >= other.y_max
        {
            return GeoRelation::Surrounding;
        }
        if self.x_min >= other.x_min
            && self.y_min >= other.y_min
            && self.x_max <= other.x_max
            && self.y_max <= other.y_max
        {
            return GeoRelation::Enclosed;
        }
        GeoRelation::Intersecting
    }

    /// Returns the [`GeoRelation`] between `self` and another [`Rect`], with a tolerance for floating point precision.
    /// In edge cases, this method will lean towards `Surrounding` and `Enclosed` instead of `Intersecting`.
    #[inline(always)]
    #[must_use]
    pub fn almost_relation_to(&self, other: Rect) -> GeoRelation {
        if !self.almost_collides_with(&other) {
            return GeoRelation::Disjoint;
        }
        if FPA::from(self.x_min) <= FPA::from(other.x_min)
            && FPA::from(self.y_min) <= FPA::from(other.y_min)
            && FPA::from(self.x_max) >= FPA::from(other.x_max)
            && FPA::from(self.y_max) >= FPA::from(other.y_max)
        {
            return GeoRelation::Surrounding;
        }
        if FPA::from(self.x_min) >= FPA::from(other.x_min)
            && FPA::from(self.y_min) >= FPA::from(other.y_min)
            && FPA::from(self.x_max) <= FPA::from(other.x_max)
            && FPA::from(self.y_max) <= FPA::from(other.y_max)
        {
            return GeoRelation::Enclosed;
        }
        GeoRelation::Intersecting
    }

    /// Returns a new rectangle with the same centroid but inflated
    /// to be the minimum square that contains `self`.
    #[must_use]
    pub fn inflate_to_square(&self) -> Rect {
        let width = self.x_max - self.x_min;
        let height = self.y_max - self.y_min;
        let mut dx = 0.0;
        let mut dy = 0.0;
        if height < width {
            dy = (width - height) / 2.0;
        } else if width < height {
            dx = (height - width) / 2.0;
        }
        Rect {
            x_min: self.x_min - dx,
            y_min: self.y_min - dy,
            x_max: self.x_max + dx,
            y_max: self.y_max + dy,
        }
    }

    /// Returns a new rectangle with the same centroid but scaled by `factor`.
    #[must_use]
    pub fn scale(self, factor: Scalar) -> Self {
        let dx = (self.x_max - self.x_min) * (factor - 1.0) / 2.0;
        let dy = (self.y_max - self.y_min) * (factor - 1.0) / 2.0;
        self.resize_by(dx, dy)
            .expect("scaling should not lead to invalid rectangle")
    }

    /// Returns a new rectangle with the same centroid as `self` but expanded by `dx` in both x-directions and by `dy` in both y-directions.
    /// If the new rectangle is invalid (`x_min` >= `x_max` or `y_min` >= `y_max`), returns None.
    #[must_use]
    pub fn resize_by(mut self, dx: Scalar, dy: Scalar) -> Option<Self> {
        self.x_min -= dx;
        self.y_min -= dy;
        self.x_max += dx;
        self.y_max += dy;

        if self.x_min < self.x_max && self.y_min < self.y_max {
            Some(self)
        } else {
            //resizing would lead to invalid rectangle
            None
        }
    }

    /// For all quadrants, contains indices of the two neighbors of the quadrant at that index.
    pub const QUADRANT_NEIGHBOR_LAYOUT: [[usize; 2]; 4] = [[1, 3], [0, 2], [1, 3], [0, 2]];

    /// Returns the 4 quadrants of `self`.
    /// Ordered in the same way as quadrants in a cartesian plane:
    /// <https://en.wikipedia.org/wiki/Quadrant_(plane_geometry)>
    #[must_use]
    pub fn quadrants(&self) -> [Self; 4] {
        let mid = self.centroid();
        let corners = self.corners();

        let q1 = Rect::from_diagonal_corners(corners[0], mid).unwrap();
        let q2 = Rect::from_diagonal_corners(corners[1], mid).unwrap();
        let q3 = Rect::from_diagonal_corners(corners[2], mid).unwrap();
        let q4 = Rect::from_diagonal_corners(corners[3], mid).unwrap();

        [q1, q2, q3, q4]
    }

    /// Returns the four corners of `self`, in the same order as [`Rect::quadrants`].
    #[must_use]
    pub fn corners(&self) -> [Point; 4] {
        [
            Point(self.x_max, self.y_max),
            Point(self.x_min, self.y_max),
            Point(self.x_min, self.y_min),
            Point(self.x_max, self.y_min),
        ]
    }

    /// Returns the four sides that make up `self`, in the same order as [`Rect::quadrants`].
    #[must_use]
    pub fn sides(&self) -> [Edge; 4] {
        let c = self.corners();
        [
            Edge {
                start: c[0],
                end: c[1],
            },
            Edge {
                start: c[1],
                end: c[2],
            },
            Edge {
                start: c[2],
                end: c[3],
            },
            Edge {
                start: c[3],
                end: c[0],
            },
        ]
    }

    /// Returns the four edges that make up `self`, in the same order as [`Rect::quadrants`].
    #[must_use]
    pub fn edges(&self) -> [Edge; 4] {
        let c = self.corners();
        [
            Edge {
                start: c[0],
                end: c[1],
            },
            Edge {
                start: c[1],
                end: c[2],
            },
            Edge {
                start: c[2],
                end: c[3],
            },
            Edge {
                start: c[3],
                end: c[0],
            },
        ]
    }
    #[must_use]
    pub fn width(&self) -> Scalar {
        self.x_max - self.x_min
    }

    #[must_use]
    pub fn height(&self) -> Scalar {
        self.y_max - self.y_min
    }

    /// Returns the largest rectangle that is contained in both `a` and `b`.
    #[must_use]
    pub fn intersection(a: Rect, b: Rect) -> Option<Rect> {
        let x_min = Scalar::max(a.x_min, b.x_min);
        let y_min = Scalar::max(a.y_min, b.y_min);
        let x_max = Scalar::min(a.x_max, b.x_max);
        let y_max = Scalar::min(a.y_max, b.y_max);
        if x_min < x_max && y_min < y_max {
            Some(Rect {
                x_min,
                y_min,
                x_max,
                y_max,
            })
        } else {
            None
        }
    }

    /// Returns the smallest rectangle that contains both `a` and `b`.
    #[must_use]
    pub fn bounding_rect(a: Rect, b: Rect) -> Rect {
        let x_min = Scalar::min(a.x_min, b.x_min);
        let y_min = Scalar::min(a.y_min, b.y_min);
        let x_max = Scalar::max(a.x_max, b.x_max);
        let y_max = Scalar::max(a.y_max, b.y_max);
        Rect {
            x_min,
            y_min,
            x_max,
            y_max,
        }
    }

    #[must_use]
    pub fn centroid(&self) -> Point {
        Point(
            Scalar::midpoint(self.x_min, self.x_max),
            Scalar::midpoint(self.y_min, self.y_max),
        )
    }

    #[must_use]
    pub fn area(&self) -> Scalar {
        (self.x_max - self.x_min) * (self.y_max - self.y_min)
    }

    #[must_use]
    pub fn diameter(&self) -> Scalar {
        let dx = self.x_max - self.x_min;
        let dy = self.y_max - self.y_min;
        (dx.powi(2) + dy.powi(2)).sqrt()
    }
}

impl CollidesWith<Rect> for Rect {
    #[inline(always)]
    fn collides_with(&self, other: &Rect) -> bool {
        Scalar::max(self.x_min, other.x_min) <= Scalar::min(self.x_max, other.x_max)
            && Scalar::max(self.y_min, other.y_min) <= Scalar::min(self.y_max, other.y_max)
    }
}

impl AlmostCollidesWith<Rect> for Rect {
    #[inline(always)]
    fn almost_collides_with(&self, other: &Rect) -> bool {
        FPA(Scalar::max(self.x_min, other.x_min)) <= FPA(Scalar::min(self.x_max, other.x_max))
            && FPA(Scalar::max(self.y_min, other.y_min))
                <= FPA(Scalar::min(self.y_max, other.y_max))
    }
}

impl CollidesWith<Point> for Rect {
    #[inline(always)]
    fn collides_with(&self, point: &Point) -> bool {
        let Point(x, y) = *point;
        x >= self.x_min && x <= self.x_max && y >= self.y_min && y <= self.y_max
    }
}

impl AlmostCollidesWith<Point> for Rect {
    #[inline(always)]
    fn almost_collides_with(&self, point: &Point) -> bool {
        let (x, y) = (*point).into();
        FPA(x) >= FPA(self.x_min)
            && FPA(x) <= FPA(self.x_max)
            && FPA(y) >= FPA(self.y_min)
            && FPA(y) <= FPA(self.y_max)
    }
}

impl CollidesWith<Edge> for Rect {
    #[inline(always)]
    #[allow(clippy::similar_names)]
    fn collides_with(&self, edge: &Edge) -> bool {
        //inspired by: https://stackoverflow.com/questions/99353/how-to-test-if-a-line-segment-intersects-an-axis-aligned-rectange-in-2d

        //First check if the bounding boxes of the rectangle and edge overlap
        let e_x_min = edge.x_min();
        let e_x_max = edge.x_max();
        let e_y_min = edge.y_min();
        let e_y_max = edge.y_max();

        let x_no_overlap = e_x_min.max(self.x_min) > e_x_max.min(self.x_max);
        let y_no_overlap = e_y_min.max(self.y_min) > e_y_max.min(self.y_max);

        if x_no_overlap || y_no_overlap {
            // Edge is completely outside the x- or y-range of the rectangle
            return false;
        }

        if self.collides_with(&edge.start) || self.collides_with(&edge.end) {
            // Edge has at least one end point in the rectangle
            return true;
        }

        let Point(s_x, s_y) = edge.start;
        let Point(e_x, e_y) = edge.end;
        let edge_dx = e_x - s_x;
        let edge_dy = e_y - s_y;

        let c = self.corners();

        //All corners need to be on the same side of the edge for there to be no intersection.
        //Meaning the 2D cross-products should either all positive or all negative
        let sides = [
            (c[0].0 - s_x) * edge_dy - (c[0].1 - s_y) * edge_dx,
            (c[1].0 - s_x) * edge_dy - (c[1].1 - s_y) * edge_dx,
            (c[2].0 - s_x) * edge_dy - (c[2].1 - s_y) * edge_dx,
            (c[3].0 - s_x) * edge_dy - (c[3].1 - s_y) * edge_dx,
        ];

        let all_positive = sides.iter().all(|&s| s > 0.0);
        let all_negative = sides.iter().all(|&s| s < 0.0);
        !(all_positive || all_negative)
    }
}

impl DistanceTo<Point> for Rect {
    #[inline(always)]
    fn distance_to(&self, point: &Point) -> Scalar {
        self.sq_distance_to(point).sqrt()
    }

    #[inline(always)]
    fn sq_distance_to(&self, point: &Point) -> Scalar {
        let Point(x, y) = *point;
        let mut distance: Scalar = 0.0;
        if x < self.x_min {
            distance += (x - self.x_min).powi(2);
        } else if x > self.x_max {
            distance += (x - self.x_max).powi(2);
        }
        if y < self.y_min {
            distance += (y - self.y_min).powi(2);
        } else if y > self.y_max {
            distance += (y - self.y_max).powi(2);
        }
        distance.abs()
    }
}

impl SeparationDistance<Point> for Rect {
    #[inline(always)]
    fn separation_distance(&self, point: &Point) -> (GeoPosition, Scalar) {
        let (position, sq_distance) = self.sq_separation_distance(point);
        (position, sq_distance.sqrt())
    }

    #[inline(always)]
    fn sq_separation_distance(&self, point: &Point) -> (GeoPosition, Scalar) {
        if self.collides_with(point) {
            let Point(x, y) = *point;
            let min_distance = [
                (x - self.x_min).abs(),
                (x - self.x_max).abs(),
                (y - self.y_min).abs(),
                (y - self.y_max).abs(),
            ]
            .into_iter()
            .min_by_key(|&d| OrderedFloat(d))
            .unwrap();
            (GeoPosition::Interior, min_distance.powi(2))
        } else {
            (GeoPosition::Exterior, self.sq_distance_to(point))
        }
    }
}

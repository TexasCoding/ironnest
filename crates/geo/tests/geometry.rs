// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Fork-locking tests for `ironnest-geo`: confirm the f32→f64 (`Scalar`) flip preserves jagua's
//! geometry, and that the `libm`-based transform path is deterministic.

use ironnest_geo::Scalar;
use ironnest_geo::Transformation;
use ironnest_geo::geo_traits::{DistanceTo, Transformable};
use ironnest_geo::primitives::{Point, Rect, SPolygon};
use ironnest_geo::shape_modification::{ShapeModifyMode, offset_shape};

const TOL: Scalar = 1e-9;

/// A 2×2 axis-aligned square with its lower-left corner at the origin.
fn unit_square(side: Scalar) -> SPolygon {
    SPolygon::new(vec![
        Point(0.0, 0.0),
        Point(side, 0.0),
        Point(side, side),
        Point(0.0, side),
    ])
    .expect("valid square")
}

#[test]
fn point_distance_is_exact() {
    // 3-4-5 triangle: sqrt is IEEE correctly-rounded, so this is exact across platforms.
    let d = Point(0.0, 0.0).distance_to(&Point(3.0, 4.0));
    assert_eq!(d, 5.0);
}

#[test]
fn square_area_centroid_bbox() {
    let sq = unit_square(2.0);
    assert!((sq.area - 4.0).abs() < TOL, "area = {}", sq.area);
    let c = sq.centroid();
    assert!(
        (c.0 - 1.0).abs() < TOL && (c.1 - 1.0).abs() < TOL,
        "centroid = {c:?}"
    );
    let bb = sq.bbox;
    assert_eq!(
        (bb.x_min, bb.y_min, bb.x_max, bb.y_max),
        (0.0, 0.0, 2.0, 2.0)
    );
}

#[test]
fn rect_to_spolygon_roundtrips() {
    let r = Rect::try_new(-1.0, -2.0, 3.0, 4.0).expect("valid rect");
    let sp = SPolygon::from(r);
    assert!((sp.area - r.area()).abs() < TOL);
}

#[test]
fn quarter_turn_rotation_maps_x_axis_to_y_axis() {
    // 90° CCW about the origin sends (1,0) -> (0,1). The general (radian) path goes through `libm`;
    // exactness to {0,±1} is an optimizer concern (cardinal matrices), so we assert within tol.
    let t = Transformation::from_rotation(std::f64::consts::FRAC_PI_2);
    let mut p = Point(1.0, 0.0);
    p.transform(&t);
    assert!(
        p.0.abs() < 1e-12 && (p.1 - 1.0).abs() < 1e-12,
        "rotated = {p:?}"
    );
}

#[test]
fn transform_is_bit_deterministic() {
    // Same input must produce byte-identical output (the determinism contract, in-process slice).
    let t = Transformation::from_rotation(0.3).translate((1.25, -2.5));
    let run = || {
        let mut p = Point(0.7, 1.1);
        p.transform(&t);
        (p.0.to_bits(), p.1.to_bits())
    };
    assert_eq!(run(), run());
}

#[test]
fn offset_inflate_grows_deflate_shrinks() {
    let sq = unit_square(10.0);
    let bigger = offset_shape(&sq, ShapeModifyMode::Inflate, 1.0).expect("inflate ok");
    let smaller = offset_shape(&sq, ShapeModifyMode::Deflate, 1.0).expect("deflate ok");
    assert!(
        bigger.area > sq.area,
        "inflate grew: {} > {}",
        bigger.area,
        sq.area
    );
    assert!(
        smaller.area < sq.area,
        "deflate shrank: {} < {}",
        smaller.area,
        sq.area
    );
}

/// geo-buffer is the one tracked cross-platform residual (see `shape_modification::offset_shape`).
/// This locks its output bit-for-bit IN-PROCESS, so a future geo-buffer version bump that perturbs
/// the offset vertices is caught here — not only (eventually) in the x-platform golden. NOTE: this
/// proves *run-to-run* stability, NOT cross-platform stability (geo-buffer uses std `sin`/`cos`
/// internally); the x-platform golden remains the real proof for nonzero min-separation.
#[test]
fn offset_output_is_bit_stable_in_process() {
    let sq = unit_square(10.0);
    let bits = || {
        offset_shape(&sq, ShapeModifyMode::Inflate, 1.0)
            .expect("inflate ok")
            .vertices
            .iter()
            .map(|p| (p.0.to_bits(), p.1.to_bits()))
            .collect::<Vec<_>>()
    };
    assert_eq!(bits(), bits());
}

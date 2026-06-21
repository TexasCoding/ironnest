// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Fork-locking tests for `ironnest-geo`: confirm the f32→f64 (`Scalar`) flip preserves jagua's
//! geometry, and that the `libm`-based transform path is deterministic.

use ironnest_geo::Scalar;
use ironnest_geo::Transformation;
use ironnest_geo::geo_traits::{DistanceTo, Transformable};
use ironnest_geo::primitives::{Point, Rect, SPolygon};
use ironnest_geo::shape_modification::{ShapeModifyMode, offset_shape, simplify_dp};

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

/// An `n`-vertex CCW regular polygon ("circle") of radius `r`. Test-only input generation — std trig
/// is fine here (this is not the deterministic placement path the gate targets).
#[allow(clippy::disallowed_methods)]
fn circle_spolygon(n: usize, r: Scalar) -> SPolygon {
    let pts = (0..n)
        .map(|i| {
            let a = std::f64::consts::TAU * (i as Scalar) / (n as Scalar);
            Point(r * a.cos(), r * a.sin())
        })
        .collect();
    SPolygon::new(pts).expect("valid circle")
}

/// Distance from point `p` to segment `a`–`b` (pure arithmetic; independent of the engine).
fn point_seg_dist(p: Point, a: Point, b: Point) -> Scalar {
    let (abx, aby) = (b.0 - a.0, b.1 - a.1);
    let len2 = abx * abx + aby * aby;
    if len2 == 0.0 {
        return ((p.0 - a.0).powi(2) + (p.1 - a.1).powi(2)).sqrt();
    }
    let t = (((p.0 - a.0) * abx + (p.1 - a.1) * aby) / len2).clamp(0.0, 1.0);
    let (px, py) = (a.0 + t * abx, a.1 + t * aby);
    ((p.0 - px).powi(2) + (p.1 - py).powi(2)).sqrt()
}

/// A `teeth`-tooth gear of `samples` vertices (radius oscillates between `r_in` and `r_out`) — a
/// high-vertex CONCAVE closed polygon. Test-only input generation (std trig is fine here).
#[allow(clippy::disallowed_methods)]
fn gear_spolygon(teeth: usize, r_out: Scalar, r_in: Scalar, samples: usize) -> SPolygon {
    let mid = (r_out + r_in) / 2.0;
    let amp = (r_out - r_in) / 2.0;
    let pts = (0..samples)
        .map(|i| {
            let a = std::f64::consts::TAU * (i as Scalar) / (samples as Scalar);
            let rr = mid + amp * (teeth as Scalar * a).cos();
            Point(rr * a.cos(), rr * a.sin())
        })
        .collect();
    SPolygon::new(pts).expect("valid gear")
}

/// Min distance from every vertex of `boundary` to polygon `region`'s edges.
fn min_vertex_clearance(boundary: &SPolygon, region: &SPolygon) -> Scalar {
    let n = region.n_vertices();
    boundary
        .vertices
        .iter()
        .map(|v| {
            (0..n)
                .map(|i| point_seg_dist(*v, region.vertices[i], region.vertices[(i + 1) % n]))
                .fold(Scalar::INFINITY, Scalar::min)
        })
        .fold(Scalar::INFINITY, Scalar::min)
}

#[test]
fn deflate_decimation_over_reserves_a_concave_boundary() {
    // Container side of the fix. A high-vertex CONCAVE boundary deflated on the decimation path (DP,
    // then Deflate by min_sep/2 + tol) must keep the deflated wall AT LEAST min_sep/2 inside the
    // ORIGINAL wall everywhere — so a part footprint touching the deflated wall stays ≥ min_sep/2 from
    // the original boundary (and thus, with the part's own min_sep/2 reserve, ≥ min_sep). The straight-
    // skeleton offsetter under-reserves a concave curve by a chord deficit; the +tol margin covers it.
    let cont = gear_spolygon(8, 30.0, 22.0, 240);
    let min_sep = 0.75;
    let half = min_sep / 2.0;
    let tol = min_sep / 16.0;

    let decimated = offset_shape(
        &simplify_dp(&cont, tol),
        ShapeModifyMode::Deflate,
        half + tol,
    )
    .expect("deflate ok");
    let clearance = min_vertex_clearance(&cont, &decimated);
    assert!(
        clearance >= half - 1e-9,
        "decimated deflate reserves only {clearance} from the original wall, under min_sep/2 {half}"
    );

    // Proof the fix is necessary: the EXACT deflate (no decimation, no +tol) under-reserves the
    // concave troughs — the same offsetter deficit the part-side change addresses, here on the wall.
    let exact = offset_shape(&cont, ShapeModifyMode::Deflate, half).expect("deflate ok");
    let exact_clearance = min_vertex_clearance(&cont, &exact);
    assert!(
        exact_clearance < half,
        "exact deflate of a concave curve should under-reserve (got {exact_clearance} vs {half}); \
         if it does not, the container-side margin would be unnecessary"
    );
}

#[test]
fn simplify_dp_decimates_a_circle_within_tolerance() {
    // The mechanism behind the curved-part fix. Douglas–Peucker turns a hundreds-of-vertex curve
    // into a few dozen vertices — collision cost scales with vertex count, so this IS the speedup
    // (≈ the consumer's 362→58, ~175× faster) — while keeping every original vertex within `tol` of
    // the simplified boundary. That bounded deviation is exactly what lets the optimizer offset the
    // result by `+tol` to recover a superset of the original (the min-separation guarantee).
    let circle = circle_spolygon(360, 12.0);
    let tol = 0.0625; // = min_sep/16 at a consumer-scale min_sep ≈ 1.0

    let simplified = simplify_dp(&circle, tol);

    // Heavy vertex reduction (the speedup): well under a quarter of the original.
    assert!(
        simplified.n_vertices() >= 3 && simplified.n_vertices() < circle.n_vertices() / 4,
        "expected heavy decimation, got {} of {} vertices",
        simplified.n_vertices(),
        circle.n_vertices()
    );

    // Bounded deviation (the correctness precondition): every ORIGINAL vertex lies within `tol` of
    // the simplified boundary, checked independently against every simplified edge (incl. the closing
    // edge).
    let n = simplified.n_vertices();
    for v in &circle.vertices {
        let d = (0..n)
            .map(|i| point_seg_dist(*v, simplified.vertices[i], simplified.vertices[(i + 1) % n]))
            .fold(Scalar::INFINITY, Scalar::min);
        assert!(
            d <= tol + 1e-9,
            "original vertex {v:?} is {d} from the simplified boundary (tol {tol})"
        );
    }

    // Deterministic: same input ⇒ byte-identical vertices (in-process slice of the contract).
    let bits = |sp: &SPolygon| {
        sp.vertices
            .iter()
            .map(|p| (p.0.to_bits(), p.1.to_bits()))
            .collect::<Vec<_>>()
    };
    assert_eq!(bits(&simplify_dp(&circle, tol)), bits(&simplified));
}

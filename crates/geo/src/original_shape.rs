// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use crate::DTransformation;
use crate::Scalar;
use crate::geo_traits::Transformable;
use crate::primitives::{Point, Rect, SPolygon};
use crate::shape_modification::{
    DECIMATION_MIN_VERTICES, ShapeModifyConfig, ShapeModifyMode, close_narrow_concavities,
    offset_shape, shape_modification_valid, simplify_dp, simplify_shape,
};
use anyhow::Result;

#[derive(Clone, Debug)]
/// A [`SPolygon`] exactly as is defined in the input file
///
/// Also contains all required operations to convert it to a shape that can be used internally.
/// Currently, these are centering and simplification operations, but could be extended in the future.
pub struct OriginalShape {
    pub shape: SPolygon,
    pub pre_transform: DTransformation,
    pub modify_mode: ShapeModifyMode,
    pub modify_config: ShapeModifyConfig,
}

impl OriginalShape {
    pub fn convert_to_internal(&self) -> Result<SPolygon> {
        // Apply the transformation
        let mut internal = self.shape.transform_clone(&self.pre_transform.compose());

        // FORK(ironnest): collision-footprint decimation path, taken only for HIGH-VERTEX shapes (the
        // curved parts/containers that are both slow and slightly under-reserved by the polygonal
        // offsetter). Douglas–Peucker simplify FIRST, then offset by `offset + tol`. Because DP keeps
        // the simplified boundary within `tol` of the original, offsetting by the extra `+tol` makes
        // the result a superset of `original ⊕ disk(offset)` for Inflate (and, symmetrically, a subset
        // of `original ⊖ disk(offset)` for Deflate). Empirically `tol` is also ~100× the offsetter's
        // polygonal curve deficit, so it covers that too — keeping placed *original* outlines ≥ the
        // intended clearance from each other AND from a curved container boundary. The gate on vertex
        // count keeps simple parts and low-vertex containers (incl. axis-aligned sheets) on the exact
        // path below, so their behavior — and the determinism golden — is byte-for-byte unchanged.
        // `self.shape` (the original outline that drives the reported placement frame) is never touched.
        if let Some(tol) = self.modify_config.collision_decimation
            && self.shape.n_vertices() > DECIMATION_MIN_VERTICES
        {
            // This path replaces (does not compose with) the default offset-then-simplify below.
            debug_assert!(
                self.modify_config.simplify_tolerance.is_none()
                    && self.modify_config.narrow_concavity_cutoff.is_none(),
                "collision_decimation is mutually exclusive with simplify_tolerance / narrow_concavity_cutoff"
            );
            internal = simplify_dp(&internal, tol);
            let total_offset = self.modify_config.offset.unwrap_or(0.0) + tol;
            if total_offset != 0.0 {
                internal = offset_shape(&internal, self.modify_mode, total_offset)?;
            }
            return Ok(internal);
        }

        if let Some(offset) = self.modify_config.offset {
            // Offset the shape
            if offset != 0.0 {
                internal = offset_shape(&internal, self.modify_mode, offset)?;
            }
        }
        if let Some(tolerance) = self.modify_config.simplify_tolerance {
            let pre_simplified = internal.clone();
            // Simplify the shape
            internal = simplify_shape(&internal, self.modify_mode, tolerance);
            if let Some(narrow_concavity_cutoff) = self.modify_config.narrow_concavity_cutoff {
                // Close narrow concavities
                internal =
                    close_narrow_concavities(&internal, self.modify_mode, narrow_concavity_cutoff);
                // Do another simplification after closing concavities
                internal = simplify_shape(&internal, self.modify_mode, tolerance / 10.0);
            }
            debug_assert!(shape_modification_valid(
                &pre_simplified,
                &internal,
                self.modify_mode
            ));
        }

        Ok(internal)
    }

    #[must_use]
    pub fn centroid(&self) -> Point {
        self.shape.centroid()
    }

    #[must_use]
    pub fn area(&self) -> Scalar {
        self.shape.area
    }

    #[must_use]
    pub fn bbox(&self) -> Rect {
        self.shape.bbox
    }

    #[must_use]
    pub fn diameter(&self) -> Scalar {
        self.shape.diameter
    }
}

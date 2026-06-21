// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use crate::Scalar;
use std::borrow::Borrow;
use std::f64::consts::PI;
use std::fmt::Display;

use crate::Transformation;
use ordered_float::NotNan;

#[derive(Clone, Debug, PartialEq, Eq, Hash, Copy, Default)]
/// [Proper rigid transformation](https://en.wikipedia.org/wiki/Rigid_transformation),
/// decomposed into a rotation followed by a translation.
pub struct DTransformation {
    /// The rotation in radians
    pub rotation: NotNan<Scalar>,
    /// The translation in the x and y-axis
    pub translation: (NotNan<Scalar>, NotNan<Scalar>),
}

impl DTransformation {
    #[must_use]
    pub fn new(rotation: Scalar, translation: (Scalar, Scalar)) -> Self {
        Self {
            rotation: NotNan::new(rotation).expect("rotation is NaN"),
            translation: (
                NotNan::new(translation.0).expect("translation.0 is NaN"),
                NotNan::new(translation.1).expect("translation.1 is NaN"),
            ),
        }
    }

    #[must_use]
    pub const fn empty() -> Self {
        const _0: NotNan<Scalar> = unsafe { NotNan::new_unchecked(0.0) };
        Self {
            rotation: _0,
            translation: (_0, _0),
        }
    }

    #[must_use]
    pub fn rotation(&self) -> Scalar {
        self.rotation.into()
    }

    #[must_use]
    pub fn translation(&self) -> (Scalar, Scalar) {
        (self.translation.0.into(), self.translation.1.into())
    }

    #[must_use]
    pub fn compose(&self) -> Transformation {
        self.into()
    }
}

impl<T> From<T> for DTransformation
where
    T: Borrow<Transformation>,
{
    fn from(t: T) -> Self {
        t.borrow().decompose()
    }
}

impl Display for DTransformation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "r: {:.3}°, t: ({:.3}, {:.3})",
            self.rotation.to_degrees(),
            self.translation.0.into_inner(),
            self.translation.1.into_inner()
        )
    }
}

/// Normalizes a rotation angle to the range [0, 2π).
#[must_use]
pub fn normalize_rotation(r: Scalar) -> Scalar {
    let normalized = r % (2.0 * PI);
    if normalized < 0.0 {
        normalized + 2.0 * PI
    } else {
        normalized
    }
}

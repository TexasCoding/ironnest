// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#![allow(clippy::inline_always)]

use crate::Scalar;
use std::borrow::Borrow;
use std::ops::{Add, Div, Mul, Sub};

use ordered_float::NotNan;

use crate::DTransformation;

#[derive(Clone, Debug)]
///The matrix form of [`DTransformation`].
///[read more](https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/geometry/geo-tran.html)
pub struct Transformation {
    matrix: [[NotNan<Scalar>; 3]; 3],
}

impl Transformation {
    /// Creates a transformation with no effect.
    #[must_use]
    pub const fn empty() -> Self {
        Self {
            matrix: EMPTY_MATRIX,
        }
    }

    #[must_use]
    pub fn from_translation((tx, ty): (Scalar, Scalar)) -> Self {
        Self {
            matrix: transl_m((tx, ty)),
        }
    }

    #[must_use]
    pub fn from_rotation(angle: Scalar) -> Self {
        Self {
            matrix: rot_m(angle),
        }
    }

    /// Applies a rotation to `self`.
    #[must_use]
    pub fn rotate(mut self, angle: Scalar) -> Self {
        self.matrix = dot_prod(&rot_m(angle), &self.matrix);
        self
    }

    /// Applies a translation to `self`.
    #[must_use]
    pub fn translate(mut self, (tx, ty): (Scalar, Scalar)) -> Self {
        self.matrix = dot_prod(&transl_m((tx, ty)), &self.matrix);
        self
    }

    /// Applies a translation followed by a rotation to `self`.
    #[must_use]
    pub fn rotate_translate(mut self, angle: Scalar, (tx, ty): (Scalar, Scalar)) -> Self {
        self.matrix = dot_prod(&rot_transl_m(angle, (tx, ty)), &self.matrix);
        self
    }

    /// Applies a rotation followed by a translation to `self`.
    #[must_use]
    pub fn translate_rotate(mut self, (tx, ty): (Scalar, Scalar), angle: Scalar) -> Self {
        self.matrix = dot_prod(&transl_rot_m((tx, ty), angle), &self.matrix);
        self
    }

    /// Applies `other` to `self`.
    #[must_use]
    pub fn transform(mut self, other: &Self) -> Self {
        self.matrix = dot_prod(&other.matrix, &self.matrix);
        self
    }

    #[must_use]
    pub fn transform_from_decomposed(self, other: &DTransformation) -> Self {
        self.rotate_translate(other.rotation(), other.translation())
    }

    /// Generates the transformation that undoes the effect of `self`.
    #[must_use]
    pub fn inverse(mut self) -> Self {
        self.matrix = inverse(&self.matrix);
        self
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.matrix == EMPTY_MATRIX
    }

    #[must_use]
    pub fn matrix(&self) -> &[[NotNan<Scalar>; 3]; 3] {
        &self.matrix
    }

    #[must_use]
    pub fn decompose(&self) -> DTransformation {
        let m = self.matrix();
        // DETERMINISM(ironnest): `f64::atan2` routes through the platform C libm (glibc/MSVCRT/
        // Darwin differ in ULPs). The pure-Rust `libm` crate is byte-identical across platforms.
        // IO/metadata path only — the placement path never decomposes.
        let angle = libm::atan2(m[1][0].into_inner(), m[0][0].into_inner());
        let (tx, ty) = (m[0][2].into_inner(), m[1][2].into_inner());
        DTransformation::new(angle, (tx, ty))
    }
}

impl<T> From<T> for Transformation
where
    T: Borrow<DTransformation>,
{
    fn from(dt: T) -> Self {
        let rot = dt.borrow().rotation();
        let transl = dt.borrow().translation();
        Self {
            matrix: rot_transl_m(rot, transl),
        }
    }
}

impl Default for Transformation {
    fn default() -> Self {
        Self::empty()
    }
}

const _0: NotNan<Scalar> = unsafe { NotNan::new_unchecked(0.0) };
const _1: NotNan<Scalar> = unsafe { NotNan::new_unchecked(1.0) };

const EMPTY_MATRIX: [[NotNan<Scalar>; 3]; 3] = [[_1, _0, _0], [_0, _1, _0], [_0, _0, _1]];

// DETERMINISM(ironnest): jagua built rotation matrices with `f64::sin_cos`, which routes through
// the platform C libm (glibc/MSVCRT/Darwin differ in the last ULPs) — non-portable. We route ALL
// rotation trig through the pure-Rust `libm` crate, which is byte-identical on every target.
//
// Trig DOES run on the placement path: `DTransformation::compose()` — called by `PlacedItem::new`
// for every placed item — builds a matrix via `rot_transl_m` below. Byte-identity is preserved
// *because* this goes through `libm`, NOT because trig is avoided. This MUST stay `libm::sincos` /
// `libm::atan2`; never "optimize" it back to std `f64::sin_cos`/`sin`/`cos` — that silently
// reintroduces a cross-platform hazard. (A future optimizer may build the cardinal matrices
// {0,90,180,270} directly — exact entries ∈ {0,±1} — for cut-realizability, but that is an
// exactness optimization independent of determinism, which `libm` already guarantees.) See CLAUDE.md.
fn rot_m(angle: Scalar) -> [[NotNan<Scalar>; 3]; 3] {
    let (sin, cos) = libm::sincos(angle);
    let cos = NotNan::new(cos).expect("cos is NaN");
    let sin = NotNan::new(sin).expect("sin is NaN");

    [[cos, -sin, _0], [sin, cos, _0], [_0, _0, _1]]
}

fn transl_m((tx, ty): (Scalar, Scalar)) -> [[NotNan<Scalar>; 3]; 3] {
    let h = NotNan::new(tx).expect("tx is NaN");
    let k = NotNan::new(ty).expect("ty is NaN");

    [[_1, _0, h], [_0, _1, k], [_0, _0, _1]]
}

//rotation followed by translation
fn rot_transl_m(angle: Scalar, (tx, ty): (Scalar, Scalar)) -> [[NotNan<Scalar>; 3]; 3] {
    let (sin, cos) = libm::sincos(angle);
    let cos = NotNan::new(cos).expect("cos is NaN");
    let sin = NotNan::new(sin).expect("sin is NaN");
    let h = NotNan::new(tx).expect("tx is NaN");
    let k = NotNan::new(ty).expect("ty is NaN");

    [[cos, -sin, h], [sin, cos, k], [_0, _0, _1]]
}

//translation followed by rotation
fn transl_rot_m((tx, ty): (Scalar, Scalar), angle: Scalar) -> [[NotNan<Scalar>; 3]; 3] {
    let (sin, cos) = libm::sincos(angle);
    let cos = NotNan::new(cos).expect("cos is NaN");
    let sin = NotNan::new(sin).expect("sin is NaN");
    let h = NotNan::new(tx).expect("tx is NaN");
    let k = NotNan::new(ty).expect("ty is NaN");

    [
        [cos, -sin, h * cos - k * sin],
        [sin, cos, h * sin + k * cos],
        [_0, _0, _1],
    ]
}

#[inline(always)]
fn dot_prod<T>(l: &[[T; 3]; 3], r: &[[T; 3]; 3]) -> [[T; 3]; 3]
where
    T: Add<Output = T> + Mul<Output = T> + Copy + Default,
{
    [
        [
            l[0][0] * r[0][0] + l[0][1] * r[1][0] + l[0][2] * r[2][0],
            l[0][0] * r[0][1] + l[0][1] * r[1][1] + l[0][2] * r[2][1],
            l[0][0] * r[0][2] + l[0][1] * r[1][2] + l[0][2] * r[2][2],
        ],
        [
            l[1][0] * r[0][0] + l[1][1] * r[1][0] + l[1][2] * r[2][0],
            l[1][0] * r[0][1] + l[1][1] * r[1][1] + l[1][2] * r[2][1],
            l[1][0] * r[0][2] + l[1][1] * r[1][2] + l[1][2] * r[2][2],
        ],
        [
            l[2][0] * r[0][0] + l[2][1] * r[1][0] + l[2][2] * r[2][0],
            l[2][0] * r[0][1] + l[2][1] * r[1][1] + l[2][2] * r[2][1],
            l[2][0] * r[0][2] + l[2][1] * r[1][2] + l[2][2] * r[2][2],
        ],
    ]
}

#[inline(always)]
fn inverse<T>(m: &[[T; 3]; 3]) -> [[T; 3]; 3]
where
    T: Add<Output = T> + Mul<Output = T> + Sub<Output = T> + Div<Output = T> + Copy,
{
    let det =
        m[0][0] * m[1][1] * m[2][2] + m[0][1] * m[1][2] * m[2][0] + m[0][2] * m[1][0] * m[2][1]
            - m[0][2] * m[1][1] * m[2][0]
            - m[0][1] * m[1][0] * m[2][2]
            - m[0][0] * m[1][2] * m[2][1];

    [
        [
            (m[1][1] * m[2][2] - m[1][2] * m[2][1]) / det,
            (m[0][2] * m[2][1] - m[0][1] * m[2][2]) / det,
            (m[0][1] * m[1][2] - m[0][2] * m[1][1]) / det,
        ],
        [
            (m[1][2] * m[2][0] - m[1][0] * m[2][2]) / det,
            (m[0][0] * m[2][2] - m[0][2] * m[2][0]) / det,
            (m[0][2] * m[1][0] - m[0][0] * m[1][2]) / det,
        ],
        [
            (m[1][0] * m[2][1] - m[1][1] * m[2][0]) / det,
            (m[0][1] * m[2][0] - m[0][0] * m[2][1]) / det,
            (m[0][0] * m[1][1] - m[0][1] * m[1][0]) / det,
        ],
    ]
}

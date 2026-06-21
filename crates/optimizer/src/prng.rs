// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! A small, self-contained, portable PRNG for the placement search.
//!
//! DETERMINISM(ironnest): the entire engine's randomness comes from here. We deliberately do NOT
//! depend on `rand`/`rand_pcg` — instead we vendor a fully-specified [PCG64][pcg] (XSL-RR 128/64)
//! so the exact `u64` stream produced by a given seed is **version-independent and byte-identical on
//! every platform, forever**. It is seeded explicitly (no `getrandom`, no thread RNG, no
//! `rand::random()` fallback). All arithmetic is wrapping integer / IEEE float — no transcendentals.
//!
//! [pcg]: https://www.pcg-random.org/  (M. E. O'Neill, PCG64 = XSL-RR over a 128-bit LCG)

use ironnest_geo::Scalar;

/// PCG64 (XSL-RR 128/64): a 128-bit LCG whose high bits drive an xorshift-low + random-rotate output.
#[derive(Clone, Debug)]
pub struct Prng {
    state: u128,
}

/// PCG64 LCG multiplier (O'Neill's canonical constant).
const PCG_MULT: u128 = 0x2360_ed05_1fc6_5da4_4385_df64_9fcc_f645;
/// A fixed odd increment (the "stream"). Constant across the engine — determinism does not need a
/// configurable stream, and a fixed one keeps the seed the single source of variation.
const PCG_INC: u128 = 0x5851_f42d_4c95_7f2d_1405_7b7e_f767_814f;

impl Prng {
    /// Creates a PRNG from a `u64` seed, expanding it to the 128-bit state with `SplitMix64` (so
    /// even low-entropy seeds like `0` or `1` start in a well-mixed state).
    #[must_use]
    pub fn seed_from_u64(seed: u64) -> Self {
        let mut sm = seed;
        let mut splitmix64 = || {
            sm = sm.wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut z = sm;
            z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            z ^ (z >> 31)
        };
        let lo = u128::from(splitmix64());
        let hi = u128::from(splitmix64());
        let mut prng = Prng {
            state: (hi << 64) | lo,
        };
        // advance once so the first output does not trivially expose the seed expansion
        prng.next_u64();
        prng
    }

    /// The core PCG64 step: advance the LCG, then emit the XSL-RR output of the new state.
    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_mul(PCG_MULT).wrapping_add(PCG_INC);
        // XSL-RR: xor the two 64-bit halves, then rotate by the top 6 bits of the state.
        #[allow(clippy::cast_possible_truncation)]
        let rot = (self.state >> 122) as u32;
        #[allow(clippy::cast_possible_truncation)]
        let xsl = ((self.state >> 64) ^ self.state) as u64;
        xsl.rotate_right(rot)
    }

    /// A uniform [`Scalar`] in `[0, 1)` with 53 bits of mantissa precision.
    /// Pure IEEE arithmetic on the raw bits → identical on every platform.
    pub fn unit(&mut self) -> Scalar {
        // top 53 bits → an integer in [0, 2^53), scaled to [0, 1). 2^53 is exactly representable.
        const SCALE: Scalar = 1.0 / 9_007_199_254_740_992.0; // 1 / 2^53
        // The shifted value is < 2^53, so the cast is exact (the lint can't prove the bound).
        #[allow(clippy::cast_precision_loss)]
        let mantissa = (self.next_u64() >> 11) as Scalar;
        mantissa * SCALE
    }

    /// A uniform [`Scalar`] in `[lo, hi)`. If `hi <= lo`, returns `lo`.
    pub fn range(&mut self, lo: Scalar, hi: Scalar) -> Scalar {
        if hi <= lo {
            lo
        } else {
            lo + (hi - lo) * self.unit()
        }
    }

    /// A uniform index in `0..n` (`n > 0`). Uses Lemire's unbiased bounded method so the result is
    /// exactly uniform AND a deterministic function of the stream (no rejection-order ambiguity for
    /// the small `n` we use, but unbiased regardless).
    // The `as u64` / `as usize` casts here are deliberate bit extractions (low half, high half) —
    // truncation is the intent, not a hazard.
    #[allow(clippy::cast_possible_truncation)]
    pub fn below(&mut self, n: usize) -> usize {
        debug_assert!(n > 0);
        let n = n as u64;
        // Lemire: multiply a full-width random by n, take the high 64 bits; reject the low zone.
        let mut m = u128::from(self.next_u64()) * u128::from(n);
        let mut low = m as u64;
        if low < n {
            // threshold = (2^64 - n) % n, computed without 2^64 overflow
            let threshold = n.wrapping_neg() % n;
            while low < threshold {
                m = u128::from(self.next_u64()) * u128::from(n);
                low = m as u64;
            }
        }
        (m >> 64) as usize
    }

    /// In-place Fisher–Yates shuffle, drawing swap indices from the stream. Deterministic for a given
    /// stream and slice length — used to randomize the order colliding parts are moved.
    pub fn shuffle<T>(&mut self, slice: &mut [T]) {
        // Iterate high→low, swapping each element with a uniformly-chosen earlier-or-equal index.
        for i in (1..slice.len()).rev() {
            let j = self.below(i + 1);
            slice.swap(i, j);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn same_seed_same_stream() {
        let mut a = Prng::seed_from_u64(42);
        let mut b = Prng::seed_from_u64(42);
        for _ in 0..1000 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn different_seeds_diverge() {
        let mut a = Prng::seed_from_u64(1);
        let mut b = Prng::seed_from_u64(2);
        assert_ne!(a.next_u64(), b.next_u64());
    }

    #[test]
    fn unit_in_range() {
        let mut p = Prng::seed_from_u64(7);
        for _ in 0..10_000 {
            let u = p.unit();
            assert!((0.0..1.0).contains(&u));
        }
    }

    #[test]
    fn below_in_range_and_covers() {
        let mut p = Prng::seed_from_u64(9);
        let mut seen = [false; 4];
        for _ in 0..1000 {
            let i = p.below(4);
            assert!(i < 4);
            seen[i] = true;
        }
        assert!(seen.iter().all(|&s| s), "all 4 indices should appear");
    }

    #[test]
    fn shuffle_is_deterministic_and_a_permutation() {
        let mut a: Vec<u32> = (0..50).collect();
        let mut b = a.clone();
        Prng::seed_from_u64(123).shuffle(&mut a);
        Prng::seed_from_u64(123).shuffle(&mut b);
        assert_eq!(a, b, "same seed → same permutation");

        // The result is a permutation of the input (no elements lost/duplicated).
        let mut sorted = a.clone();
        sorted.sort_unstable();
        assert_eq!(sorted, (0..50).collect::<Vec<_>>());

        // A different seed gives a different order (overwhelmingly likely for 50 elements).
        let mut c: Vec<u32> = (0..50).collect();
        Prng::seed_from_u64(124).shuffle(&mut c);
        assert_ne!(a, c);
    }
}

# ironnest

A **deterministic, embeddable, true-shape 2D nesting engine** for irregular parts — built in
Rust, exposed to Python via a [PyO3](https://pyo3.rs) wheel.

`ironnest` is a fork-and-extend of the peer-reviewed
[jagua-rs](https://github.com/JeroenGar/jagua-rs) Collision Detection Engine (MPL-2.0), pushed to
**f64** and made **cross-platform byte-reproducible**, with our own placement optimizer on top.

## Why it exists

Existing irregular nesters force a bad trade:

| | true-shape density | irregular remnants + holes | **deterministic** | embeddable library |
|---|---|---|---|---|
| Deepnest | ✅ | ✅ | ❌ (GA, non-deterministic) | ❌ (an Electron app) |
| SVGnest | ✅ | ❌ | ❌ | ❌ |
| libnest2d / pynest2d | ✅ | ❌ (box bins only) | partial | ✅ |
| jagua-rs / sparrow | ✅ | ✅ | ❌ (wall-clock-terminated) | partial (Rust only) |
| **ironnest** | ✅ | ✅ | ✅ **byte-identical re-nest** | ✅ **Rust + Python wheel** |

The differentiator is **not** the collision-detection geometry (that's a solved, peer-reviewed
problem we stand on jagua-rs for) — it's **determinism + plasma-grade auditability + a clean
embeddable API**. ironnest's first consumer is a plasma CAD/CAM tool where a re-nest *must*
reproduce a byte-identical cut program for the machine audit trail.

## Status

🚧 **Early development.** A working end-to-end deterministic nester exists.

- **Phase 1 (2026-06-20):** the jagua-rs Collision Detection Engine forked to f64 — `crates/geo` +
  `crates/cde`, ported `f32`→`f64`, determinism-scrubbed, green.
- **Phase 2a (2026-06-21):** `crates/optimizer` — a deterministic constructive nester (Left-Bottom-
  Fill + drop-on-place + slide compaction; self-contained PCG64 PRNG; fixed budget; configurable
  discrete rotations). **87–99% density on rectangular parts** (beats the 85–90% target), ~65% on
  irregular. Public `nest(...)`; in-process determinism golden holds.

**Next:** the overlap-minimization **separation search** (irregular-part density), the cross-platform
determinism golden, and the PyO3 wheel. See
[`docs/00-ironnest-architecture-and-plan.md`](docs/00-ironnest-architecture-and-plan.md) and the
optimizer design / separation-search plan in
[`docs/02-optimizer-and-separation-search.md`](docs/02-optimizer-and-separation-search.md).

## License

**MPL-2.0** across the whole repo — uniform with the forked jagua-rs core (file-scoped copyleft;
our modifications are published). See [`docs/00-…`](docs/00-ironnest-architecture-and-plan.md) §12.

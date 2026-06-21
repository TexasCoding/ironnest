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

🚧 **Planning / pre-spike.** See [`docs/00-ironnest-architecture-and-plan.md`](docs/00-ironnest-architecture-and-plan.md).

## License

**MPL-2.0** across the whole repo — uniform with the forked jagua-rs core (file-scoped copyleft;
our modifications are published). See [`docs/00-…`](docs/00-ironnest-architecture-and-plan.md) §12.

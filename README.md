# ironnest

[![PyPI](https://img.shields.io/pypi/v/ironnest)](https://pypi.org/project/ironnest/)
[![Python](https://img.shields.io/pypi/pyversions/ironnest)](https://pypi.org/project/ironnest/)
[![CI](https://github.com/TexasCoding/ironnest/actions/workflows/ci.yml/badge.svg)](https://github.com/TexasCoding/ironnest/actions/workflows/ci.yml)
[![License: MPL-2.0](https://img.shields.io/badge/license-MPL--2.0-blue.svg)](https://mozilla.org/MPL/2.0/)

A **deterministic, embeddable, true-shape 2D nesting engine** for irregular parts — built in
Rust, exposed to Python via a [PyO3](https://pyo3.rs) wheel.

`ironnest` is a fork-and-extend of the peer-reviewed
[jagua-rs](https://github.com/JeroenGar/jagua-rs) Collision Detection Engine (MPL-2.0), pushed to
**f64** and made **cross-platform byte-reproducible**, with our own placement optimizer on top.

## Install

```sh
pip install ironnest
```

One `abi3` wheel per platform — **macOS-arm64 / Windows-x64 / linux-x64** — serves CPython **3.13+**.

## Quickstart

```python
import ironnest

# one outline per part type, in item-local coordinates
items = [[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]]   # a 10×10 square
container = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)]  # 100×100 sheet

placements, unplaced = ironnest.nest(
    items,
    qty=[25],                         # demand per part type
    container=container,
    holes=[],                         # interior keep-out zones ([] = none)
    min_sep=0.0,                      # part-to-part gap; 0.0 = byte-identical across platforms
    rotations=[0.0, 90.0, 180.0, 270.0],
    seed=1,                           # explicit — no entropy fallback, ever
    budget=2000,                      # fixed sample budget, never a wall clock
)
# placements: list[(item, x, y, rotation_deg)], where placed = Rot(rotation_deg)·original + (x, y)
# unplaced:   list[int] — the item-type index of each instance that did not fit
```

- **`holes`** are interior keep-out polygons on the sheet — nest *around* voids, or *inside* a part.
- **`nest_multi(items, qty, sheets, min_sep, rotations, seed, budget)`** packs across several sheets
  in order, spilling leftover demand to the next; each sheet is an `(outline, holes)` pair.

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

## Determinism is the product

The same inputs produce **byte-identical** placements — and therefore a byte-identical downstream cut
program — on **every platform we ship** (macOS-arm64 / Windows-x64 / linux-x64). This is proven by a
**cross-platform CI golden**, not assumed: `f64` throughout, all trig routed through the pinned
pure-Rust [`libm`](https://crates.io/crates/libm), a vendored deterministic min-separation offsetter,
a self-contained seeded PCG64 (no `rand`), and no threads or wall-clock on any placement-deciding path.

The engine is a pure **placement oracle**: in go polygons + an irregular container (boundary +
holes/keep-out zones) + a min-separation + an allowed-rotation set + a seed + an iteration budget;
out come `(item, x, y, rotation)` tuples. It knows **nothing** about kerf, lead-ins, pierces, cut
sequencing, or G-code — those live in the consuming CAD/CAM application, which re-validates every
layout.

## Status

A complete, end-to-end deterministic nester, shipping as a PyPI wheel.

- **Phase 1 — fork → f64.** The jagua-rs Collision Detection Engine forked to `f64` (`crates/geo` +
  `crates/cde`), determinism-scrubbed.
- **Phase 2 — optimizer.** A deterministic constructive nester (Left-Bottom-Fill + drop-on-place +
  slide compaction) plus a sparrow-style Guided-Local-Search **separation search** that discovers
  interlocks greedy construction cannot. **92–99% density on rectangular parts** (beats the 85–90%
  target); irregular parts ~79% (pentagon) / ~75% (concave sheet) — a structural ceiling
  (see [`docs/02`](docs/02-optimizer-and-separation-search.md) §11).
- **Phase 3 — determinism harness.** A cross-platform CI golden gates every change on
  macOS-arm64 / Windows-x64 / linux-x64.
- **Phase 4 — Python wheel.** `import ironnest` via a PyO3 `abi3-py313` wheel, built + published by
  maturin/cibuildwheel.
- **Phase 6 — interior voids + multi-sheet.** `nest(holes=…)` keep-out zones (nesting inside parts)
  and `nest_multi(sheets=…)`, with a deterministic min-separation offsetter.

**Next:** Phase 5 — integration into the first consumer (a plasma CAD/CAM application). See
[`docs/00-ironnest-architecture-and-plan.md`](docs/00-ironnest-architecture-and-plan.md) (the plan)
and [`docs/02-optimizer-and-separation-search.md`](docs/02-optimizer-and-separation-search.md) (the
optimizer + separation-search design).

## Building from source

```sh
cargo build --release --locked   # the Rust core
cargo test                       # incl. the cross-platform determinism golden
cd crates/py && maturin develop --release   # build + install the wheel into the active venv
```

The toolchain is pinned via `rust-toolchain.toml`; all dependencies are pure Rust (no C, no
`build.rs`) for clean cross-platform wheels.

## License

**MPL-2.0** across the whole repo — uniform with the forked jagua-rs core (file-scoped copyleft;
our modifications are published). See [`docs/00-…`](docs/00-ironnest-architecture-and-plan.md) §12.

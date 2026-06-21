# ironnest

**Deterministic, embeddable, true-shape 2D nesting engine for irregular parts** — a Rust core
(a [jagua-rs](https://github.com/JeroenGar/jagua-rs) fork at f64) exposed to Python via PyO3.

The differentiator is **determinism**: the same inputs produce **byte-identical** placements on every
platform (macOS-arm64 / Windows-x64 / linux-x64), proven by a cross-platform CI golden — so a re-nest
reproduces a byte-identical downstream cut program for a machine audit trail.

## Install

```sh
pip install ironnest
```

One `abi3` wheel per platform (macOS-arm64 / Windows-x64 / linux-x64) serves CPython **3.13+**.

## Usage

```python
import ironnest

# one outline per part type, in item-local coords
items = [[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]]
container = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)]

placements, unplaced = ironnest.nest(
    items,
    qty=[25],
    container=container,
    holes=[],                        # interior keep-out zones ([] = none)
    min_sep=0.0,                     # 0.0 keeps output byte-identical across platforms
    rotations=[0.0, 90.0, 180.0, 270.0],
    seed=1,                          # explicit — no entropy fallback, ever
    budget=2000,                     # a fixed sample budget, never a wall clock
)
# placements: list[(item, x, y, rotation_deg)] with placed = Rot(rotation_deg)·original + (x, y)
# unplaced:   list[int] (item-type index per instance that did not fit)
```

`holes` are interior keep-out polygons on the sheet (nesting *around* voids / inside parts). To pack
across several sheets in order, use `nest_multi(items, qty, sheets, min_sep, rotations, seed, budget)`
where each sheet is an `(outline, holes)` pair.

The engine is a pure **placement oracle**: it knows nothing about kerf, lead-ins, pierces, cut
sequencing, or G-code — those belong to the consuming CAD/CAM application, which re-validates every
layout. See the [project repository](https://github.com/TexasCoding/ironnest) for the full design.

Licensed under **MPL-2.0**.

# jagua-rs source verification (fork grounding)

Source-verified read of `JeroenGar/jagua-rs` @ commit `43e8137` (workspace member declares
`version = "0.7.2"`, `edition = "2024"`, `license = "MPL-2.0"`). This is the evidence behind the
fork decisions in [`00-ironnest-architecture-and-plan.md`](00-ironnest-architecture-and-plan.md).
Workspace members: `["jagua-rs"` (library), `"lbf"` (the reference Left-Bottom-Fill optimizer +
WASM demo — **not** forked; read for the call pattern).

## A. Float type — HARD `f32`, no `fsize` flag → f64 is a typed rewrite

- **No float type alias and no Cargo feature selects f32/f64.** The `fsize` alias older jagua had
  was **removed**. The scalar is baked into the core primitive: `geometry/primitives/point.rs:8`
  → `pub struct Point(pub f32, pub f32);`.
- **309 occurrences of `f32`** across **41 files**; only 3 incidental `f64` (the `geo_buffer`
  upcast in `shape_modification.rs:350,356`). `NotNan<f32>`/`OrderedFloat<f32>` appear 21× across
  7 files; the transform matrix is `[[NotNan<f32>;3];3]` (`transformation.rs:14`).
- `[features]` = `spp`, `bpp`, `mspp` (problem variants only) — none touch float width.

**→ Effort:** mechanical token sweep across ~41 files (`f32`→`f64`, `NotNan<f32>`→`NotNan<f64>`,
`std::f32::consts::PI`→f64, `f32::MAX/EPSILON/…`). Real hotspots are few: `Point` + its
`to_bits()` hash (`point.rs:61-62`), the matrix constants + `EMPTY_MATRIX`
(`transformation.rs:125-128`), `FPA(pub f32)` (`util/fpa.rs:7`), and the `geo_buffer` bridge
(already f64 internally — the f32 round-trip at `shape_modification.rs:374` *disappears*, a net
win). ~1–2 focused days incl. test fixup. **Action: define `pub type Scalar = f64;` up front** so
future width changes are a flag.

## B. Fork map (library `jagua-rs/src`, 8,168 LOC; minimal fork ≈ 5,800 LOC)

| Area | Key files | ~LOC | Fork? |
|---|---|---|---|
| Geometry primitives | `geometry/primitives/*` + `transformation`, `d_transformation`, `original_shape`, **`shape_modification.rs` (550 — offsetting + min-sep)**, `fail_fast/*` | ~3,040 | ✅ all (ex-SVG) |
| Collision Detection Engine | `collision_detection/cd_engine.rs` (365) + `quadtree/*` + `hazards/*` | ~1,420 | ✅ all |
| Entities | `entities/`: `layout`, `container` (holes = quality-0 zones), `item`, `placed_item`, `instance` | ~453 | ✅ all |
| IO + ext JSON repr | `io/import.rs` (261, drives `min_item_separation`), `io/ext_repr.rs` (121), `io/export.rs` (46) | ~430 | ✅ (skip `io/svg/` ~750) |
| Problems | `probs/bpp/*` (problem 270, instance, import, bin/solution) | ~480 | ✅ bpp only (drop spp/mspp) |
| util | `util/fpa.rs` (38) + `util/assertions.rs` (279, debug-only) | ~320 | ✅ fpa; assertions optional |

`lbf/` (the reference optimizer) is **replaced** by `crates/optimizer`; it also owns the project's
only RNG (`rand`/`getrandom` are `lbf`/wasm deps) — we own seeding in our optimizer.

## C. Determinism hazards — only TWO real cross-platform risks

| Category | Where | Hot path? | Mitigation |
|---|---|---|---|
| **Transcendentals `sin_cos` / `atan2`** | `sin_cos`: `transformation.rs:131,148,159,170` (rotation matrix). `atan2`: `transformation.rs:100` (decompose, IO-only) | **YES** for `sin_cos` when items rotate | **Discrete `0/90/180/270` ⇒ hardcode exact matrices (entries ∈ {0,±1}) — no `sin_cos` at all.** For any continuous rotation later, route through the pure-Rust `libm` crate. |
| **`geo-buffer::buffer_polygon_rounded`** | `shape_modification.rs:356`, via `offset_shape`←`OriginalShape::generate`; driven by `Importer(min_item_separation)` (`offset = sep/2`) + Inflate/Deflate of holes/quality-zones | **No (import/preproc)** but its output feeds every later collision query | **Replace with our own deterministic offsetter** (miter, or pinned-resolution arc) **or vendor+pin `geo-buffer` and verify bit-stability in the x-platform CI golden.** Already f64. **⚠ FORK-AUDIT (2026-06-20): `geo-buffer 0.2` calls *std* `f64::sin`/`f64::cos` internally (`ray::rotate_by`) → registry-pinning is NOT sufficient.** Vendor+swap-to-`libm` or replace; the residual is live only when `min_item_separation != 0`. |
| `sqrt` / `powi` | `sqrt`: many (point/circle/rect/edge/poly/piers); `powi`: 33 sites | distance/sep path | **SAFE.** `sqrt` is IEEE-754 correctly-rounded (deterministic across platforms); `powi` is plain multiplication. No action. |
| `rayon` / `par_iter` | `*/io/import.rs` only | **No** — each `par_iter` is immediately `.sort_by_key(id)`; per-item floats are independent (no parallel reduction) | Low risk; optionally swap to sequential `iter()`. No `par_sort`/`par_bridge`. |
| `HashMap`/`HashSet` | `util/assertions.rs` only (`HashSet`, debug-only) | **No** | Live path uses `slotmap`/`SecondaryMap` + `BTreeMap`/`BTreeSet`. No `HashMap` in the library. Optionally swap the assertion set to `BTreeSet`. |
| `Instant` / wall-clock | `lib.rs:36`; `*/problem.rs` `time_stamp`; `export.rs` epoch | **No** — solution metadata only | Set to a fixed value / drop for reproducible output. |
| `rand` / RNG | **none in the library** | — | We own seeding (seeded portable PRNG, e.g. `rand_pcg`/ChaCha). |

**Net:** the CDE/quadtree/entities core is already deterministic; the only cross-platform work is
killing `sin_cos` (free with discrete rotations) and taming `geo-buffer`.

## D. Public API the optimizer builds on (signatures abridged)

**`CDEngine`** (`collision_detection/cd_engine.rs`): `new(bbox, static_hazards, config)`,
`register_hazard`, `deregister_hazard_by_entity/by_key`, `save()->CDESnapshot`, `restore`,
`detect_poly_collision(shape, filter)->bool`, `detect_surrogate_collision`,
`detect_containment_collision`, `collect_poly_collisions(shape, collector)`,
`collect_surrogate_collisions`, `bbox()`. `CDEConfig { quadtree_depth, cd_threshold,
item_surrogate_config }`.

**`Layout`** (`entities/layout.rs`): `new(container)`, `from_snapshot`, `save()->LayoutSnapshot`,
`restore`, `place_item(&Item, DTransformation)->PItemKey`, `remove_item(PItemKey)->PlacedItem`,
`density(&Instance)`, `cde()->&CDEngine`, `is_feasible()->bool`.

**`BPProblem`** (`probs/bpp/entities/problem.rs`): `new(BPInstance)`,
`place_item(BPPlacement)->(LayKey, PItemKey)`, `remove_item(LayKey, PItemKey)->BPPlacement`,
`save()->BPSolution`, `restore(&BPSolution)->bool`, `density()`, `bin_cost()`, `n_placed_items()`.
`BPPlacement { layout_id: BPLayoutType, item_id, d_transf }`; `BPLayoutType = Open(LayKey) |
Closed{bin_id}`.

**Types:** `HazardEntity = PlacedItem{id,dt,pk} | Exterior | Hole{idx} |
InferiorQualityZone{quality,idx}`; `Container { …, quality_zones: [Option<InferiorQualityZone>;
N_QUALITIES=10] }` where **quality 0 == a hole**; `Item { …, allowed_rotation: RotationRange,
min_quality }`; `Importer::new(cde_config, simplify_tolerance, min_item_separation,
narrow_concavity_cutoff)` — **`min_item_separation` is the min-sep knob**. Ext JSON repr:
`ExtShape = Rectangle | SimplePolygon | Polygon{outer,inner} | MultiPolygon`. **`import_item`
warns+ignores holes on items** (`import.rs:62`) — holes live on containers/quality-zones, not items.

## E. Deps / edition / cross-platform

- **Edition 2024, no declared MSRV** (needs Rust ≥ 1.85). `rust-toolchain.toml` → stable + `rust-src,
  rustfmt, clippy` + `wasm32` target.
- **All deps pure Rust — NO C/system deps, NO `build.rs`, NO `-sys`/`cc`/`cmake`/`bindgen`.**
  Clean cross-platform wheels.
- Notable: `slotmap` (Zlib, deterministic keyed storage — core), `ordered-float` (MIT),
  `geo-types`/**`geo-buffer 0.2`** (MIT — the offset hazard), `rayon` (import only), `serde`,
  `ndarray`, `itertools`, `anyhow`, `log`, `web-time`. `rand_distr` is imported only for num-traits
  (no RNG). `lbf` adds `rand`, `clap`, `criterion`, wasm — not needed.

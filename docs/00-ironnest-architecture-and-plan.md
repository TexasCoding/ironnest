# ironnest — Architecture & Implementation Plan

**Status:** Planning / pre-spike · **Owner:** TexasCoding · **Started:** 2026-06-20
**Consumer:** `drawing_and_gcode` (plasma CAD/CAM) — tracked there by the narrowed issue #258.

> This is the engine's source of truth. The deep design lives here, in `ironnest`. The
> *consumer-side* integration (remove the in-house Python nesters, install the wheel, wire it
> through the nest pipeline) is tracked in `drawing_and_gcode` #258.

---

## 1. Vision & non-goals

**ironnest is a deterministic, embeddable, true-shape 2D nesting engine for irregular parts.**
A fork-and-extend of [jagua-rs](https://github.com/JeroenGar/jagua-rs)'s Collision Detection
Engine (CDE), pushed to **f64**, made **cross-platform byte-reproducible**, with **our own
placement optimizer** on top, shipped as a Rust crate **and** a Python (PyO3) wheel.

**What ironnest IS:**
- A pure **placement oracle**: in = polygons + an irregular container (with holes / keepout
  zones) + a minimum separation + an allowed-rotation set + a seed + an iteration budget;
  out = a list of `(part, x, y, rotation)` placements. Nothing else.
- **Deterministic by construction** — same inputs → byte-identical placements, *on every
  platform we ship* (macOS-arm64 dev == Windows-x64 prod), validated by a cross-platform CI
  golden.
- **True-shape**: it packs the actual part outline, not its bounding box.
- **Irregular-bin native**: it nests onto remnants/offcuts (arbitrary boundary) and around
  holes / defect zones — and packs small parts *inside* large parts' holes (interior-void).

**What ironnest is NOT (non-goals):**
- It is **not** plasma-aware. It knows nothing about kerf, lead-ins, pierces, cut sequencing,
  G-code, or the `.CNC` audit banner. Those live in the *consumer* and re-validate every
  layout. ironnest never invents or trusts a machine number.
- It is **not** a GUI or an app (the Deepnest mistake). It is a library.
- It is **not** a from-scratch computational-geometry research project. We stand on jagua-rs's
  peer-reviewed CDE for collision geometry; our value is the optimizer + determinism + the API.

---

## 2. Why this shape (the decisions, with rationale)

These were settled in the #258 planning session (see `drawing_and_gcode` memory
`nesting-engine-direction.md` for the full history):

1. **Build the optimizer; stand on jagua-rs for the geometry.** "Better than Deepnest/SVGnest"
   is won in the **placement search** and in **determinism**, not in the NFP/collision math.
   jagua-rs's CDE is peer-reviewed (INFORMS J. Computing, DOI 10.1287/ijoc.2024.1025) and its
   sibling optimizer `sparrow` is current SoTA (beats prior art by 1–4% density) — *because* of
   the CDE, which proves the "your own optimizer on this CDE" pattern is the supported design.
   Reimplementing that core from scratch would be a year of subtle bugs and likely *slower*.

2. **Fork it, don't just depend on it.** Owning the source is what makes the next two decisions
   *enforceable* rather than a fight with upstream:
   - **f64** for numerical robustness (a near-tangent "just fits vs overlaps" decision won't
     flip on f32's ~7 significant digits; grid-snapping is clean).
   - **Cross-platform determinism** requires controlling transcendentals, threading, and hash
     iteration *throughout* the geometry core — only possible if we own it.
   MPL-2.0 permits this; we publish our modifications to jagua's files (which we'd want anyway
   for an open engine). The optimizer (our value) sits in *new* crates, behind the CDE
   interface, insulated from the fork.

3. **Standalone repo, consumed as a wheel.** A clean library API **forces** the oracle boundary
   (the engine can't leak plasma concepts), it's reusable/open-sourceable, and it keeps the Rust
   toolchain + wheel CI out of the consumer app. `drawing_and_gcode` depends on the published
   wheel (a `path`/editable dep during co-development).

4. **This IS the nester — not optional, not gated.** The consumer is in development with no
   legacy and no break-risk, so there is no off-by-default / fallback / parity machinery. The
   old Python engines (`shapely_blf`, `rectpack`, the stale CLI `jagua.py` stub) are **removed**.
   ironnest is a hard dependency, like shapely.

5. **Determinism is a hard requirement, not legacy baggage.** A plasma re-nest must reproduce a
   byte-identical cut program for the machine audit trail (the consumer's Cardinal rule). It is
   *also* ironnest's headline differentiator — nobody else ships a deterministic irregular nester.

---

## 3. Architecture

```
┌─────────────────────────────────────────── ironnest (this repo) ──────────────────────────────┐
│                                                                                                 │
│  crates/geo        forked jagua-rs geometry primitives + shape modification  (MPL-2.0)          │
│  crates/cde        forked jagua-rs collision-detection engine + entities      (MPL-2.0)         │
│       │            (f64; rayon stripped from hot path; BTree/slotmap; pure-Rust libm)           │
│       ▼                                                                                          │
│  crates/optimizer  OUR deterministic placement search  (new — our license)                      │
│       │            fixed seed · iteration/sample budget (NO wall-clock) · single canonical      │
│       │            worker · stable sort + total-order tie-break · discrete rotations             │
│       │            (lifts sparrow's separator/GLS *math*, MIT — does not link sparrow)           │
│       ▼                                                                                          │
│  crates/ironnest   the public Rust API:  nest(items, container, min_sep, rots, seed, budget)     │
│       │                                  -> Vec<Placement{item, x, y, rotation}>                 │
│       ▼                                                                                          │
│  crates/py         PyO3 binding  ──build──>  ironnest  (abi3-py313 wheel on PyPI)                │
│                                                                                                  │
│  benches/   density + determinism benchmark suite     tests/  cross-platform golden placements   │
│  .github/   cibuildwheel (Win-x64 + macOS-arm64 + linux) -> PyPI  ·  the X-PLATFORM golden gate  │
└─────────────────────────────────────────────────────────────────────────────────────────────────┘
                                              │  pip install ironnest
                                              ▼
┌──────────────────────────────── drawing_and_gcode (consumer) ─────────────────────────────────┐
│  dxfgcode/nesting/engine.py    Plate · SpacingRules · effective_part_gap · Placement · NestResult│
│  dxfgcode/nesting/<adapter>.py marshals Part.outer / Plate.usable_polygon()⊖edge_margin / holes  │
│                                → ironnest.nest(...) → NestResult                                  │
│  ── SAFETY SPINE (unchanged, engine-agnostic) ──                                                 │
│  ManualNest.violations()  re-erodes by the SAME edge_margin, checks the SAME effective_part_gap  │
│  sheet.py::run_sheet      kerf-comp · leads/pierce · cut sequencing · UNVERIFIED banner          │
│       → a denser-but-ILLEGAL layout is REFUSED, not cut.  audit.py records engine{rev,seed,…}     │
└───────────────────────────────────────────────────────────────────────────────────────────────┘
```

(The exact crate split — one workspace crate vs `geo`/`cde`/`optimizer`/`ironnest`/`py` — is
finalized after the source-verification report lands; see §5.)

---

## 4. The fork strategy

**Take from jagua-rs (fork, keep MPL-2.0 headers, publish our changes):**
- geometry primitives (`SPolygon`, `Point`, `Rect`, `Circle`, transforms),
- the CDE (quadtree + hazard model + surrogate fail-fast + `detect/collect_poly_collision`,
  `save`/`restore`),
- the entity model (`Item`, `Container` with holes/quality-zones, `Layout`, `Instance`),
- the importer's shape modification (min-separation inflation, simplification),
- the bin-packing problem driver (`place_item`/`remove_item`/`save`/`restore`).

**Change in the fork** (grounded by the source verification — see
[`01-jagua-source-verification.md`](01-jagua-source-verification.md)):
- **f64 — a typed rewrite, not a flag.** Verified: jagua 0.7.2 is hard `f32` across ~41 files
  (the old `fsize` alias was removed; no Cargo feature selects width). The sweep is mechanical
  (`f32`→`f64`, `NotNan<f32>`→`NotNan<f64>`, the `*::consts::PI`/`f32::*` constants), ~1–2 days
  with test fixup; the `geo_buffer` bridge already runs f64 internally so its boundary cast
  *disappears*. **Action: define `pub type Scalar = f64;` up front** so future width changes ARE a
  flag (the thing jagua removed).
- **Determinism scrub — only TWO real cross-platform hazards** (the core is otherwise already
  deterministic: `slotmap`+`BTree`, no library RNG, `rayon` is import-only-then-resorted, `Instant`
  is metadata): **(1)** `sin_cos` on the rotation path (`transformation.rs:131,148,159,170`) —
  killed *entirely* by hardcoding the exact `0/90/180/270` matrices (entries ∈ {0,±1}); **(2)**
  `geo-buffer::buffer_polygon_rounded`, the min-separation offset — replace with our own
  deterministic offsetter (miter / pinned-resolution arc) or vendor+pin it and prove bit-stability
  in the x-platform CI golden. (`sqrt` is IEEE correctly-rounded → safe; `powi` is multiplication →
  safe.)

**Do NOT take:** `sparrow` itself (binary-only, no `[lib]`, wall-clock-terminated, strip-packing
objective). We **lift its separator/guided-local-search math** (MIT) into `crates/optimizer`
under our own deterministic loop. We also don't need jagua's `lbf` binary or its WASM demo.

**Build our own (new crates, our license):** the optimizer, the public `nest()` API, the PyO3
binding, the benches, the determinism harness.

---

## 5. The optimizer (where "better than others" is won)

The optimizer is the brain; the CDE only answers "does this placement collide?". Design intent:

- **Construction + improvement.** Start from a deterministic constructive placement (e.g. a
  best-fit / bottom-left over the irregular container using the CDE), then improve with a
  **separation/overlap-minimization local search** lifted from sparrow's math — but driven by a
  **fixed iteration/sample budget**, never a wall clock.
- **Discrete rotations `{0, 90, 180, 270}`.** At these angles cos/sin ∈ {0, ±1}, so rotating a
  polygon is an *exact* coordinate swap/negate — **zero trig, zero rounding error**. This is both
  the right call for downstream cut realizability *and* the single biggest determinism enabler.
  (Continuous rotation is a possible future flag, but it reintroduces trig and must then route
  through pure-Rust `libm`.)
- **Irregular bins + holes are native.** Remnants are the container boundary; holes / defect
  keepouts are quality-0 zones the CDE already avoids. **Interior-void (parts inside a placed
  part's hole)** is modeled as *container* geometry (a placed large part's hole becomes a
  quality-0 region for a subsequent pass) — **not** an item hole (jagua 0.7.2 silently drops item
  holes; confirm in the fork).
- **Multi-bin / spill.** Pack a cut list across multiple sheets remnant-first (mirrors the
  consumer's existing `nest_across_sheets` intent) — but that orchestration can stay consumer-side
  initially; ironnest's first cut targets single-container nesting well.

Target: **~85–90%** utilization on the consumer's committed mixed benchmark, vs the current
pure-Python `ShapelyBLFNester` ~75.5% and the rectpack bbox floor 67.3%.

---

## 6. Determinism design (the contract: cross-platform byte-identical)

The promise: **the same nest produces byte-identical placements — and therefore a byte-identical
`.CNC` — on macOS-arm64 (dev) and Windows-x64 (prod).** This is achievable with discipline; it is
the standard "deterministic lockstep" technique. The mechanism is **not** "use f64" — it is
controlling the four real divergence sources:

| # | Source | Verified location / hot path? | Mitigation (enforced in the fork) |
|---|---|---|---|
| **1** | **`sin_cos` on the rotation path** | `transformation.rs:131,148,159,170` — **hot** when items rotate | **Hardcode the exact `0/90/180/270` matrices (entries ∈ {0,±1}) ⇒ no `sin_cos` at all.** Continuous rotation later ⇒ route through the pure-Rust `libm` crate. |
| **2** | **`geo-buffer::buffer_polygon_rounded`** (min-sep offset, arc joins) | `shape_modification.rs:356` — import/preproc, but its output feeds every collision query | **Replace with our own deterministic offsetter** (miter / pinned-resolution arc) **or vendor+pin `geo-buffer`** and prove bit-stability in the x-platform CI golden. |
| 3 | **Threads** (`rayon`) | `io/import.rs` only — each `par_iter` is immediately `.sort_by_key(id)`; no parallel float reduction | Low risk (output already re-sorted); optionally make import sequential. No `par_sort`/`par_bridge` anywhere. |
| 4 | **`HashMap`/`HashSet`** | `util/assertions.rs` only (`HashSet`, **debug-only**) | Live path is `slotmap`/`BTree` (deterministic). No `HashMap` in the library. Optionally swap the assertion set to `BTreeSet`. |
| 5 | **`Instant`/wall-clock** | `*/problem.rs` `time_stamp`, `export.rs` epoch — **metadata only, never a placement input** | Fix to a constant / drop for reproducible output. |
| 6 | **RNG** | **none in the library** (lives in `lbf`, which we don't fork) | We own seeding: a seeded portable PRNG (`rand_pcg`/ChaCha), explicit seed, no `rand::random()` fallback. |
| 7 | FMA / fast-math | — | Never call `f64::mul_add`; don't enable fast-math (Rust/LLVM default is already safe). |
| — | basic `+ - * / sqrt`, `powi` | — | *Safe — IEEE-754 correctly rounded (`sqrt`) / plain multiplication (`powi`); already bit-identical across platforms.* Default round-to-nearest. |

**Why f64 then?** Robustness, not determinism: f64's ~15–16 digits keep near-tangent collision
decisions from flipping, and make a fixed grid-snap (e.g. 1e-6") clean. f32 is *also*
cross-platform deterministic for basic ops, but its precision makes "just fits" brittle.

**Proof, not hope:** a CI job runs one representative nest on a **macOS-arm64 runner AND a
Windows-x64 runner** and asserts **byte-identical** placement output (and a `.CNC`-level sha256
in the consumer). Any divergence fails CI loudly. That is what makes "dev == prod" a guarantee
instead of an aspiration.

**Test pyramid:**
1. same-process run-to-run equality of rounded placements,
2. **cross-subprocess** byte-diff (catches per-process HashMap reseed a same-process loop misses),
3. per-commit **golden placements** (a seed/rev bump that changes output fails and forces a
   re-bless),
4. the **cross-platform** CI golden (the headline contract),
5. (consumer) a `.CNC` sha256 golden via the real nest→post path.

---

## 7. The Python binding & public API

**Rust API (the entire surface):**
```rust
pub struct Placement { pub item: usize, pub x: f64, pub y: f64, pub rotation_deg: f64 }

pub fn nest(
    items: &[Polygon],          // one outline per part type, item-local coords (f64)
    qty: &[usize],
    container: &Container,      // boundary + holes/keepout zones (f64)
    min_sep: f64,               // = the consumer's effective_part_gap
    rotations: &[f64],          // {0,90,180,270}
    seed: u64,                  // no rand::random() fallback, ever
    budget: u64,                // fixed iteration/sample budget (NOT wall-clock)
) -> NestSolution;              // placements + unplaced
```

**PyO3 binding** (`crates/py`, module `ironnest`): marshal polygons as plain
`list[list[tuple[float,float]]]` (PyO3 `Vec<Vec<[f64;2]>>`) — no numpy, no JSON wire (the JSON
wire is exactly the drift class that killed the old CLI stub). One `#[pyfunction] nest(...)`.
abi3-py313 so one wheel per platform serves all 3.13+ interpreters.

**Consumer adapter** (`drawing_and_gcode`, new small module replacing the stub): convert
`Part.outer` (bbox-min normalized) + `Plate.usable_polygon().buffer(-edge_margin)` + holes/keepouts
→ ironnest inputs; convert placements back to the existing `Placement`/`NestResult`. Keep the loud
"rotation ∉ allowed set" guard. **Watch the anchor convention** — jagua centroid-centers items at
import; the adapter must reconcile to the consumer's bbox-min anchor (do it in Rust before
returning so Python sees one convention).

---

## 8. Consumer integration (`drawing_and_gcode` #258, narrowed)

- **Remove** `dxfgcode/nesting/shapely_blf.py`, `rectpack_nester.py`, the CLI `jagua.py` stub
  (+ their tests).
- **Keep** the engine-agnostic seam: `engine.py` (`Plate`, `SpacingRules`, `effective_part_gap`,
  `Placement`, `NestResult`), `manual.py` (`ManualNest.violations`), `multi_sheet.py`, and the
  whole safety spine (`sheet.py`, kerf/leads/cut-sequencing, the UNVERIFIED banner).
- **Add** `ironnest` as a hard dependency + the thin adapter module.
- **Audit sidecar** (`audit.py`): record `engine{ name:"ironnest", version, jagua_fork_rev, seed,
  iteration_budget, rotations, platform }` so any future drift is *explained*, closing the
  Cardinal-rule traceability gap.
- **Interior-void (#257)**: ironnest provides the native-holes capability; the cut-path
  sequencing DAG (the `InteriorVoidSequencingError` hard-block) stays #257's job.

---

## 9. The spike (validate the two load-bearing assumptions before building the optimizer)

We've committed, so this is *de-risking the foundation*, not a GO/NO-GO. Two questions:

1. **Density (cheap, ~½ day).** Build upstream jagua's `lbf -p bpp`, nest the consumer's
   committed mixed corpus (`tools/nest_benchmark.py --mixed`, on branch
   `feat/blf-perf-and-interior-void`), and **recompute utilization our way** (net of holes — never
   trust jagua's reported `density`). Confirm it clears ~75.5%.
2. **Cross-platform f64 determinism (the real risk, ~1 day).** On the fork: flip to f64 (or
   confirm the `fsize` flag), lock discrete rotations, run one representative nest on a Mac **and**
   a Windows CI runner, and **byte-diff**. If they match, the whole thesis holds.

If density disappoints *or* cross-platform determinism proves intractable, the fallback is an
all-Python pyclipper-NFP + seeded GA (deterministic by construction) — but the bar it must clear
is the spike's jagua number.

---

## 10. Phased roadmap

- **Phase 0 — Spike** (§9): density + x-platform determinism proofs. *(source verify ✅; density
  proof ✅ via Phase 2 — 92–99 % rectangular, 79 % irregular; x-platform determinism harness ✅ wired
  in Phase 3 — the cross-platform CI golden is the standing proof, green on its first push.)*
- **Phase 1 — Fork & f64. ✅ DONE (2026-06-20).** Vendored the minimal jagua subset into
  `crates/geo` (geometry + `fpa`) + `crates/cde` (CDE + entities + io + bpp + assertions); flipped
  every `f32`→`Scalar` (=f64); ported with the crate-split done via `pub use ironnest_geo as
  geometry` (upstream `crate::geometry::*` paths resolve unchanged). Determinism scrub: `sin_cos`/
  `atan2`→pure-Rust `libm`; rayon `par_iter`→sequential; debug `HashSet`→`Vec`; wall-clock
  `Instant`→`u64` 0. Green: `build --locked`, `clippy --all-targets -D warnings` (determinism gate +
  jagua's pedantic posture), 11 fork-locking tests. Adversarially verified (fidelity diff = clean;
  determinism clean except the geo-buffer residual). MSRV floor corrected 1.85→1.87 (`cast_*`).
- **Phase 2 — Optimizer.** `crates/optimizer`: deterministic placement search, discrete rotations,
  iteration budget. Benchmark vs 75.5%.
  - **2a Constructive + compaction ✅ DONE (2026-06-21).** Item order by descending diameter →
    sampled Left-Bottom-Fill search (surrogate fail-fast, `10·x_max+y_max` loss, bbox-tightening) →
    **drop-on-place** (binary-search BLF slide to contact) → slide-compaction + fill rounds. Self-
    contained portable **PCG64** PRNG (no `rand` dep), fixed sample budget, public `nest(items, qty,
    container, min_sep, rotations, seed, budget) → NestSolution`, caller-configurable discrete
    rotations (default cardinal). Anchor-free placements. **Density: 87–99% on rectangular parts
    (beats the 85–90% target; ~75.5% prior Python), ~65% on irregular.** In-process determinism
    golden holds; 22 tests; gate green. Committed `ed8ba72`. (See `docs/02` for full design.)
  - **2b Separation search ✅ DONE (2026-06-21).** sparrow's overlap-minimization **Guided Local
    Search** ported deterministically into `crates/optimizer/src/sep/` (proxy → tracker → evaluator →
    sampler/coord-descent → strike loop → bin-packing insertion driver). Allows overlap then shoves
    parts apart under GLS weights — *discovers* interlocking arrangements greedy can't (two triangles
    → a square, **2/2**). Irregular density: pentagon **65 → 79 %**, bricks **87 → 92 %**. Fixed
    budgets (no clock), single worker, PCG64 + Fisher-Yates shuffle, no Wiggle, canonical summation.
    Adversarial review caught + fixed one blocker (unplaceable-pose guard). Committed `39370c6`.
    **Design + what-shipped: `docs/02-optimizer-and-separation-search.md` (§10).**
- **Phase 3 — Determinism harness. ✅ DONE (2026-06-21).** The §6 test pyramid landed: (1) in-process
  run-to-run equality (`tests/nest.rs`); (2) **cross-subprocess** byte-diff; (3) per-commit **golden
  placements** — an `insta` snapshot of the canonical solver output (`item x y rot`, full-precision
  f64) produced by the `golden_dump` bin over a fixed `min_sep=0` corpus, blessed on macOS-arm64; (4)
  the **cross-platform CI golden** — `.github/workflows/ci.yml` runs build + clippy(`-D warnings`) +
  fmt + the golden (debug **and** release) on **macOS-arm64 + Windows-x64 + linux-x64**, so any
  divergence fails loudly. Verified locally: same-platform debug == release == blessed snapshot; the
  *cross-platform* pass executes on the first push (can't run Win/Linux on the dev box). `min_sep=0`
  keeps the corpus inside the proven-deterministic envelope (the geo-buffer offset, risk #2, is still
  not byte-stable). 37 tests green. (5) the consumer `.CNC` sha256 golden stays `drawing_and_gcode`'s.
- **Phase 4 — PyO3 wheel + CI.** `crates/py`, maturin + cibuildwheel → PyPI (Win-x64, macOS-arm64,
  linux), abi3-py313, committed `Cargo.lock`, `cargo deny` + SBOM.
- **Phase 5 — Consumer integration** (`drawing_and_gcode` #258): adapter, remove Python engines,
  audit sidecar, overlay/dry-run still gate the torch.
- **Phase 6 — Interior-void & multi-sheet polish** (subsumes #257's geometry).

---

## 11. Risks & open questions (ranked)

1. ~~**f64 port size**~~ **RESOLVED** (verified): typed rewrite of ~41 files, mechanical, ~1–2
   days; no `fsize` flag exists. Mitigated by adding our own `Scalar` alias. See `01-…md` §A.
2. **Cross-platform byte-identity actually holding** — after the Phase-1 fork, **exactly ONE**
   residual remains (everything else verified clean): `sin_cos`/`atan2` are now routed through
   pure-Rust `libm` (portable; runs on the placement path via `compose()` but byte-identically), so
   the lone hazard is **`geo-buffer`'s rounded offset**. ⚠ **Sharpened by the Phase-1 fork audit:**
   `geo-buffer 0.2` calls **std `f64::sin`/`f64::cos` internally** (its `ray::rotate_by`), so
   *registry-pinning alone is NOT enough* — its output is platform-divergent. It only matters when
   `min_item_separation != 0` (offset feeds the item's collision shape). **Action before shipping
   nonzero-separation configs:** replace with our own deterministic offsetter (miter/pinned-arc), OR
   vendor `geo-buffer` and swap its `sin`/`cos` for `libm`, then prove bit-stability in the
   x-platform golden. Until then, treat nonzero-min-sep layouts as not-yet-byte-identical. The
   x-platform CI golden is the safety net (fails loud).
3. **Density clears the bar** — `lbf` is a weak heuristic and `sparrow` is strip-packing; our
   optimizer must reach ~85–90% on a *fixed irregular bin*. The spike measures it before we invest.
4. **jagua edition 2024 / Rust ≥ ~1.85** in CI; no declared MSRV upstream → pin the toolchain.
5. **Item holes unsupported in jagua 0.7.2** → interior-void via container quality-0 geometry,
   confirmed against the fork.
6. **MPL-2.0** — fine; keep new code in new crates, publish changes to forked files.
7. **Co-development friction** (two repos) — use a `path`/editable dep until the wheel stabilizes.

**Decided 2026-06-20:**
- Repo **visibility: private now**, flip to public when ready to open-source (publishing is the
  one-way door).
- **License: MPL-2.0 uniform** (see §12).
- **Crate split: FINALIZED** as `geo`/`cde`/`optimizer`/`ironnest`/`py` (Phase 1 landed it).
  Boundary decisions made during the fork: jagua's single crate split with `geometry/*`+`util/fpa`→
  `geo` and everything else→`cde`; `cde` re-exports `geo` as `geometry` (`pub use ironnest_geo as
  geometry`) so upstream `crate::geometry::*` paths resolve unchanged; `io` stays whole in `cde` and
  `geo`'s lone inbound `crate::io` call (in `shape_modification::offset_shape`) was severed by
  inlining the polygon cleanup, keeping `geo` a true leaf.

---

## 12. Licensing & distribution

- **License: MPL-2.0 uniform across the whole repo** (decided 2026-06-20). One license, zero
  compatibility reasoning, fully open-sourceable; the forked jagua files keep their MPL headers and
  our modifications are published (file-scoped copyleft); our new crates are MPL-2.0 too.
- **Wheels:** maturin + cibuildwheel in GitHub Actions, **abi3-py313**, targets
  `x86_64-pc-windows-msvc` (prod), `aarch64-apple-darwin` (dev), `x86_64-unknown-linux-gnu` (CI),
  published via **PyPI Trusted Publishing (OIDC)** — no token secret. The shop `pip install`s it
  (full internet; the old "offline wheel" premise was stale). **No Rust toolchain on the shop PC.**
- **Auditability:** commit `Cargo.lock`, build `--locked`, pin the jagua fork rev; stamp resolved
  version + git sha into a `_build_info` surfaced in the consumer's verify-before-cut banner.

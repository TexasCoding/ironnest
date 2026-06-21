# ironnest optimizer — design & the separation search (Phase 2b)

**Status:** Phase 2a (constructive + compaction nester) ✅ shipped (`ed8ba72`). Phase 2b (the
overlap-minimization **separation search** for irregular-part density) is **next** and is what this
document specs out. Read [`00-…`](00-ironnest-architecture-and-plan.md) (the plan) and
[`01-…`](01-jagua-source-verification.md) (the fork grounding) first.

> **One-line orientation for a fresh session:** the engine already nests rectangular parts at
> **87–99%** (beats target). Irregular parts sit at **~65%** because greedy construction can't
> *discover* interlocking arrangements. The fix is sparrow's overlap-minimization **Guided Local
> Search (GLS)**, ported to our deterministic loop. A naive version was attempted this session and
> **reverted** — §3 explains exactly why it failed and §4 is the algorithm that works.

---

## 1. Where we are — the optimizer today (`crates/optimizer`)

Public API (`lib.rs`):

```rust
pub fn nest(
    items: &[Vec<[Scalar; 2]>],  // one outline per item type, item-local coords
    qty: &[usize],               // demand per type
    container: &[[Scalar; 2]],   // container boundary outline
    min_sep: Scalar,             // min separation (0.0 disables; drives the geo-buffer offset)
    rotations_deg: &[Scalar],    // allowed discrete rotations, degrees (default {0,90,180,270})
    seed: u64,                   // explicit PRNG seed — no entropy fallback, ever
    budget: u64,                 // samples per item placement (fixed; NEVER a wall clock)
) -> NestSolution;               // { placements: Vec<Placement>, unplaced: Vec<usize> }

pub struct Placement { pub item: usize, pub x: Scalar, pub y: Scalar, pub rotation_deg: Scalar }
```

Placements are **anchor-free**: `placed_point = Rot(rotation_deg)·original_point + (x, y)`. The
centroid-centering jagua applies at import is folded out by `search::original_to_placed`.

**Algorithm (current):**
1. **Order** items largest-first (descending CD-shape diameter; stable on ties) — `lib.rs`.
2. **Constructive fill** — for each item, `search()` (`search.rs`): two-phase sampling (80 % uniform
   over a tightening bbox + 20 % local refine), surrogate fail-fast (`detect_surrogate_collision`),
   full check (`detect_poly_collision`), kept by the **LBF loss** `10·x_max + y_max` (`loss.rs`).
   Then **drop-on-place** — `improve::place_dropped` binary-search-slides the part to bottom-left
   contact (true BLF) before committing.
3. **Improve** — `IMPROVE_ROUNDS = 3` rounds of: slide-compact every placed part toward
   bottom-left, then fill freed space with unplaced parts (`improve.rs`).
4. **Extract** anchor-free placements.

**Determinism mechanisms (all in place):** self-contained **PCG64** PRNG (`prng.rs`, no `rand`
dep — version-independent byte-stable stream); fixed sample/round budgets (no wall clock); fixed
draw order (rotation→x→y); slide is pure geometry (no PRNG); trig only via `libm`; `slotmap`/BTree,
no `HashMap`; no threads. The **in-process determinism golden** (`tests/nest.rs
::determinism_same_seed_is_byte_identical`) passes.

**Density baseline** (`examples/density.rs`, release): 10×10 in 100.5 (slack) **99 %**, 7×7 **96 %**,
13×7 bricks **87 %**, irregular pentagon **~65 %**. (The 81 % "10×10 in exactly 100" case is an
**exact-fit artifact** — touching counts as collision, so 9 fit per row not 10; not a real ceiling.)

Key files: `lib.rs` (API + orchestration), `search.rs` (per-item search + `feasible_at` +
`original_to_placed`), `loss.rs` (`LbfLoss`), `improve.rs` (`place_dropped`, `slide_bottom_left`),
`prng.rs` (PCG64), `tests/nest.rs`, `examples/density.rs`.

---

## 2. The problem Phase 2b solves

Greedy construction (even with drop-to-contact) places each part where it locally minimizes the
bottom-left loss. It **cannot discover** arrangements where parts must *interlock*: two right
triangles that pair into a square, a part that nests into another's concavity, etc. Empirically
(this session): finer rotations (4→8→12) and 4× budget did **not** help — pentagon stuck 65–69 %,
triangles 75 % when two should pair into ~100 %. This is structural, not a sampling-resolution issue.

The fix is **overlap-minimization separation** (Umetani 2009 → sparrow 2025): allow parts to
overlap, then iteratively shove them apart, using a **Guided Local Search** to escape local minima.
That is the only mechanism that rearranges already-placed parts to make room.

---

## 3. Why the first attempt failed (so we don't repeat it)

A pole-gradient separator was prototyped this session and **reverted** (not committed). It never
converged — `resolved=false` on every insertion. Root causes, each fixed by §4:

| What the attempt did | Why it failed | What sparrow does instead (§4) |
|---|---|---|
| **Hard cutoff** `ov = r₁+r₂−d` if `ov>0` | No gradient once circles barely separate → motion dies right before resolving | **Hyperbolic decay** keeps the proxy smooth & differentiable *everywhere* |
| **Analytic gradient** move (sum of repulsion vectors) | Wedged parts have canceling gradients → stuck; clamped parts block | **Sample + adaptive coordinate descent** — try candidate poses, keep the best |
| **No penalty weights** | Plain overlap-min sits in local minima forever | **GLS weights** accumulate on persistent overlaps → forces the jump out |
| **Drove the *proxy* to zero, then checked the CDE** | Inscribed poles under-cover → proxy=0 while shapes still overlap → CDE rejects → revert | **CDE is the feasibility arbiter**; the proxy only *ranks* candidates. Sample eval calls `collect_poly_collisions` (exact "who collides") and scores those by proxy |
| Single insertion, give up on first fail | No diversification | **Strikes + restart-with-two-item-swap**, keeping GLS weights |

**The key conceptual error:** the pole proxy is *not* meant to certify feasibility — the CDE does
that. The proxy is a *smooth ranking signal* for "which candidate pose overlaps least," and the
search's accept test is the exact `Layout::is_feasible` / `detect_poly_collision`. sparrow uses the
*same* inscribed poles we did — it works because of the decay + GLS + sample-based moves, not a
better overlap measure.

---

## 4. The algorithm to build — sparrow's GLS, ported deterministically

Source of truth: **sparrow** (Jeroen Gardeyn, MIT) — repo `https://github.com/JeroenGar/sparrow`,
paper *"An open-source heuristic to reboot 2D nesting research"* (arXiv **2509.13329**). Built on
jagua-rs, the same CDE we forked. Ancestor: **Umetani et al. 2009** (GLS for overlap minimization,
*Intl. Trans. OR* 16:661–683). jagua-rs CDE paper: DOI **10.1287/ijoc.2024.1025**.

> **Adapt, don't copy:** sparrow does **strip packing** (shrink a strip's width, separate, repeat).
> ironnest packs a **fixed irregular container**. We take sparrow's **separation core** (§4.1–4.4)
> but replace its strip-shrink outer loop with a **bin-packing driver** (§4.5).

### 4.1 The overlap proxy (`sparrow/src/quantify/overlap_proxy.rs`)

Each part carries **surrogate poles** = inscribed circles (poles of inaccessibility; jagua already
generates these — `SPSurrogate::poles: Vec<Circle>`, ~8–16 per part, transformed rigidly with the
part). For a pole pair `(p, q)` across two parts, circle–circle penetration with **hyperbolic
decay**:

```
pd        = (p.radius + q.radius) - dist(p.center, q.center)
epsilon   = max(diam_a, diam_b) * 0.01          // OVERLAP_PROXY_EPSILON_DIAM_RATIO
pd_decay  = if pd >= epsilon { pd } else { epsilon^2 / (-pd + 2*epsilon) }   // smooth, >0 always
overlap  += pd_decay * min(p.radius, q.radius)
// after all pairs:  overlap *= PI
```

Final pair loss with a **shape-difficulty penalty** (geometric mean of √convex-hull-area, so
big/concave parts cost more): `loss = sqrt(overlap_proxy + epsilon^2) * sqrt(sqrt(hullA)*sqrt(hullB))`.
Container collisions: same with a `2.0` factor and `penalty = shape_penalty(s, s)`.

Only `+ − × ÷ sqrt` — all IEEE-deterministic. Port `f32 → Scalar` (f64). The convex-hull area is on
the surrogate (`SPSurrogate::convex_hull_area`, geo `fail_fast/sp_surrogate.rs`).

### 4.2 Guided Local Search weights (`sparrow/src/quantify/tracker.rs`)

Objective = **total weighted overlap**: `Σ weight·loss` over all colliding pairs **and** container
collisions, stored in a symmetric `PairMatrix` (item×item) of `{ loss, weight }` (weight ≥ 1.0).
After each separation iteration, every entry's weight is multiplied by `m`, then `max(1.0)`:

- colliding pair: `m = 1.2 + (2.0 − 1.2) · (loss / max_loss_this_iter)` — worst collision grows fastest
- separated pair: `m = 0.95` (decay toward 1)

Constants (`consts.rs`): `GLS_WEIGHT_MIN_INC_RATIO=1.2`, `GLS_WEIGHT_MAX_INC_RATIO=2.0`,
`GLS_WEIGHT_DECAY=0.95`. Persistently-overlapping pairs accumulate weight until a part faces
prohibitive weight with all neighbours and **jumps elsewhere** — the GLS escape (no annealing).

### 4.3 Moving a part — sample + coordinate descent (`sparrow/src/sample/`, `eval/sep_evaluator.rs`)

**No analytic gradient, no MTV.** For each colliding part (processed in a PRNG-shuffled order),
`move_item` = remove → evaluate candidate poses → place at the best:

1. **Sampling** (`uniform_sampler.rs`): ~**1000** uniform poses across the container (+ optional
   focused-near-current). Default `N_UNIFORM_SAMPLES=1000`, `N_COORD_DESCENTS=3` (`consts.rs`).
2. **Refinement** (`coord_descent.rs`): **adaptive coordinate descent**, not gradient — cycle axes
   {Horizontal, Vertical, ForwardDiag, BackwardDiag, (Wiggle=rotation)}; propose two candidates
   either side of current along the active axis; step ×`1.1` on improvement, ×`0.5` on failure;
   stop when all steps fall below limits. (Drop **Wiggle** for discrete rotations → no continuous
   trig — a determinism win.)
3. **Sample evaluation** (`sep_evaluator.rs`): for a candidate `dt`, ask the **CDE** which entities
   it collides with (`collect_poly_collisions`), then score `Σ weight·overlap_proxy` over those.
   Supports an **`upper_bound` early bailout** (`collector.early_terminate`) — abandon a candidate
   once it exceeds the best-so-far; this is what makes 1000 samples affordable.

### 4.4 The separator loop (`sparrow/src/optimizer/separator.rs`)

```
while n_strikes < strike_limit:                       // kmax (explore ~3, compress ~5)
  while n_iter_no_improvement < iter_no_improve_limit: // nmax (explore ~200, compress ~100)
    move_items(...)                                   // §4.3 over all colliding parts
    update_weights(...)                               // §4.2
    loss = total UNWEIGHTED overlap
    if loss < min_loss * 0.98 { min_loss = loss; n_iter_no_improvement = 0 } else { ++ }
  if improved vs strike start { reset strikes } else { ++n_strikes }
  restore_but_keep_weights(incumbent); swap two large items   // perturbation, keeps GLS weights
return feasible?(loss == 0)
```

### 4.5 Bin-packing driver (our adaptation — replaces sparrow's strip shrink)

sparrow creates overlap by shrinking the strip. We have a fixed container, so create overlap by
**insertion**. After the current `improve()` pass, for each still-unplaced part (largest-first):

```
snapshot = layout.save()
place the part at its lowest-overlap pose (sample by proxy, §4.3 step 1)
run the separator (§4.4) on the whole layout    // may move neighbours
if layout.is_feasible(): keep (part placed)      // CDE is the arbiter
else: layout.restore(snapshot)                   // couldn't make room
```

Optional richer driver (closer to sparrow, higher density): place **all** parts up front (overlaps
allowed), then separate the whole layout; if it can't reach zero overlap, remove the
highest-weighted part and retry. Start with the simpler insertion driver; graduate if needed.

---

## 5. Determinism adaptations (mandatory — sparrow is NOT byte-deterministic as shipped)

| sparrow (as shipped) | ironnest requirement |
|---|---|
| `f32` proxy/tracker | `Scalar` = **f64** throughout (proxy uses only `+−×÷sqrt`, safe) |
| `N_WORKERS = 3` threads (`worker.rs`) | **single canonical worker** (no rayon/threads on placement path) |
| **wall-clock** time limit (`-t`, 80/20 split) | **fixed iteration/sample budget** — never a clock |
| `rand` ecosystem RNG | our vendored **PCG64** (`prng.rs`); seed explicitly; fix every order (sample draw, shuffle, swap) to the PRNG stream |
| `Wiggle` continuous rotation (trig) | discrete rotations {0,90,180,270} → **drop Wiggle**, no `sin_cos` (or route through `libm` if continuous is ever added) |
| `target-cpu=native` + nightly `simd/` | scalar path only; CLAUDE.md already bans `-C target-cpu=native` |
| `min_by_key(OrderedFloat(..))` ties | **deterministic tie-break** (by item id / sample index) — equal proxy losses are plausible |
| float summation in the collector | **fix the accumulation order** — float `+` is non-associative; sum pairs in a stable (id) order |

Everything must keep the existing rules (CLAUDE.md "determinism rules"): `BTree`/`slotmap` only, no
`HashMap`, no FMA, libm-only trig, no wall clock. The in-process golden + (eventually) the
cross-platform golden (Phase 3) are the proof.

---

## 6. The geometry/CDE API to build on (verified file:line)

**Collision (the feasibility arbiter):**
- `CDEngine::detect_poly_collision(shape, filter) -> bool` — `cde/.../cd_engine.rs:166`
- `CDEngine::detect_surrogate_collision(surrogate, transf, filter) -> bool` — `:210` (fast fail)
- `CDEngine::collect_poly_collisions(shape, &mut collector)` — `:268` (which hazards collide → the
  set sparrow scores). `BasicHazardCollector` (`hazards/collector.rs`); `NoFilter` /
  `HazKeyFilter::from_keys` (`hazards/filter.rs`).

**Surrogate poles (the proxy input):** `shape.surrogate() -> &SPSurrogate`
(`geo/.../simple_polygon.rs:115`); `SPSurrogate { poles: Vec<Circle>, convex_hull_area: Scalar, … }`
(`geo/.../fail_fast/sp_surrogate.rs:22`); `Circle { center: Point, radius: Scalar }`, implements
`Transformable`. A **placed** item already carries transformed poles:
`layout.placed_items[pk].shape.surrogate().poles`. Pole circle–circle penetration is the proxy.

**Distance/separation primitives that DO exist** (note: only against `Point`, not poly–poly):
`SeparationDistance<Point> for SPolygon` returns `(Interior, distance_to_closest_edge)` if inside —
i.e. a **per-point penetration depth** (`geo/.../simple_polygon.rs:344`). `Edge::collides_at(&Edge)
-> Option<Point>` exact intersection (`edge.rs:63`), `Edge::closest_point_on_edge`
(`edge.rs:75`), `edge_intersection` (`edge.rs:199`). There is **no** SPolygon–SPolygon distance and
**no** Minkowski/NFP in the fork.

**Layout mutation (integration):** `Layout::save()->LayoutSnapshot` / `restore` (`layout.rs:58/69`),
`place_item(&Item, DTransformation)->PItemKey` / `remove_item(PItemKey)` (`:81/:96`), `cde()`
(`:133`), `is_feasible()` (`:139`), public `placed_items: SlotMap<PItemKey, PlacedItem>`.
`PlacedItem { item_id, d_transf, shape: Arc<SPolygon> }`.

**Integration point:** in `lib.rs::nest`, the separator runs **after** the `improve::improve(...)`
call and **before** `extract_placements(...)`, mutating `layout` and `placed_per_type`.

---

## 7. Two design paths (recommended order)

1. **PRIMARY — proxy-driven GLS (sparrow's core).** §4 verbatim, de-threaded / f64 / PCG64 /
   fixed-budget. Pros: proven SoTA; only `+−×÷sqrt`; reuses jagua's poles + CDE; lowest determinism
   risk. The pole proxy never needs to be exact (CDE is the arbiter). **Build this.**
2. **OPTIONAL refinement — exact NFP directional penetration**, *only* on discrete rotations
   (precompute No-Fit-Polygons per shape-pair × rotation; penetration = distance to nearest NFP
   edge — Umetani's exact measure). Most accurate, but NFP construction is the classic
   numerical-robustness minefield and a determinism liability to prove. Consider only if the proxy
   GLS leaves density on the table after tuning.
   - **Avoid GJK+EPA** as the primary measure (convex-only → concave decomposition distorts the very
     concave-pocket interlocking we want; iterative tolerance is an extra determinism risk).
   - **Avoid raster/penetration-map** (grid-quantizes placements — incompatible with byte-exact
     continuous `(x,y)`).

The CDE-survey agent independently proposed an **edge-based poly-penetration / MTV** built from
`collect_poly_collisions` + `Edge::collides_at` + per-vertex `sq_separation_distance`. That's a valid
exact-penetration path (a lighter cousin of NFP), but it is *not* what the SoTA does and adds
robustness/determinism surface. **Treat it as a fallback, not the first build** — the proxy GLS is
the recommended path.

---

## 8. Concrete build plan (incremental, each step verifiable)

1. **`quantify` module** — port the overlap proxy (§4.1) to f64 + the `PairMatrix`/tracker (§4.2).
   Unit-test: proxy of two identical squares at increasing offsets is monotone & smooth; weight
   update matches the formula.
2. **Sample-eval** — `collect_poly_collisions` + proxy scoring + the `upper_bound` early bailout,
   with a **fixed summation order** (sort colliding keys). Unit-test determinism (same layout+pose →
   same score).
3. **`move_item`** — sampler (PCG64, 1000 uniform) + adaptive coordinate descent (§4.3, no Wiggle).
   Single worker. Determinism test.
4. **Separator loop** (§4.4) — strikes, no-improvement counter, restore-keeping-weights, two-item
   swap. All counts fixed (no clock).
5. **Bin-packing driver** (§4.5) — the insertion form first. Wire after `improve()`.
6. **Measure** on `examples/density.rs` (pentagon, plus add a right-triangle case that *should* pair
   to ~100 %). Target: irregular density 65 % → 80–85 %.
7. **Tune** constants (sample count, nmax/kmax, decay) — but keep them **fixed integers/ratios**,
   never time. Re-run the in-process determinism golden after every change.
8. **Verify** — determinism-auditor + rust-reviewer workflow (as in Phases 1–2); then the Phase-3
   cross-platform golden is the final proof.

**Budget/perf note:** the separator is the expensive part (1000 samples × parts × iterations ×
strikes). Keep test sizes tiny (debug is slow — the existing dense test was 131 s before shrinking).
Profile in `--release`; gate large runs behind the `density` example, not unit tests.

---

## 9. Open questions / verify-before-coding (research caveats)

Read these in the actual sparrow source before hardcoding (the briefing was from GitHub raw fetch,
not a compile):
- **Exact constants** — `nmax/kmax/iter_no_improve/strike_limit` and the sample counts: confirm in
  `sparrow/src/config.rs` + `consts.rs` (paper Table 1 gives ~200/3 explore, ~100/5 compress).
- **`max_loss`** in the weight update — per-iteration vs global? Check `tracker.rs::update_weights`.
- **Collector summation order** — confirm `sep_evaluator.rs` / the specialized collector sums in a
  fixed order (float `+` is non-associative → our determinism depends on it).
- **Surrogate-pole generation determinism** — poles come from jagua's `SPSurrogate` (polylabel:
  priority-queue cell subdivision). Confirm our fork's generation is ordered (BTree/sorted, not
  HashMap) so the pole set is byte-identical cross-platform. (`geo/.../fail_fast/pole.rs`.)
- **Umetani 2009** weight rule (paywalled) — free preprint exists if the exact increment is needed;
  sparrow's rule (§4.2) is the practical one to implement.

## References
- sparrow — `https://github.com/JeroenGar/sparrow` · paper arXiv `2509.13329`.
- jagua-rs CDE — DOI `10.1287/ijoc.2024.1025` · arXiv `2508.08341` ·
  `https://github.com/INFORMSJoC/2024.1025`.
- Umetani et al. 2009 — *Intl. Trans. in Op. Research* 16:661–683 (GLS for overlap minimization).
- Local upstream source to read: `/private/tmp/jagua_src/jagua-rs` (surrogate/poles). sparrow files
  to read: `src/quantify/{overlap_proxy,tracker,pair_matrix}.rs`, `src/optimizer/separator.rs`,
  `src/sample/coord_descent.rs`, `src/eval/sep_evaluator.rs`, `src/{consts,config}.rs`.

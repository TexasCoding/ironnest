# ironnest — Claude working notes

**ironnest is a deterministic, embeddable, true-shape 2D nesting engine for irregular parts.**
Rust core + a PyO3 wheel. It is a **fork-and-extend of [jagua-rs](https://github.com/JeroenGar/jagua-rs)**
(MPL-2.0) — its collision-detection engine pushed to **f64** and made **cross-platform
byte-reproducible** — with **our own placement optimizer** on top.

**Read first:** [`docs/00-ironnest-architecture-and-plan.md`](docs/00-ironnest-architecture-and-plan.md)
(the plan) and [`docs/01-jagua-source-verification.md`](docs/01-jagua-source-verification.md)
(the source-verified fork grounding). They are the source of truth; this file is the short version.

## ⛔ Two prime directives

1. **Determinism is the product.** The same inputs MUST produce **byte-identical** placements — and
   therefore a byte-identical downstream cut program — on **every platform we ship** (macOS-arm64
   dev == Windows-x64 prod). This is what the consumer (a 260 A plasma tool) needs for its machine
   audit trail, and it is ironnest's differentiator. It is proven by a **cross-platform CI golden**,
   not assumed. Anything that can break it is a bug, not a tuning knob.

2. **The engine is a pure placement oracle.** In: polygons + an irregular container (boundary +
   holes/keepout zones) + a min-separation + an allowed-rotation set + a seed + an iteration budget.
   Out: `(item, x, y, rotation)`. **It knows NOTHING about kerf, leads, pierces, cut sequencing,
   G-code, or any machine number** — those live in the consumer, which re-validates every layout and
   refuses an illegal-but-dense one. Never put plasma/machine concepts in here.

## The determinism rules (how directive #1 is enforced)

The forked jagua core is *already* mostly deterministic (slotmap + BTree, no library RNG, rayon is
import-only-then-resorted, `Instant` is metadata). Keep it that way. The verified rules:

- **`Scalar = f64`** — define `pub type Scalar = f64;` and use it everywhere; never re-introduce
  hard-coded `f32`. (f64 is for robustness; it is *not* the determinism lever.)
- **Discrete rotations `{0, 90, 180, 270}`** → **hardcode the exact rotation matrices** (entries ∈
  {0, ±1}). NO `sin_cos` on the placement path. (Continuous rotation, if ever added, must route
  through the pure-Rust `libm` crate, behind a flag.)
- **The min-separation offset must be deterministic.** jagua uses `geo-buffer::buffer_polygon_rounded`
  (arc joins, third-party) — **replace it** with our own deterministic offsetter (miter / pinned
  arc) or vendor+pin it and prove bit-stability in the x-platform golden.
- **No RNG except our own seeded, portable PRNG** (e.g. `rand_pcg`/ChaCha). Explicit seed always; no
  `rand::random()` fallback, ever.
- **No `rayon` / threads on any placement-deciding path.** Single canonical worker; if parallel, fix
  the reduction order. (Import-time `par_iter` is fine — it's re-sorted by id.)
- **`BTreeMap`/`BTreeSet`/`slotmap` only** where order affects placement. Never std `HashMap`
  (per-process random iteration).
- **No `f64::mul_add` / no fast-math.** Basic `+ - * / sqrt` and `powi` are IEEE-deterministic — safe.
- Stamp solution metadata (`time_stamp`) to a constant for reproducible output.

## The fork

- Upstream: **jagua-rs 0.7.2** (commit `43e8137`), MPL-2.0, Rust **edition 2024** (needs Rust ≥ 1.85).
  All deps pure-Rust (no C / no `build.rs`) → clean cross-platform wheels.
- **Fork ≈ 5,800 LOC** (see `docs/01` §B): all of `geometry/` (ex-SVG) + `collision_detection/` +
  `entities/` + `io/import.rs`+`ext_repr.rs`+`export.rs` + `probs/bpp/` + `util/fpa.rs`. **Skip**
  `io/svg/`, `probs/spp`+`mspp`, and the `lbf` binary (read `lbf` only for the
  place/remove/save/restore call pattern — it owns the only RNG, which we replace).
- **Holes** live on the *container* as quality-0 zones (`Container.quality_zones`,
  `N_QUALITIES=10`). jagua 0.7.2 **drops item holes** → interior-void is modeled as container
  geometry / a second pass, never an item hole.
- MPL-2.0 is **uniform across this whole repo**: keep MPL headers on forked files, publish our
  modifications; new crates are MPL-2.0 too.

## Architecture / recommended crate split (finalize at scaffold time)

```
crates/core       forked jagua geometry + CDE + entities + io + bpp, at f64, determinism-scrubbed
crates/optimizer  OUR deterministic placement search (lifts sparrow's separator math, MIT — no link)
crates/ironnest   the public API:  nest(items, container, min_sep, rotations, seed, budget) -> [Placement]
crates/py         PyO3 binding -> the `ironnest` wheel (abi3-py313)
benches/  tests/   density + the cross-platform determinism golden
.github/           cibuildwheel (Win-x64 + macOS-arm64 + linux) -> PyPI; the x-platform golden gate
```
(A simpler `core` + `optimizer` + `py` split is fine to start; the public `nest()` can live in
`optimizer`.)

## Build / dev

- `cargo build --release --locked` · `cargo test` (commit `Cargo.lock` — auditability).
- Python dev loop: `cd crates/py && maturin develop --release` → `import ironnest`.
- Wheels: maturin + cibuildwheel, **abi3-py313**, targets `x86_64-pc-windows-msvc`,
  `aarch64-apple-darwin`, `x86_64-unknown-linux-gnu`; publish via PyPI Trusted Publishing (OIDC).
- `cargo deny` (license allowlist) + a CycloneDX SBOM in CI.

## Claude Code workspace (how the rules above are made mechanical)

The repo is wired so the determinism rules are enforced, not just remembered:

- **`clippy.toml`** bans the hazards as lints — `HashMap`/`HashSet`, `Instant`/`SystemTime::now`,
  `mul_add`, and the std transcendentals (`sin_cos`/`sin`/`cos`/`tan`/`atan2`/`powf`/`exp`/`ln`).
  Enforced by `cargo clippy -- -D warnings`. Uncomment the `rand::*` entries once `rand` is a dep;
  add `#![warn(clippy::disallowed_types, clippy::disallowed_methods)]` to each forked crate root.
- **`rust-toolchain.toml`** pins rustc (1.96.0) + clippy/rustfmt/rust-analyzer/rust-src via **rustup**
  (not Homebrew rust — that can't pin and would drift on `brew upgrade`). Bump deliberately and
  re-bless goldens; never implicitly.
- **`.claude/settings.json`** — cargo command allowlist (fewer prompts), a `PostToolUse` `cargo fmt`,
  a `Stop` `cargo clippy -- -D warnings` gate (both no-op until a `Cargo.toml` exists), and enables
  the `rust-analyzer-lsp` plugin so diagnostics surface after every edit.
- **`.claude/agents/determinism-auditor.md`** — delegate a full hazard sweep before merging.
- **`.claude/agents/rust-reviewer.md`** — fork-aware quality / idiom / MPL review.
- **`/rust-determinism-audit`** skill — fast inline grep + clippy sweep.

One-time local setup: rustup (not brew rust) → `/plugin install rust-analyzer-lsp@claude-plugins-official`
(needs the `rust-analyzer` binary on PATH; the toolchain file installs it). For the cross-platform
golden use `cargo insta` snapshots of the **solver output** (`item,x,y,rotation`) — never hash the
wheel/binary (cross-platform binaries are never byte-identical; only the placement output is the
contract). Never build with `-C target-cpu=native`.

## Consumer & tracking

- Consumer: `drawing_and_gcode` (the plasma CAD/CAM app, Python), at
  `/Volumes/RAID-0/Code/Python/drawing_and_gcode`. It depends on the ironnest wheel and owns the
  safety re-validation. Integration is tracked there by **issue #258**.
- Density bar to beat: the consumer's `tools/nest_benchmark.py --mixed` (on branch
  `feat/blf-perf-and-interior-void`) — today's pure-Python engine tops **~75.5%** util; rectpack
  floor **67.3%**; target **~85–90%**.

## Working style

- Determinism-first; prove byte-stability in CI before claiming it. Match upstream jagua's code
  style in forked files. Don't commit/push unless asked. Keep the oracle boundary clean.

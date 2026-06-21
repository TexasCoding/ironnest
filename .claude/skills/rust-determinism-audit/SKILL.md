---
name: rust-determinism-audit
description: Fast inline determinism sweep for ironnest Rust — greps for cross-platform hazards (f32, HashMap, transcendentals, FMA, RNG, threads, wall-clock) and runs the clippy gate. Use before committing placement-path changes.
---

# Rust determinism audit (inline)

Determinism is ironnest's product: byte-identical placements on every shipped platform. Run this
quick sweep before committing changes to `geo` / `cde` / `optimizer` or any placement-deciding code.
For a deeper, delegated review use the `determinism-auditor` subagent instead.

## 1. The clippy gate (mechanical — encodes the rules)

```bash
cargo clippy --all-targets -- -D warnings
```

Any `disallowed_types` / `disallowed_methods` hit is a determinism violation defined in
`clippy.toml`. Fix it — do not `#[allow]` it.

## 2. Grep for hazards clippy can't fully see

```bash
rg -n --type rust \
  -e '\bf32\b' -e 'NotNan<f32>' \
  -e '\bHashMap\b' -e '\bHashSet\b' \
  -e 'sin_cos|\.sin\(|\.cos\(|\.tan\(|atan2|powf|\.exp\(|\.ln\(' \
  -e 'mul_add|_algebraic|fast.?math' \
  -e 'rand::random|thread_rng|StdRng|getrandom' \
  -e 'par_iter|par_sort|par_bridge|rayon' \
  -e 'Instant::now|SystemTime::now' \
  -e 'target-cpu\s*=\s*.native.'
```

## 3. Triage each hit against the rule it touches

| Hit | Rule | Allowed? |
|---|---|---|
| `f32` on a Scalar path | width = f64 | ❌ (test / FFI edge only) |
| `HashMap`/`HashSet` where order → placement | BTree/slotmap only | ❌ (debug-only assertion set OK) |
| `sin_cos`/`sin`/`cos`/`atan2`/`powf`/`exp`/`ln` | platform libm, non-portable | ❌ on placement path — hardcode {0,±1} matrices or pinned `libm` |
| `mul_add` / `_algebraic` | FMA / fast-math diverge | ❌ — use `a*b + c` |
| `rand::random`/`thread_rng`/`getrandom` | explicit seed only | ❌ — seeded portable PRNG |
| `par_*`/`rayon` deciding placement | fixed reduction order | ❌ (import-then-resort OK) |
| `Instant::now`/`SystemTime::now` feeding output | metadata only | ❌ — constant `time_stamp` |
| `+ - * / sqrt powi` | IEEE-deterministic | ✅ |

A hazard is a bug, not a tuning knob. Prove byte-stability in the cross-platform CI golden before
claiming it.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! The determinism golden (Phase 3). Two levels of the §6 test pyramid, both fed by the
//! `golden_dump` binary (the single source of the canonical solver output):
//!
//! - **Golden placements (level 3 + the cross-platform contract).** An `insta` snapshot of the dump.
//!   The committed `.snap` is blessed once on macOS-arm64; CI re-runs this test on Windows-x64 and
//!   linux-x64, so **any cross-platform divergence — or any unintended output change on a commit —
//!   fails here loudly** and forces a deliberate re-bless (`cargo insta accept`).
//! - **Cross-subprocess byte-diff (level 2).** Runs the dump in two separate processes and asserts
//!   byte-identical stdout — catches per-process nondeterminism (e.g. a randomized `HashMap` seed)
//!   that a same-process loop would miss. (Belt-and-suspenders: the engine bans `HashMap` via the
//!   clippy gate, so this is a regression guard.)
//!
//! In-process run-to-run equality (level 1) lives in `nest.rs`; the cross-platform CI golden
//! (level 4) is this snapshot executed by `.github/workflows/ci.yml` on all three runners.

use std::process::Command;

/// Runs the `golden_dump` binary in a fresh process and returns its stdout. `CARGO_BIN_EXE_*` is
/// injected by cargo for binaries in this package.
fn run_golden_dump() -> String {
    let exe = env!("CARGO_BIN_EXE_golden_dump");
    let output = Command::new(exe)
        .output()
        .expect("failed to run the golden_dump binary");
    assert!(
        output.status.success(),
        "golden_dump exited with {:?}\nstderr:\n{}",
        output.status,
        String::from_utf8_lossy(&output.stderr),
    );
    String::from_utf8(output.stdout).expect("golden_dump output is valid UTF-8")
}

#[test]
fn golden_placements_match_snapshot() {
    insta::assert_snapshot!(run_golden_dump());
}

#[test]
fn cross_subprocess_byte_identical() {
    let first = run_golden_dump();
    let second = run_golden_dump();
    assert_eq!(
        first, second,
        "two independent processes must produce byte-identical solver output"
    );
}

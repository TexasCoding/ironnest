# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

"""Smoke + determinism tests for the built `ironnest` wheel.

Run in CI after the wheel is installed (`pytest crates/py/tests`). The engine's *cross-platform*
byte-identity is proven by the Rust golden (Phase 3); these tests prove the wheel exposes that engine
correctly and that the Python marshalling preserves determinism (Python `float` is IEEE f64, so
`==` on the returned tuples is an exact bit comparison)."""

import math
import os
import threading
import time

import pytest

import ironnest

CARDINAL = [0.0, 90.0, 180.0, 270.0]


def _rect(w, h):
    """A w×h axis-aligned rectangle, lower-left at the origin (CCW)."""
    return [(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)]


def _rect_at(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def test_module_surface():
    assert callable(ironnest.nest)
    assert callable(ironnest.nest_multi)
    assert isinstance(ironnest.__version__, str)


def test_places_all_when_there_is_room():
    placements, unplaced = ironnest.nest(
        [_rect(10.0, 10.0)], [4], _rect(100.0, 100.0), [], 0.0, CARDINAL, 1, 1000
    )
    assert len(placements) == 4
    assert unplaced == []
    for item, x, y, rot in placements:
        assert item == 0
        assert rot in CARDINAL
        assert isinstance(x, float) and isinstance(y, float)


def test_deterministic_same_seed_is_byte_identical():
    items = [_rect(10.0, 10.0), _rect(20.0, 5.0)]
    args = (items, [6, 4], _rect(60.0, 60.0), [], 0.0, CARDINAL, 7, 1500)
    first = ironnest.nest(*args)
    second = ironnest.nest(*args)
    # Exact tuple/float equality — the wheel must reproduce the engine byte-for-byte.
    assert first == second


def test_separation_search_discovers_interlock():
    # Two right triangles can only both fit in 11×11 by interlocking into a 10×10 square — a thing
    # greedy construction cannot find and the GLS separation search must.
    tri = [(0.0, 0.0), (10.0, 0.0), (0.0, 10.0)]
    placements, _ = ironnest.nest([tri], [2], _rect(11.0, 11.0), [], 0.0, CARDINAL, 42, 2000)
    assert len(placements) == 2


def test_holes_are_respected():
    # A 60×60 sheet with a central 20×20 keep-out hole; no-rotation 10×10 squares must avoid it.
    hole = _rect_at(20.0, 20.0, 40.0, 40.0)
    placements, _ = ironnest.nest([_rect(10.0, 10.0)], [16], _rect(60.0, 60.0), [hole], 0.0, [], 9, 1200)
    assert len(placements) > 0
    for _item, x, y, _rot in placements:
        ox = min(x + 10.0, 40.0) - max(x, 20.0)
        oy = min(y + 10.0, 40.0) - max(y, 20.0)
        assert not (ox > 1e-3 and oy > 1e-3), f"part at ({x},{y}) overlaps the hole"


def test_nest_multi_spills_and_is_deterministic():
    item = _rect(10.0, 10.0)
    sheets = [(_rect(25.0, 25.0), [])] * 3  # three 25×25 sheets, no holes
    first = ironnest.nest_multi([item], [8], sheets, 0.0, CARDINAL, 1, 800)
    second = ironnest.nest_multi([item], [8], sheets, 0.0, CARDINAL, 1, 800)
    assert first == second
    per_sheet, unplaced = first
    placed = sum(len(s) for s in per_sheet)
    assert placed + len(unplaced) == 8
    assert len(per_sheet) >= 2 and len(per_sheet[1]) > 0  # demand spilled past sheet 1


def test_length_mismatch_raises_value_error():
    with pytest.raises(ValueError):
        ironnest.nest([_rect(1.0, 1.0)], [1, 2], _rect(10.0, 10.0), [], 0.0, CARDINAL, 1, 100)


# --- Per-item allowed-rotation sets (rotations as list[list[float]]) ----------------------------

_TRI8 = [(0.0, 0.0), (10.0, 0.0), (0.0, 10.0)]
_SQUARE_SET = [0.0, 90.0]
_TRI_SET = [0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0]


def test_per_item_rotations_are_respected_per_type():
    # A square pinned axis-aligned ({0,90}) and a right triangle free on a fine 45° step, in ONE nest.
    # Every placement's rotation must come from THAT item's own set.
    items = [_rect(10.0, 10.0), _TRI8]
    placements, unplaced = ironnest.nest(
        items, [4, 6], _rect(40.0, 40.0), [], 0.0, [_SQUARE_SET, _TRI_SET], 11, 1500
    )
    assert unplaced == []

    def _in_set(deg, s):
        return any(abs(deg - r) < 1e-9 for r in s)

    for item, _x, _y, rot in placements:
        allowed = _SQUARE_SET if item == 0 else _TRI_SET
        assert _in_set(rot, allowed), f"item {item} placed at {rot}°, not in {allowed}"
    # The square's set has no 45°-family angle, so item 0 may never use one.
    assert all(_in_set(rot, _SQUARE_SET) for it, _x, _y, rot in placements if it == 0)


def test_per_item_rotations_are_deterministic():
    items = [_rect(10.0, 10.0), _TRI8]
    args = (items, [4, 6], _rect(40.0, 40.0), [], 0.0, [_SQUARE_SET, _TRI_SET], 11, 1500)
    assert ironnest.nest(*args) == ironnest.nest(*args)


def test_per_item_broadcast_equals_uniform():
    # The same set repeated per item must be byte-identical to the uniform flat-list form — the
    # invariant that keeps the cross-platform golden valid across the per-item refactor.
    items = [_rect(10.0, 10.0), _rect(20.0, 5.0)]
    uniform = ironnest.nest(items, [6, 4], _rect(60.0, 60.0), [], 0.0, CARDINAL, 7, 1500)
    per_item = ironnest.nest(items, [6, 4], _rect(60.0, 60.0), [], 0.0, [CARDINAL, CARDINAL], 7, 1500)
    assert uniform == per_item

    sheets = [(_rect(60.0, 60.0), [])]
    m_uniform = ironnest.nest_multi(items, [6, 4], sheets, 0.0, CARDINAL, 7, 1500)
    m_per_item = ironnest.nest_multi(items, [6, 4], sheets, 0.0, [CARDINAL, CARDINAL], 7, 1500)
    assert m_uniform == m_per_item


def test_single_item_disambiguation_is_by_type_not_length():
    # For a 1-item nest, [0.0] is the uniform form and [[0.0]] is the per-item form; both mean "no
    # rotation" here, so they must agree — proving the parser disambiguates by element TYPE, not length.
    items = [_rect(10.0, 10.0)]
    uniform = ironnest.nest(items, [4], _rect(40.0, 40.0), [], 0.0, [0.0], 3, 800)
    per_item = ironnest.nest(items, [4], _rect(40.0, 40.0), [], 0.0, [[0.0]], 3, 800)
    assert uniform == per_item
    assert all(rot == 0.0 for _it, _x, _y, rot in uniform[0])


def test_empty_rotations_means_no_rotation():
    placements, _ = ironnest.nest([_rect(10.0, 10.0)], [4], _rect(40.0, 40.0), [], 0.0, [], 1, 800)
    assert placements and all(rot == 0.0 for _it, _x, _y, rot in placements)


def test_int_angles_are_accepted_and_match_floats():
    items = [_rect(10.0, 10.0), _TRI8]
    as_floats = ironnest.nest(items, [4, 6], _rect(40.0, 40.0), [], 0.0, [_SQUARE_SET, _TRI_SET], 11, 1500)
    as_ints = ironnest.nest(
        items, [4, 6], _rect(40.0, 40.0), [], 0.0, [[0, 90], [0, 45, 90, 135, 180, 225, 270, 315]], 11, 1500
    )
    assert as_floats == as_ints


# Each entry is a `rotations` value that the parser must reject with ValueError. Two item types.
_BAD_ROTATIONS = [
    pytest.param([_SQUARE_SET], id="per-item-length-mismatch"),
    pytest.param([_SQUARE_SET, []], id="empty-inner-set"),
    pytest.param([0.0, float("nan")], id="nan-uniform"),
    pytest.param([_SQUARE_SET, [0.0, float("inf")]], id="inf-per-item"),
    pytest.param([0.0, "x"], id="non-numeric-uniform"),
    pytest.param("90", id="bare-string"),
]


@pytest.mark.parametrize("rotations", _BAD_ROTATIONS)
def test_invalid_rotations_raise_value_error(rotations):
    items = [_rect(10.0, 10.0), _TRI8]
    with pytest.raises(ValueError):
        ironnest.nest(items, [4, 6], _rect(40.0, 40.0), [], 0.0, rotations, 11, 500)


@pytest.mark.parametrize("rotations", _BAD_ROTATIONS)
def test_invalid_rotations_raise_value_error_in_nest_multi(rotations):
    items = [_rect(10.0, 10.0), _TRI8]
    sheets = [(_rect(40.0, 40.0), [])]
    with pytest.raises(ValueError):
        ironnest.nest_multi(items, [4, 6], sheets, 0.0, rotations, 11, 500)


# A solve heavy enough (~tens of ms) that thread-start and argument-marshalling overhead is noise.
_TRI = [(0.0, 0.0), (10.0, 0.0), (0.0, 10.0)]
_GIL_ARGS = ([_TRI, _rect(7.0, 3.0), _rect(4.0, 4.0)], [8, 8, 8], _rect(40.0, 40.0), [], 0.25, CARDINAL, 42, 20000)


def _time_single():
    t0 = time.perf_counter()
    ironnest.nest(*_GIL_ARGS)
    return time.perf_counter() - t0


def _time_concurrent(n):
    threads = [threading.Thread(target=lambda: ironnest.nest(*_GIL_ARGS)) for _ in range(n)]
    t0 = time.perf_counter()
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    return time.perf_counter() - t0


def _median(samples):
    return sorted(samples)[len(samples) // 2]


def _circle(n, r):
    """An n-vertex CCW regular polygon ("circle") of radius r, centered at the origin."""
    return [(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n)) for i in range(n)]


def _apply_pose(outline, rot_deg, x, y):
    """placed = Rot(rot_deg)·original + (x, y) — exactly how the consumer decodes a placement."""
    a = math.radians(rot_deg)
    c, s = math.cos(a), math.sin(a)
    return [(c * px - s * py + x, s * px + c * py + y) for px, py in outline]


def _point_seg_dist(p, a, b):
    abx, aby = b[0] - a[0], b[1] - a[1]
    len2 = abx * abx + aby * aby
    if len2 == 0.0:
        return math.dist(p, a)
    t = max(0.0, min(1.0, ((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / len2))
    return math.dist(p, (a[0] + t * abx, a[1] + t * aby))


def _seg_seg_dist(a, b, c, d):
    def ccw(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    if (ccw(c, d, a) > 0) != (ccw(c, d, b) > 0) and (ccw(a, b, c) > 0) != (ccw(a, b, d) > 0):
        return 0.0  # segments cross
    return min(
        _point_seg_dist(a, c, d),
        _point_seg_dist(b, c, d),
        _point_seg_dist(c, a, b),
        _point_seg_dist(d, a, b),
    )


def _poly_poly_dist(p, q):
    """Min boundary-to-boundary distance between two closed polygons (the consumer's gate metric)."""
    return min(
        _seg_seg_dist(p[i], p[(i + 1) % len(p)], q[j], q[(j + 1) % len(q)])
        for i in range(len(p))
        for j in range(len(q))
    )


def test_high_vertex_curved_part_keeps_min_sep_through_the_wheel():
    """The consumer's failure case, end-to-end through the wheel: a high-vertex curved part (the class
    that under-reserved on curves and was slow) must nest with its returned ORIGINAL outlines kept at
    least min_sep apart — from each other and the boundary — to within 1e-6. This is exactly the
    re-validation gate the consumer runs before cutting. Decimation is automatic; no API change."""
    min_sep = 0.75
    r = 12.0
    part = _circle(200, r)
    container = _rect(80.0, 52.0)
    placements, _ = ironnest.nest([part], [6], container, [], min_sep, CARDINAL, 7, 2500)
    assert len(placements) >= 2, f"need ≥2 placed to check spacing, got {len(placements)}"

    placed = [_apply_pose(part, rot, x, y) for (_item, x, y, rot) in placements]
    tol = 1e-6
    for i in range(len(placed)):
        for j in range(i + 1, len(placed)):
            gap = _poly_poly_dist(placed[i], placed[j])
            assert gap >= min_sep - tol, f"parts {i},{j} are {gap:.9f} apart, under min_sep {min_sep}"
        edge_gap = _poly_poly_dist(placed[i], container)
        assert edge_gap >= min_sep - tol, f"part {i} is {edge_gap:.9f} from boundary, under {min_sep}"


@pytest.mark.skipif((os.cpu_count() or 1) < 2, reason="needs ≥2 cores to observe parallelism")
def test_nest_releases_the_gil():
    """The solve must run with the GIL released so a worker-thread nest does not freeze the whole
    interpreter — the failure mode that stalled the consumer's asyncio event loop ("connection
    lost") and killed Ctrl+C. We prove the release indirectly but robustly: N identical solves on
    N threads run in *parallel* (true Rust threads, no GIL) instead of serializing, so the
    concurrent wall-clock stays well under N× a single solve. Were the GIL held for the whole C
    call (the bug), the N solves would serialize and the ratio would approach N.

    This is a coarse, self-calibrating check (a ratio measured on the same machine) — not a
    determinism assertion. Output byte-identity is covered by the Rust golden and the determinism
    tests above; releasing the GIL changes scheduling, never results. To stay non-flaky on shared
    CI runners it warms up first (discards the cache-cold solve) and takes the median of 3 samples,
    so a single GC pause or co-tenant burst cannot trip it; the 3× gate sits equidistant between
    the worst real parallel case (~2× on a 2-core box) and the serial bug (~n×), so it still fails
    a regression that re-acquires the GIL for the solve."""
    _time_single()  # warm up: discard the cache-cold first solve so the timings below are stable

    n = 4
    single = _median([_time_single() for _ in range(3)])
    concurrent = _median([_time_concurrent(n) for _ in range(3)])
    assert single > 0.002, f"baseline solve implausibly fast ({single * 1e3:.1f} ms) — test miscalibrated"

    ratio = concurrent / single
    assert ratio < 3.0, (
        f"{n} concurrent solves took {concurrent * 1e3:.0f} ms vs {single * 1e3:.0f} ms for one "
        f"(ratio {ratio:.2f}×; fully serial would be ~{n}×) — the GIL appears not to be released"
    )

# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

"""Smoke + determinism tests for the built `ironnest` wheel.

Run in CI after the wheel is installed (`pytest crates/py/tests`). The engine's *cross-platform*
byte-identity is proven by the Rust golden (Phase 3); these tests prove the wheel exposes that engine
correctly and that the Python marshalling preserves determinism (Python `float` is IEEE f64, so
`==` on the returned tuples is an exact bit comparison)."""

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

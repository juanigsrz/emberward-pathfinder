import random

import pytest

from interdiction.contract import contract
from interdiction.grid import parse_map
from interdiction.master import MasterSolver
from interdiction.window_master import solve_window


def _window(grid, r0, c0, size):
    return {(r, c)
            for r in range(max(r0, 0), min(r0 + size, grid.rows))
            for c in range(max(c0, 0), min(c0 + size, grid.cols))}


@pytest.mark.parametrize("r0,c0,size", [(0, 0, 3), (1, 1, 3), (0, 2, 4)])
def test_window_solve_matches_master_region_solve(make_map, r0, c0, size):
    grid = parse_map(make_map("""
        S.....
        ......
        ......
        .....T
    """))
    window = _window(grid, r0, c0, size)
    free = window & grid.buildable

    master = MasterSolver(grid, rng=random.Random(0))
    ref = master.solve(free=free, fixed_walls=set(), time_limit=60)

    cw = contract(grid, window, outside_walls=set())
    got = solve_window(cw, time_limit=60, corridor_hint=False)

    assert got.status == "OPTIMAL" and ref.status == "OPTIMAL"
    assert got.maximin == ref.maximin
    # window walls actually achieve the claim on the real grid
    val, _ = grid.evaluate(got.walls)
    assert val == got.maximin


def test_window_solve_with_outside_walls(make_map):
    grid = parse_map(make_map("""
        S.....
        ......
        ......
        .....T
    """))
    outside = {(1, 4), (2, 4)}
    window = _window(grid, 0, 0, 3)
    free = window & grid.buildable

    master = MasterSolver(grid, rng=random.Random(0))
    ref = master.solve(free=free, fixed_walls=outside, time_limit=60)

    cw = contract(grid, window, outside_walls=outside)
    got = solve_window(cw, time_limit=60, corridor_hint=False)

    assert got.maximin == ref.maximin
    val, _ = grid.evaluate(set(got.walls) | outside)
    assert val == got.maximin


def test_hinted_solution_respects_corridor_constraints(make_map):
    grid = parse_map(make_map("""
        S.....
        ......
        ......
        ......
        .....T
    """))
    window = _window(grid, 0, 0, 6)
    cw = contract(grid, window, set())
    res = solve_window(cw, time_limit=60, corridor_hint=True)
    assert res.walls is not None
    for (r, c) in sorted(cw.free):
        square = [(r, c), (r + 1, c), (r, c + 1), (r + 1, c + 1)]
        if all(v in cw.free for v in square):
            n = sum(1 for v in square if v in res.walls)
            assert 1 <= n <= 3


def test_hints_never_beat_unhinted(make_map):
    grid = parse_map(make_map("""
        S....
        .....
        ....T
    """))
    window = _window(grid, 0, 0, 5)
    cw = contract(grid, window, set())
    unhinted = solve_window(cw, time_limit=60, corridor_hint=False)
    hinted = solve_window(cw, time_limit=60, corridor_hint=True)
    assert unhinted.status == "OPTIMAL"
    if hinted.walls is not None:
        assert hinted.maximin <= unhinted.maximin


def test_warm_start_accepted(make_map):
    grid = parse_map(make_map("""
        S....
        .....
        ....T
    """))
    window = _window(grid, 0, 0, 5)
    cw = contract(grid, window, set())
    ref = solve_window(cw, time_limit=60, corridor_hint=False)
    again = solve_window(cw, time_limit=60, corridor_hint=False,
                         warm_start=ref.walls)
    assert again.maximin == ref.maximin

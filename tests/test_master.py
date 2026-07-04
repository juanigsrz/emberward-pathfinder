import random

import pytest

from interdiction.grid import parse_map
from interdiction.master import MasterSolver
from tests.conftest import brute_force_opt

TINY_MAPS = [
    # 4x4 open: 14 buildable cells
    """
    S...
    ....
    ....
    ...T
    """,
    # obstacle + two spawns: maximin coupling
    """
    S....
    .#...
    S...T
    """,
    # X cells: unbuildable corridor the solver cannot block
    """
    S.X..
    ..X.T
    """,
]


@pytest.mark.parametrize("text", TINY_MAPS)
def test_master_matches_brute_force(make_map, text):
    grid = parse_map(make_map(text))
    expected, _ = brute_force_opt(grid)
    res = MasterSolver(grid, rng=random.Random(0)).solve(time_limit=60)
    assert res.status == "OPTIMAL"
    assert res.maximin == expected
    # returned walls must actually achieve the claimed value
    val, per = grid.evaluate(res.walls)
    assert val == res.maximin and per == res.per_spawn
    # bound is tight at optimality
    assert round(res.bound) == expected


def test_master_never_overclaims_and_pool_grows(make_map):
    grid = parse_map(make_map("""
        S.....
        ......
        ......
        .....T
    """))
    solver = MasterSolver(grid, rng=random.Random(0))
    res = solver.solve(time_limit=60)
    assert res.status == "OPTIMAL"
    val, _ = grid.evaluate(res.walls)
    assert val == res.maximin
    assert len(solver.cut_pool) > 0


def test_region_solve_respects_fixed_cells(make_map):
    grid = parse_map(make_map("""
        S.....
        ......
        ......
        .....T
    """))
    solver = MasterSolver(grid, rng=random.Random(0))
    full = solver.solve(time_limit=60)

    # freeze everything outside a 2-column window to the incumbent
    window = {(r, c) for r in range(grid.rows) for c in (2, 3)}
    free = window & grid.buildable
    fixed = set(full.walls) - free
    res = solver.solve(free=free, fixed_walls=fixed, time_limit=60,
                       warm_start=full.walls)

    assert res.status == "OPTIMAL"
    # outside the window nothing changed
    assert (res.walls - free) == fixed
    # the full optimum is feasible for the restricted problem (it was the
    # warm start), and restricted can never beat full -> exact equality
    assert res.maximin == full.maximin


def test_pool_persists_and_grows_across_solves(make_map):
    grid = parse_map(make_map("""
        S....
        .....
        ....T
    """))
    solver = MasterSolver(grid, rng=random.Random(0))
    after_init = len(solver.cut_pool)
    solver.solve(time_limit=30)
    after_first = len(solver.cut_pool)
    assert after_first >= after_init
    solver.solve(time_limit=30)
    assert len(solver.cut_pool) >= after_first


def test_infeasible_warm_start_rejected(make_map):
    grid = parse_map(make_map("""
        S..
        ...
        ..T
    """))
    solver = MasterSolver(grid)
    with pytest.raises(AssertionError, match="disconnect"):
        solver.solve(time_limit=10, warm_start={(0, 1), (1, 0)})


def test_pool_cuts_valid_for_random_configs(make_map):
    grid = parse_map(make_map("""
        S.....
        ......
        ......
        ......
        .....T
    """))
    rng = random.Random(7)
    solver = MasterSolver(grid, rng=rng)
    solver.solve(time_limit=30)
    assert solver.cut_pool
    U = len(grid.walkable) - 1

    checked = 0
    for _ in range(200):
        walls = {v for v in grid.buildable if rng.random() < 0.3}
        val, per = grid.evaluate(walls)
        if val is None:
            continue                      # infeasible config: cuts don't apply
        checked += 1
        for k, path in solver.cut_pool:
            length = len(path) - 1
            hits = sum(1 for v in path if v in walls)
            assert per[k] <= length + (U - length) * hits, (
                f"cut (spawn {k}, len {length}) violated by feasible config")
    assert checked > 20

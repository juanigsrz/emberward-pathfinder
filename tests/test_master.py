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

import math
import random

from interdiction.bound import gap, run_bound
from interdiction.grid import parse_map
from interdiction.master import MasterSolver


def test_bound_run_tightens_to_optimum_on_small_map(make_map):
    grid = parse_map(make_map("""
        S....
        .....
        .....
        ....T
    """))
    master = MasterSolver(grid, rng=random.Random(0))
    seed = set()
    res = run_bound(grid, master, seed, time_limit=60)
    assert res.status == "OPTIMAL"
    # ObjBound is float and may sit epsilon below the integer optimum
    assert res.bound >= res.maximin - 1e-6
    assert round(res.bound) == res.maximin


def test_gap():
    assert gap(10, 12.0) == 0.2
    assert gap(10, 10.0) == 0.0
    assert math.isinf(gap(0, 5.0))

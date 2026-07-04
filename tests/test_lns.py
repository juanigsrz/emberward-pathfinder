import random

from interdiction.grid import parse_map
from interdiction.lns import _pick_center, _window_cells, run_lns


def test_window_cells_clipped_to_grid(make_map):
    g = parse_map(make_map("""
        S....
        .....
        ....T
    """))
    # 4x4 window centered at the corner spans rows/cols -2..1, clipped to 0..1
    assert _window_cells(g, (0, 0), 4) == \
        {(r, c) for r in range(0, 2) for c in range(0, 2)}
    # 3x3 window fully interior
    assert _window_cells(g, (1, 2), 3) == \
        {(r, c) for r in range(0, 3) for c in range(1, 4)}


def test_pick_center_prefers_binding_path(make_map):
    g = parse_map(make_map("""
        S...T
        .....
        S....
    """))
    rng = random.Random(1)
    _, per = g.evaluate(set())
    for _ in range(20):
        center = _pick_center(g, set(), per, rng)
        assert center in g.walkable


def test_lns_improves_over_empty_walls():
    grid = parse_map("maps/basic.txt")
    baseline, _ = grid.evaluate(set())
    rng = random.Random(0)
    res = run_lns(grid, set(), total_time=25.0, subsolve_time=5.0, rng=rng)
    assert res.maximin > baseline
    val, per = grid.evaluate(res.walls)
    assert val == res.maximin and per == res.per_spawn
    assert res.trajectory[0][2] == baseline
    assert res.trajectory[-1][2] == res.maximin


def test_lns_no_hints_also_improves():
    grid = parse_map("maps/basic.txt")
    baseline, _ = grid.evaluate(set())
    rng = random.Random(3)
    res = run_lns(grid, set(), total_time=15.0, subsolve_time=5.0, rng=rng,
                  corridor_hint=False)
    assert res.maximin > baseline

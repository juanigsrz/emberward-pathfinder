import random
from itertools import combinations

import pytest

from interdiction.cli import main
from interdiction.contract import contract
from interdiction.grid import parse_map, parse_solution, square2, \
    tile2_decompose
from interdiction.lns import run_lns
from interdiction.master import MasterSolver
from interdiction.window_master import solve_window


def assert_block_tiling(walls, anchors):
    """Walls must be exactly the disjoint union of the anchors' 2x2 squares."""
    tiles = {v for a in anchors for v in square2(a)}
    assert len(tiles) == 4 * len(anchors), "blocks overlap"
    assert tiles == set(walls)


def brute_force_blocks_opt(grid):
    """Exhaustive optimum over disjoint 2x2 block placements. Tiny maps only."""
    anchors = [a for a in sorted(grid.buildable)
               if all(v in grid.buildable for v in square2(a))]
    assert len(anchors) <= 12, "brute force limited to 2^12 subsets"
    best_val = grid.evaluate(set())[0]
    for r in range(1, len(anchors) + 1):
        for combo in combinations(anchors, r):
            cells = [v for a in combo for v in square2(a)]
            if len(set(cells)) < 4 * len(combo):
                continue
            val, _ = grid.evaluate(set(cells))
            if val is not None and val > best_val:
                best_val = val
    return best_val


def test_exact_blocks2_matches_brute_force(make_map):
    grid = parse_map(make_map("""
        S....
        .....
        .....
        ....T
    """))
    ref = brute_force_blocks_opt(grid)
    master = MasterSolver(grid, rng=random.Random(0), blocks2=True)
    res = master.solve(time_limit=60)
    assert res.status == "OPTIMAL"
    assert res.maximin == ref
    assert_block_tiling(res.walls, res.anchors)


def test_window_blocks2_solution_is_tiled(make_map):
    grid = parse_map(make_map("""
        S.....
        ......
        ......
        .....T
    """))
    window = {(r, c) for r in range(4) for c in range(1, 5)}
    cw = contract(grid, window, outside_walls=set())
    res = solve_window(cw, time_limit=60, blocks2=True)
    assert res.walls is not None
    assert_block_tiling(res.walls, res.anchors)
    assert all(set(square2(a)) <= cw.free for a in res.anchors)
    val, _ = grid.evaluate(res.walls)
    assert val == res.maximin


def test_lns_blocks2_improves_and_stays_tiled():
    grid = parse_map("maps/basic.txt")
    baseline, _ = grid.evaluate(set())
    res = run_lns(grid, set(), total_time=25.0, subsolve_time=5.0,
                  rng=random.Random(0), blocks2=True)
    assert res.maximin > baseline
    assert_block_tiling(res.walls, res.anchors)
    val, per = grid.evaluate(res.walls)
    assert val == res.maximin and per == res.per_spawn


def test_tile2_decompose_roundtrip():
    anchors = {(0, 0), (0, 2), (4, 1), (2, 4)}
    walls = {v for a in anchors for v in square2(a)}
    assert tile2_decompose(walls) == anchors
    assert tile2_decompose(set()) == set()
    with pytest.raises(ValueError):
        tile2_decompose(walls | {(9, 9)})          # lone cell
    with pytest.raises(ValueError):
        tile2_decompose({v for a in [(0, 0), (1, 1)]
                         for v in square2(a)})     # overlapping squares


def test_lns_blocks2_accepts_tiled_seed():
    grid = parse_map("maps/basic.txt")
    seed = set(square2((0, 3)))
    seed_val, _ = grid.evaluate(seed)
    res = run_lns(grid, seed, total_time=10.0, subsolve_time=4.0,
                  rng=random.Random(0), blocks2=True)
    assert res.maximin >= seed_val
    assert_block_tiling(res.walls, res.anchors)


def test_cli_blocks2_pipeline(make_map, tmp_path):
    path = make_map("""
        S......
        .......
        .......
        ......T
    """)
    out_file = str(tmp_path / "sol.txt")
    assert main([path, "--blocks2", "--time", "20", "--bound-frac", "0.5",
                 "--subsolve-time", "4", "--rng-seed", "1",
                 "--window-sizes", "6,8", "--out", out_file]) == 0
    grid = parse_map(path)
    walls = parse_solution(grid, out_file)
    baseline, _ = grid.evaluate(set())
    val, _ = grid.evaluate(walls)
    assert val >= baseline
    # every wall cell sits in some fully-walled 2x2 square (necessary
    # condition for a block tiling; the tiling itself is asserted in-process)
    for v in walls:
        r, c = v
        assert any(set(square2(a)) <= walls
                   for a in [(r, c), (r - 1, c), (r, c - 1), (r - 1, c - 1)])

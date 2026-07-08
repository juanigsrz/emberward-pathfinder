"""Large-neighborhood search: exact contracted-window re-optimization."""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from interdiction.contract import contract
from interdiction.grid import square2, tile2_decompose
from interdiction.window_master import solve_window

WINDOW_SIZES = (12, 16, 20)
STALL_LIMIT = 20
RANDOM_CENTER_PROB = 0.2


@dataclass
class LNSResult:
    walls: set
    maximin: int
    per_spawn: tuple
    trajectory: list = field(default_factory=list)  # (elapsed, iter, maximin)
    interrupted: bool = False
    anchors: set | None = None  # blocks2: top-left corners of placed blocks


def _window_cells(grid, center, size):
    r0 = center[0] - size // 2
    c0 = center[1] - size // 2
    return {(r, c)
            for r in range(max(r0, 0), min(r0 + size, grid.rows))
            for c in range(max(c0, 0), min(c0 + size, grid.cols))}


def _pick_center(grid, walls, per_spawn, rng):
    """Random cell of a binding spawn's shortest path, sometimes fully random."""
    if rng.random() >= RANDOM_CENTER_PROB:
        maximin = min(per_spawn)
        binding = [s for s, d in zip(grid.spawns, per_spawn) if d == maximin]
        spawn = rng.choice(binding)
        path = grid.shortest_path(walls, spawn, rng=rng)
        return rng.choice(path)
    return rng.choice(sorted(grid.walkable))


def run_lns(grid, seed_walls, *, total_time, subsolve_time=15.0, rng,
            corridor_hint=True, window_sizes=WINDOW_SIZES, blocks2=False):
    best = set(seed_walls)
    anchors: set = set()
    if blocks2:
        anchors = tile2_decompose(best)  # ValueError if seed untileable
        corridor_hint = False
    best_val, best_per = grid.evaluate(best)
    assert best_val is not None, "seed walls disconnect a spawn"

    t0 = time.monotonic()
    result = LNSResult(best, best_val, best_per,
                       trajectory=[(0.0, 0, best_val)],
                       anchors=anchors if blocks2 else None)
    stall = 0
    it = 0
    while True:
        remaining = total_time - (time.monotonic() - t0)
        if remaining < 1.0:
            break
        it += 1
        if stall >= STALL_LIMIT:
            size = window_sizes[-1]
        else:
            size = window_sizes[(it - 1) % len(window_sizes)]
        center = _pick_center(grid, result.walls, result.per_spawn, rng)
        window = _window_cells(grid, center, size)
        removed = set()
        if blocks2:
            # blocks straddling the window edge are freed whole, so the
            # window may grow by one cell per side
            removed = {a for a in anchors if set(square2(a)) & window}
            for a in removed:
                window |= set(square2(a))
        free = window & grid.buildable
        outside_walls = result.walls - free

        cw = contract(grid, window, outside_walls)
        res = solve_window(cw, time_limit=min(subsolve_time, remaining),
                           warm_start=result.walls & free,
                           corridor_hint=corridor_hint,
                           blocks2=blocks2,
                           warm_anchors=removed if blocks2 else None)
        if res.status == "INTERRUPTED":
            result.interrupted = True
        if res.walls is not None:
            candidate = outside_walls | res.walls
            val, per = grid.evaluate(candidate)
            # contraction exactness: BFS must agree with the contracted claim
            assert val == res.maximin and per == res.per_spawn, \
                "contracted claim disagrees with BFS — contraction bug"
            if val > result.maximin:
                if blocks2:
                    anchors = (anchors - removed) | res.anchors
                    tiles = {v for a in anchors for v in square2(a)}
                    assert tiles == candidate and \
                        len(tiles) == 4 * len(anchors), \
                        "walls are not a disjoint union of the tracked blocks"
                    result.anchors = anchors
                result.walls = candidate
                result.maximin, result.per_spawn = val, per
                result.trajectory.append(
                    (time.monotonic() - t0, it, result.maximin))
                stall = 0
            else:
                stall += 1
        else:
            stall += 1
        if result.interrupted:
            break
    return result

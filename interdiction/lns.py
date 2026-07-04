"""Large-neighborhood search: exact master re-optimization of map windows."""

from __future__ import annotations

import time
from dataclasses import dataclass, field

WINDOW_SIZES = (8, 12, 16)
STALL_LIMIT = 20
RANDOM_CENTER_PROB = 0.2


@dataclass
class LNSResult:
    walls: set
    maximin: int
    per_spawn: tuple
    trajectory: list = field(default_factory=list)  # (elapsed, iter, maximin)
    interrupted: bool = False


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


def run_lns(grid, master, seed_walls, *, total_time, subsolve_time=30.0, rng):
    best = set(seed_walls)
    best_val, best_per = grid.evaluate(best)
    assert best_val is not None, "seed walls disconnect a spawn"

    t0 = time.monotonic()
    result = LNSResult(best, best_val, best_per,
                       trajectory=[(0.0, 0, best_val)])
    stall = 0
    it = 0
    while True:
        remaining = total_time - (time.monotonic() - t0)
        if remaining < 1.0:
            break
        it += 1
        if stall >= STALL_LIMIT:
            size = WINDOW_SIZES[-1]
        else:
            size = WINDOW_SIZES[(it - 1) % len(WINDOW_SIZES)]
        center = _pick_center(grid, result.walls, result.per_spawn, rng)
        free = _window_cells(grid, center, size) & grid.buildable
        fixed = result.walls - free

        res = master.solve(free=free, fixed_walls=fixed,
                           time_limit=min(subsolve_time, remaining),
                           warm_start=result.walls)
        if res.status == "INTERRUPTED":
            result.interrupted = True
        if res.maximin is not None and res.maximin > result.maximin:
            result.walls = set(res.walls)
            result.maximin, result.per_spawn = res.maximin, res.per_spawn
            result.trajectory.append(
                (time.monotonic() - t0, it, result.maximin))
            stall = 0
        else:
            stall += 1
        if result.interrupted:
            break
    return result

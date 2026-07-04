"""Grid model for the interdiction problem: parsing, BFS, evaluation.

No solver dependencies — shared by the MILP master, the LNS driver and tests.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

Cell = tuple[int, int]

_DIRS = ((-1, 0), (1, 0), (0, -1), (0, 1))


@dataclass
class GridMap:
    rows: int
    cols: int
    spawns: tuple[Cell, ...]
    target: Cell
    obstacles: frozenset[Cell]      # '#': permanent, never walkable
    unbuildables: frozenset[Cell]   # 'X': walkable, never buildable
    preset_walls: frozenset[Cell]   # 'W': buildable cells that start walled

    def __post_init__(self):
        cells = {(r, c) for r in range(self.rows) for c in range(self.cols)}
        self.walkable: frozenset[Cell] = frozenset(cells - self.obstacles)
        protected = set(self.spawns) | {self.target} | set(self.unbuildables)
        self.buildable: frozenset[Cell] = frozenset(self.walkable - protected)

    def neighbors(self, cell: Cell):
        r, c = cell
        for dr, dc in _DIRS:
            n = (r + dr, c + dc)
            if n in self.walkable:
                yield n

    def evaluate(self, walls):
        walls = set(walls)
        dist = {self.target: 0}
        q = deque([self.target])
        while q:
            u = q.popleft()
            for v in self.neighbors(u):
                if v not in dist and v not in walls:
                    dist[v] = dist[u] + 1
                    q.append(v)
        per = tuple(dist.get(s) for s in self.spawns)
        if any(d is None for d in per):
            return None, per
        return min(per), per


def parse_map(path: str) -> GridMap:
    with open(path) as f:
        lines = [ln.rstrip("\n") for ln in f if ln.strip()]
    if not lines:
        raise ValueError("empty map file")
    rows, cols = len(lines), len(lines[0])
    if any(len(ln) != cols for ln in lines):
        raise ValueError("map is not rectangular")

    spawns: list[Cell] = []
    target: Cell | None = None
    obstacles, unbuildables, preset = set(), set(), set()
    for r, row in enumerate(lines):
        for c, ch in enumerate(row):
            if ch == "S":
                spawns.append((r, c))
            elif ch == "T":
                if target is not None:
                    raise ValueError("multiple T cells")
                target = (r, c)
            elif ch == "#":
                obstacles.add((r, c))
            elif ch == "X":
                unbuildables.add((r, c))
            elif ch == "W":
                preset.add((r, c))
            elif ch != ".":
                raise ValueError(f"unknown character {ch!r} at {(r, c)}")
    if not spawns:
        raise ValueError("no spawn (S) in map")
    if target is None:
        raise ValueError("no target (T) in map")

    g = GridMap(rows, cols, tuple(spawns), target,
                frozenset(obstacles), frozenset(unbuildables), frozenset(preset))

    maximin, per = g.evaluate(set())
    if maximin is None:
        bad = [s for s, d in zip(g.spawns, per) if d is None]
        raise ValueError(f"spawns unreachable even with zero walls: {bad}")
    return g

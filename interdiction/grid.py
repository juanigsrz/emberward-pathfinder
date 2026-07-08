"""Grid model for the interdiction problem: parsing, BFS, evaluation.

No solver dependencies — shared by the MILP master, the LNS driver and tests.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

Cell = tuple[int, int]

_DIRS = ((-1, 0), (1, 0), (0, -1), (0, 1))


def square2(anchor: Cell) -> tuple[Cell, Cell, Cell, Cell]:
    """The 2x2 square of cells whose top-left corner is `anchor`."""
    r, c = anchor
    return ((r, c), (r + 1, c), (r, c + 1), (r + 1, c + 1))


def tile2_decompose(walls) -> set[Cell]:
    """Anchors of the unique disjoint 2x2 tiling of `walls`.

    Raises ValueError if the cells are not tileable. Row-major greedy is
    exact here: the first uncovered wall cell must be its block's top-left
    corner, since the cells above and to the left of it are already covered.
    """
    walls = set(walls)
    anchors: set[Cell] = set()
    covered: set[Cell] = set()
    for v in sorted(walls):
        if v in covered:
            continue
        sq = square2(v)
        if not all(u in walls and u not in covered for u in sq):
            raise ValueError(f"walls are not a disjoint 2x2 tiling at {v}")
        anchors.add(v)
        covered.update(sq)
    return anchors


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

    def dist_field(self, walls) -> dict[Cell, int]:
        """BFS distances to target over walkable cells, excluding walls."""
        walls = set(walls)
        dist = {self.target: 0}
        q = deque([self.target])
        while q:
            u = q.popleft()
            for v in self.neighbors(u):
                if v not in dist and v not in walls:
                    dist[v] = dist[u] + 1
                    q.append(v)
        return dist

    def evaluate(self, walls):
        """(maximin, per-spawn distances); maximin None if any spawn cut off."""
        dist = self.dist_field(walls)
        per = tuple(dist.get(s) for s in self.spawns)
        if any(d is None for d in per):
            return None, per
        return min(per), per

    def shortest_path(self, walls, spawn: Cell, dist=None, rng=None):
        """One shortest spawn->target path as a cell list, or None.

        rng, if given, picks uniformly among predecessors on the
        shortest-path DAG — used to sample alternate optimal paths.
        """
        if dist is None:
            dist = self.dist_field(walls)
        if spawn not in dist:
            return None
        walls = set(walls)
        path = [spawn]
        cur = spawn
        while cur != self.target:
            nxts = [v for v in self.neighbors(cur)
                    if v not in walls and dist.get(v) == dist[cur] - 1]
            cur = rng.choice(nxts) if rng is not None else nxts[0]
            path.append(cur)
        return path

    def manhattan_parity(self, spawn: Cell) -> int:
        """Parity of every possible spawn->target path length (grid bipartite)."""
        return (abs(spawn[0] - self.target[0])
                + abs(spawn[1] - self.target[1])) % 2


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


def parse_solution(grid: GridMap, path: str) -> set[Cell]:
    """Read placed walls from a solution map file.

    Both conventions in maps/ are accepted: '#' (annealing/genetic solution
    files) and 'W' (integer_programming_file.py output). Only cells that are
    buildable in the base map count — base obstacles stay obstacles.
    """
    with open(path) as f:
        lines = [ln.rstrip("\n") for ln in f if ln.strip()]
    return {(r, c)
            for r, row in enumerate(lines)
            for c, ch in enumerate(row)
            if ch in "#W" and (r, c) in grid.buildable}


def write_solution(grid: GridMap, walls, path: str) -> None:
    walls = set(walls)
    with open(path, "w") as f:
        for r in range(grid.rows):
            for c in range(grid.cols):
                v = (r, c)
                if v in grid.spawns:
                    f.write("S")
                elif v == grid.target:
                    f.write("T")
                elif v in grid.obstacles:
                    f.write("#")
                elif v in walls:
                    f.write("W")
                elif v in grid.unbuildables:
                    f.write("X")
                else:
                    f.write(".")
            f.write("\n")

# Lazy-Cut MILP + LNS Interdiction Solver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Exact grid-interdiction solver (maximize the minimum spawn→nexus shortest path by placing walls) built as a lazy path-cut Gurobi master problem, wrapped in a large-neighborhood search for endless-scale maps, with a full-map bound run for gap reporting.

**Architecture:** New `interdiction/` Python package. `grid.py` holds the solver-free map model (parsing, BFS, evaluation). `master.py` is the row-generation Gurobi model (binary walls + per-spawn claimed-distance variables, continuous multi-source flow for connectivity, lazy shortest-path cuts added in a MIPSOL callback, persistent cut pool). `lns.py` re-optimizes windows around binding paths using the same master with outside cells fixed. `bound.py` + `cli.py` tie it together. Spec: `docs/superpowers/specs/2026-07-04-interdiction-milp-design.md`.

**Tech Stack:** Python 3.12 (`myenv/bin/python`), gurobipy 12.0.3 (full license), pytest.

## Global Constraints

- Interpreter for everything: `myenv/bin/python` (repo venv; has gurobipy 12.0.3). Never bare `python3`.
- Map format (from `integer_programming_file.py`): `S` spawn center, `T` nexus (exactly one), `#` permanent obstacle, `X` walkable-unbuildable, `W` preset wall (buildable), `.` free. Spawn/nexus 3x3 footprints are already encoded in map files as S/T center + X ring — the code treats them as single cells with the X ring unbuildable.
- Objective: maximin — maximize `min_k dist(spawn_k → target)`; distance 1 per orthogonal step.
- Every spawn must always stay connected to the target (flow constraints make infeasibility impossible).
- Existing scripts (`integer_programming*.py`, `ip_simlpified*.py`, `cp-sat.py`, `milp/`, `genetic/`, `annealing/`) are untouched baselines.
- Commits: subject + optional body. NO `Co-Authored-By` trailers (user preference).
- `U` (path-length upper bound) = `len(walkable) - 1` everywhere.

---

### Task 1: Package scaffold + map parsing

**Files:**
- Create: `interdiction/__init__.py`
- Create: `interdiction/grid.py`
- Create: `tests/__init__.py`
- Create: `tests/conftest.py`
- Create: `tests/test_grid.py`
- Create: `pytest.ini`

**Interfaces:**
- Produces: `GridMap` dataclass with fields `rows: int`, `cols: int`, `spawns: tuple[Cell, ...]`, `target: Cell`, `obstacles: frozenset[Cell]`, `unbuildables: frozenset[Cell]`, `preset_walls: frozenset[Cell]` and derived attributes `walkable: frozenset[Cell]`, `buildable: frozenset[Cell]`; `parse_map(path: str) -> GridMap`; `GridMap.neighbors(cell) -> Iterator[Cell]`. `Cell = tuple[int, int]` (row, col).

- [ ] **Step 1: Install pytest into the venv**

Run: `myenv/bin/python -m pip install pytest`
Expected: `Successfully installed ... pytest-8.x`

- [ ] **Step 2: Write failing tests**

`pytest.ini`:

```ini
[pytest]
testpaths = tests
```

`tests/__init__.py`: empty file.

`tests/conftest.py`:

```python
import textwrap

import pytest


@pytest.fixture
def make_map(tmp_path):
    """Write a dedented map string to a temp file, return its path."""
    def _make(text, name="m.txt"):
        f = tmp_path / name
        f.write_text(textwrap.dedent(text).strip() + "\n")
        return str(f)
    return _make
```

`tests/test_grid.py`:

```python
import pytest

from interdiction.grid import parse_map


def test_parse_symbols_and_derived_sets(make_map):
    g = parse_map(make_map("""
        S.X#.
        ..W.T
    """))
    assert (g.rows, g.cols) == (2, 5)
    assert g.spawns == ((0, 0),)
    assert g.target == (1, 4)
    assert g.obstacles == frozenset({(0, 3)})
    assert g.unbuildables == frozenset({(0, 2)})
    assert g.preset_walls == frozenset({(1, 2)})
    # walkable = everything except '#'
    assert g.walkable == frozenset(
        (r, c) for r in range(2) for c in range(5)) - {(0, 3)}
    # buildable = walkable minus spawns, target, unbuildables
    assert g.buildable == g.walkable - {(0, 0), (1, 4), (0, 2)}
    assert g.preset_walls <= g.buildable


def test_neighbors_respect_bounds_and_obstacles(make_map):
    g = parse_map(make_map("""
        S.#
        ..T
    """))
    assert set(g.neighbors((0, 0))) == {(0, 1), (1, 0)}
    assert set(g.neighbors((0, 1))) == {(0, 0), (1, 1)}  # (0,2) is '#'


@pytest.mark.parametrize("text,msg", [
    ("....\n.T..", "no spawn"),
    ("S...\n....", "no target"),
    ("S.T.\n.T..", "multiple T"),
    ("S?T.", "unknown character"),
    ("S#T", "unreachable"),
    ("S.T\n....", "rectangular"),
])
def test_parse_errors(make_map, text, msg):
    with pytest.raises(ValueError, match=msg):
        parse_map(make_map(text))
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `myenv/bin/python -m pytest tests/test_grid.py -v`
Expected: FAIL / ERROR with `ModuleNotFoundError: No module named 'interdiction'`

- [ ] **Step 4: Implement `interdiction/grid.py` (parsing half)**

`interdiction/__init__.py`: empty file.

`interdiction/grid.py`:

```python
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
```

Note: `parse_map` calls `g.evaluate`, implemented in Task 2. For this task add a
temporary minimal version to `GridMap` so the validation test passes (Task 2
replaces it with the tested implementation):

```python
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
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `myenv/bin/python -m pytest tests/test_grid.py -v`
Expected: all PASS

- [ ] **Step 6: Commit**

```bash
git add pytest.ini interdiction/ tests/
git commit -m "feat: interdiction package scaffold with map parsing"
```

---

### Task 2: BFS distance field, evaluator, path extraction

**Files:**
- Modify: `interdiction/grid.py`
- Test: `tests/test_grid.py` (append)

**Interfaces:**
- Consumes: `GridMap`, `parse_map` from Task 1.
- Produces: `GridMap.dist_field(walls: set[Cell]) -> dict[Cell, int]` (BFS distances to target, walls excluded); `GridMap.evaluate(walls) -> tuple[int | None, tuple[int | None, ...]]` (maximin, per-spawn distances; maximin `None` if any spawn disconnected); `GridMap.shortest_path(walls, spawn, dist=None, rng=None) -> list[Cell] | None` (spawn→target path; `rng` picks uniformly among shortest-path DAG successors, `rng=None` is deterministic-first); `GridMap.manhattan_parity(spawn) -> int`.

- [ ] **Step 1: Write failing tests**

Append to `tests/test_grid.py`:

```python
def test_dist_field_and_evaluate(make_map):
    g = parse_map(make_map("""
        S....
        .....
        ..#..
        .....
        ....T
    """))
    dist = g.dist_field(set())
    assert dist[g.target] == 0
    assert dist[(0, 0)] == 8
    assert (2, 2) not in dist          # obstacle not in field
    maximin, per = g.evaluate(set())
    assert (maximin, per) == (8, (8,))


def test_evaluate_with_walls_and_disconnect(make_map):
    g = parse_map(make_map("""
        S....
        .....
        T....
    """))
    assert g.evaluate(set())[0] == 2
    assert g.evaluate({(1, 0)})[0] == 4               # one detour
    assert g.evaluate({(1, 0), (1, 1)})[0] == 6       # longer detour
    maximin, per = g.evaluate({(1, 0), (0, 1)})       # spawn sealed off
    assert maximin is None and per == (None,)


def test_evaluate_multi_spawn_maximin(make_map):
    g = parse_map(make_map("""
        S...T
        .....
        S....
    """))
    maximin, per = g.evaluate(set())
    assert per == (4, 6)
    assert maximin == 4


def test_shortest_path_descends_dist_field(make_map):
    g = parse_map(make_map("""
        S....
        .....
        ..#..
        .....
        ....T
    """))
    path = g.shortest_path(set(), (0, 0))
    assert path[0] == (0, 0) and path[-1] == g.target
    assert len(path) - 1 == 8
    # consecutive cells adjacent
    for a, b in zip(path, path[1:]):
        assert abs(a[0] - b[0]) + abs(a[1] - b[1]) == 1
    assert g.shortest_path({(0, 1), (1, 0)}, (0, 0)) is None


def test_shortest_path_rng_sampling(make_map):
    import random
    g = parse_map(make_map("""
        S...
        ....
        ...T
    """))
    rng = random.Random(0)
    seen = {tuple(g.shortest_path(set(), (0, 0), rng=rng)) for _ in range(50)}
    assert len(seen) > 1                      # samples multiple shortest paths
    assert all(len(p) - 1 == 5 for p in seen)  # all optimal


def test_manhattan_parity(make_map):
    g = parse_map(make_map("""
        S...T
        .....
        S....
    """))
    assert g.manhattan_parity((0, 0)) == 0    # distance 4
    assert g.manhattan_parity((2, 0)) == 0    # distance 6
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `myenv/bin/python -m pytest tests/test_grid.py -v`
Expected: new tests FAIL with `AttributeError: ... no attribute 'dist_field'` (and friends)

- [ ] **Step 3: Implement**

In `interdiction/grid.py`, replace the temporary `evaluate` with:

```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `myenv/bin/python -m pytest tests/test_grid.py -v`
Expected: all PASS

- [ ] **Step 5: Commit**

```bash
git add interdiction/grid.py tests/test_grid.py
git commit -m "feat: BFS distance field, maximin evaluator, path sampling"
```

---

### Task 3: Solution file I/O

**Files:**
- Modify: `interdiction/grid.py`
- Test: `tests/test_grid.py` (append)

**Interfaces:**
- Consumes: `GridMap`, `parse_map`.
- Produces: `parse_solution(grid: GridMap, path: str) -> set[Cell]` (walls = cells marked `#` or `W` in the solution file that are buildable in the base map — both conventions exist in `maps/`); `write_solution(grid, walls, path)` (writes `S`/`T`/`#`/`W`/`X`/`.`, placed walls as `W`).

- [ ] **Step 1: Write failing tests**

Append to `tests/test_grid.py`:

```python
from interdiction.grid import parse_solution, write_solution


def test_solution_roundtrip(make_map, tmp_path):
    g = parse_map(make_map("""
        S....
        .....
        ....T
    """))
    walls = {(0, 2), (1, 2)}
    out = str(tmp_path / "sol.txt")
    write_solution(g, walls, out)
    assert (tmp_path / "sol.txt").read_text() == "S.W..\n..W..\n....T\n"
    assert parse_solution(g, out) == walls


def test_parse_solution_hash_convention_ignores_base_obstacles(make_map):
    base = parse_map(make_map("""
        S.#..
        .....
        ....T
    """, name="base.txt"))
    # annealing/genetic convention: placed walls written as '#'
    sol = make_map("""
        S.#.#
        ..#..
        ....T
    """, name="sol.txt")
    # (0,2) is a base obstacle -> not a wall; (0,4) and (1,2) are walls
    assert parse_solution(base, sol) == {(0, 4), (1, 2)}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `myenv/bin/python -m pytest tests/test_grid.py -v`
Expected: FAIL with `ImportError: cannot import name 'parse_solution'`

- [ ] **Step 3: Implement**

Append to `interdiction/grid.py` (module level):

```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `myenv/bin/python -m pytest tests/test_grid.py -v`
Expected: all PASS

- [ ] **Step 5: Commit**

```bash
git add interdiction/grid.py tests/test_grid.py
git commit -m "feat: solution file read/write for both wall conventions"
```

---

### Task 4: Master problem — lazy path-cut Gurobi model

**Files:**
- Create: `interdiction/master.py`
- Create: `tests/test_master.py`
- Modify: `tests/conftest.py` (brute-force helper)

**Interfaces:**
- Consumes: `GridMap` API from Tasks 1–2.
- Produces: `MasterSolver(grid, rng=None, gurobi_seed=0, output=False)` with attribute `cut_pool: set[tuple[int, tuple[Cell, ...]]]` and method `solve(*, free=None, fixed_walls=frozenset(), time_limit=None, warm_start=None) -> SolveResult`; `SolveResult` dataclass with `status: str` (`'OPTIMAL' | 'TIME_LIMIT' | 'INTERRUPTED' | 'NO_SOLUTION'`), `walls: set[Cell] | None`, `maximin: int | None`, `per_spawn: tuple | None`, `bound: float`.

The model, exactly as specified in the design doc:

- `y[v] ∈ {0,1}` for every buildable `v`; cells not in `free` are fixed via `lb == ub` (1 if in `fixed_walls`, else 0). `free=None` means all buildable cells free.
- Per spawn k: `z_k = 2*q_k + parity_k` with integer `q_k` (bipartite-grid parity), bounds `d0_k <= z_k <= U` where `d0_k` = zero-wall BFS distance and `U = len(walkable) - 1`.
- `z <= z_k` for all k; objective maximize `z`.
- Continuous flow `f` on directed arcs between adjacent walkable cells: net outflow 1 at each spawn, `-K` at target, 0 elsewhere; `inflow(v) <= K*(1 - y[v])` for buildable `v`. Guarantees connectivity; makes INFEASIBLE impossible.
- Callback on `MIPSOL`: BFS the incumbent walls; for every spawn whose claimed `z_k` exceeds its true distance, add lazy cuts `z_k <= len(P) + (U - len(P)) * sum(y[v] for v in P if buildable)` for the shortest path plus up to 3 rng-sampled alternates. Every cut also goes into `cut_pool`.
- `cut_pool` entries are added as ordinary constraints in every later `solve()` — subsolves get progressively stronger.
- Warm start: set `y`/`z_k`/`z` `.Start` from `warm_start` walls (must be feasible), and pre-seed pool with its binding paths. Zero-wall spawn paths are pooled at construction so the root is never blind.

- [ ] **Step 1: Add brute-force oracle to conftest**

Append to `tests/conftest.py`:

```python
from itertools import combinations


def brute_force_opt(grid):
    """Exhaustive optimum over all wall subsets. Only for tiny maps."""
    cells = sorted(grid.buildable)
    assert len(cells) <= 14, "brute force limited to 2^14 subsets"
    best_val, best_walls = -1, set()
    for r in range(len(cells) + 1):
        for combo in combinations(cells, r):
            val, _ = grid.evaluate(set(combo))
            if val is not None and val > best_val:
                best_val, best_walls = val, set(combo)
    return best_val, best_walls
```

- [ ] **Step 2: Write failing tests**

`tests/test_master.py`:

```python
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
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `myenv/bin/python -m pytest tests/test_master.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'interdiction.master'`

- [ ] **Step 4: Implement `interdiction/master.py`**

```python
"""Lazy path-cut master problem (Gurobi row generation).

max z
s.t. z <= z_k                                  for every spawn k
     z_k = 2 q_k + parity_k                    (bipartite grid parity)
     multi-source unit flow spawn->target with node capacity K(1-y)
     z_k <= len(P) + (U-len(P)) * sum_{v in P} y_v    (lazy, generated)

The path cuts are generated in a MIPSOL callback from BFS shortest paths on
the incumbent walls and kept in a persistent pool that is re-added as hard
constraints on every later solve (the LNS subsolves get stronger over time).
"""

from __future__ import annotations

from dataclasses import dataclass

import gurobipy as gp
from gurobipy import GRB

ALT_PATHS_PER_SPAWN = 3


@dataclass
class SolveResult:
    status: str                 # 'OPTIMAL' | 'TIME_LIMIT' | 'INTERRUPTED' | 'NO_SOLUTION'
    walls: set | None
    maximin: int | None
    per_spawn: tuple | None
    bound: float


_STATUS = {
    GRB.OPTIMAL: "OPTIMAL",
    GRB.TIME_LIMIT: "TIME_LIMIT",
    GRB.INTERRUPTED: "INTERRUPTED",
}


class MasterSolver:
    def __init__(self, grid, rng=None, gurobi_seed=0, output=False):
        self.grid = grid
        self.rng = rng
        self.gurobi_seed = gurobi_seed
        self.output = output
        self.U = len(grid.walkable) - 1
        # (spawn_index, path_tuple) — persists across solves
        self.cut_pool: set[tuple[int, tuple]] = set()
        self.cut_pool.update(self._paths_for(frozenset()))

    def _paths_for(self, walls):
        """Shortest path + alternates per spawn under `walls`, as pool entries."""
        dist = self.grid.dist_field(walls)
        out = []
        for k, s in enumerate(self.grid.spawns):
            if s not in dist:
                continue
            seen = set()
            for _ in range(1 + ALT_PATHS_PER_SPAWN):
                p = tuple(self.grid.shortest_path(walls, s, dist=dist,
                                                  rng=self.rng))
                if p not in seen:
                    seen.add(p)
                    out.append((k, p))
        return out

    def solve(self, *, free=None, fixed_walls=frozenset(), time_limit=None,
              warm_start=None) -> SolveResult:
        g = self.grid
        if free is None:
            free = g.buildable
        free = set(free) & g.buildable
        fixed_walls = (set(fixed_walls) & g.buildable) - free

        m = gp.Model("interdiction_master")
        m.Params.OutputFlag = 1 if self.output else 0
        m.Params.LazyConstraints = 1
        m.Params.Seed = self.gurobi_seed
        if time_limit is not None:
            m.Params.TimeLimit = max(time_limit, 0.01)

        # --- wall variables (fixed cells pinned via bounds) ---
        y = {}
        for v in sorted(g.buildable):
            if v in free:
                y[v] = m.addVar(vtype=GRB.BINARY, name=f"y_{v[0]}_{v[1]}")
            else:
                val = 1.0 if v in fixed_walls else 0.0
                y[v] = m.addVar(lb=val, ub=val, vtype=GRB.BINARY,
                                name=f"y_{v[0]}_{v[1]}")

        # --- claimed distances with parity encoding ---
        d0 = g.dist_field(set())
        zvars = []
        for k, s in enumerate(g.spawns):
            par = g.manhattan_parity(s)
            q = m.addVar(vtype=GRB.INTEGER, lb=(d0[s] - par) // 2,
                         ub=(self.U - par) // 2, name=f"q_{k}")
            zk = m.addVar(vtype=GRB.INTEGER, lb=d0[s], ub=self.U,
                          name=f"z_{k}")
            m.addConstr(zk == 2 * q + par)
            zvars.append(zk)
        z = m.addVar(vtype=GRB.INTEGER, lb=0, ub=self.U, name="z")
        for zk in zvars:
            m.addConstr(z <= zk)
        m.setObjective(z, GRB.MAXIMIZE)

        # --- connectivity flow (continuous; infeasibility impossible) ---
        K = len(g.spawns)
        spawn_set = set(g.spawns)
        arcs = [(u, v) for u in g.walkable for v in g.neighbors(u)]
        f = m.addVars(arcs, lb=0.0, name="f")
        for v in g.walkable:
            out_f = gp.quicksum(f[v, w] for w in g.neighbors(v))
            in_f = gp.quicksum(f[w, v] for w in g.neighbors(v))
            rhs = 1 if v in spawn_set else (-K if v == g.target else 0)
            m.addConstr(out_f - in_f == rhs)
        for v in g.buildable:
            in_f = gp.quicksum(f[w, v] for w in g.neighbors(v))
            m.addConstr(in_f <= K * (1 - y[v]))

        # --- path cuts ---
        def cut_expr(k, path):
            length = len(path) - 1
            hits = gp.quicksum(y[v] for v in path if v in y)
            return zvars[k] <= length + (self.U - length) * hits

        # --- warm start ---
        if warm_start is not None:
            ws_val, ws_per = g.evaluate(warm_start)
            assert ws_val is not None, "warm start disconnects a spawn"
            for v, var in y.items():
                var.Start = 1.0 if v in warm_start else 0.0
            for k, zk in enumerate(zvars):
                zk.Start = ws_per[k]
            z.Start = ws_val
            self.cut_pool.update(self._paths_for(warm_start))

        for k, path in self.cut_pool:
            m.addConstr(cut_expr(k, path))

        # --- lazy cut callback ---
        order = sorted(y)

        def cb(model, where):
            if where != GRB.Callback.MIPSOL:
                return
            yv = model.cbGetSolution([y[v] for v in order])
            walls = {v for v, val in zip(order, yv) if val > 0.5}
            dist = g.dist_field(walls)
            claims = model.cbGetSolution(zvars)
            violated = set()
            for k, s in enumerate(g.spawns):
                true_d = dist.get(s)
                assert true_d is not None, \
                    "spawn disconnected in incumbent — flow constraints broken"
                if claims[k] > true_d + 0.5:
                    violated.add(k)
            if not violated:
                return
            for k, p in self._paths_for(walls):
                if k in violated:
                    model.cbLazy(cut_expr(k, p))
                    self.cut_pool.add((k, p))

        m.optimize(cb)

        if m.Status == GRB.INFEASIBLE:
            m.computeIIS()
            m.write("master_infeasible.ilp")
            raise AssertionError(
                "master infeasible — impossible by construction, see .ilp")

        status = _STATUS.get(m.Status, str(m.Status))
        if m.SolCount == 0:
            return SolveResult("NO_SOLUTION", None, None, None, m.ObjBound)

        walls = {v for v, var in y.items() if var.X > 0.5}
        maximin, per = g.evaluate(walls)
        # callback guarantees incumbents never overclaim
        assert maximin is not None and round(m.ObjVal) <= maximin, \
            "incumbent overclaims shortest path — cut bug"
        return SolveResult(status, walls, maximin, per, m.ObjBound)
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `myenv/bin/python -m pytest tests/test_master.py -v`
Expected: all PASS (the 4x4 brute force takes a few seconds)

- [ ] **Step 6: Run the whole suite**

Run: `myenv/bin/python -m pytest -v`
Expected: all PASS

- [ ] **Step 7: Commit**

```bash
git add interdiction/master.py tests/test_master.py tests/conftest.py
git commit -m "feat: lazy path-cut Gurobi master with flow connectivity and parity"
```

---

### Task 5: Region interface — fixed cells, warm start, pool reuse

**Files:**
- Test: `tests/test_master.py` (append; the `solve()` keywords already exist — this task proves the region semantics the LNS relies on)

**Interfaces:**
- Consumes: `MasterSolver.solve(free=..., fixed_walls=..., warm_start=...)` from Task 4.
- Produces: verified guarantees for Task 6/7: fixed cells never flip; warm start is honored; `cut_pool` persists across solves.

- [ ] **Step 1: Write failing-or-passing tests (they must pass if Task 4 is correct — treat failures as Task 4 bugs, fix there)**

Append to `tests/test_master.py`:

```python
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
```

- [ ] **Step 2: Run tests**

Run: `myenv/bin/python -m pytest tests/test_master.py -v`
Expected: all PASS. Any failure here is a Task 4 defect — fix `master.py`, not the test.

- [ ] **Step 3: Commit**

```bash
git add tests/test_master.py
git commit -m "test: region solve semantics, pool persistence, warm-start validation"
```

---

### Task 6: Cut-validity property test

**Files:**
- Test: `tests/test_master.py` (append)

**Interfaces:**
- Consumes: `MasterSolver.cut_pool` entries `(k, path_tuple)`; `GridMap.evaluate`.

Property (design doc): every generated cut must hold for **every** feasible
wall configuration, with `z_k` at its maximum feasible value (the true
distance). If a cut can be violated by a feasible configuration, the master
would wrongly exclude it.

- [ ] **Step 1: Write the test**

Append to `tests/test_master.py`:

```python
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
```

- [ ] **Step 2: Run the test**

Run: `myenv/bin/python -m pytest tests/test_master.py::test_pool_cuts_valid_for_random_configs -v`
Expected: PASS. A failure means the cut coefficient or path collection is wrong in `master.py` — fix there.

- [ ] **Step 3: Commit**

```bash
git add tests/test_master.py
git commit -m "test: property check that pooled path cuts hold for feasible configs"
```

---

### Task 7: LNS driver

**Files:**
- Create: `interdiction/lns.py`
- Create: `tests/test_lns.py`

**Interfaces:**
- Consumes: `GridMap.evaluate/shortest_path`, `MasterSolver.solve(free=, fixed_walls=, time_limit=, warm_start=)`, `SolveResult`.
- Produces: `run_lns(grid, master, seed_walls, *, total_time, subsolve_time=30.0, rng) -> LNSResult`; `LNSResult` dataclass with `walls: set`, `maximin: int`, `per_spawn: tuple`, `trajectory: list[tuple[float, int, int]]` (elapsed s, iteration, maximin), `interrupted: bool`. Module constants `WINDOW_SIZES = (8, 12, 16)`, `STALL_LIMIT = 20`, `RANDOM_CENTER_PROB = 0.2`.

Design-doc semantics: window centers on a random cell of a binding spawn's
current shortest path (80%) or a random walkable cell (20%); sizes
round-robin over `WINDOW_SIZES`, forced to the largest after `STALL_LIMIT`
consecutive rejects until the next accept; accept only strict maximin
improvement; global evaluation every iteration.

- [ ] **Step 1: Write failing tests**

`tests/test_lns.py`:

```python
import random

from interdiction.grid import parse_map
from interdiction.lns import _pick_center, _window_cells, run_lns
from interdiction.master import MasterSolver


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
    # spawn (0,0) is binding (distance 4 < 6)
    for _ in range(20):
        center = _pick_center(g, set(), per, rng)
        assert center in g.walkable


def test_lns_improves_over_empty_walls():
    grid = parse_map("maps/basic.txt")
    baseline, _ = grid.evaluate(set())
    rng = random.Random(0)
    master = MasterSolver(grid, rng=rng)
    res = run_lns(grid, master, set(), total_time=25.0, subsolve_time=5.0,
                  rng=rng)
    assert res.maximin > baseline
    val, per = grid.evaluate(res.walls)          # invariant: no overclaim
    assert val == res.maximin and per == res.per_spawn
    assert res.trajectory[0][2] == baseline
    assert res.trajectory[-1][2] == res.maximin
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `myenv/bin/python -m pytest tests/test_lns.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'interdiction.lns'`

- [ ] **Step 3: Implement `interdiction/lns.py`**

```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `myenv/bin/python -m pytest tests/test_lns.py -v`
Expected: all PASS (`test_lns_improves_over_empty_walls` takes ~25 s)

- [ ] **Step 5: Commit**

```bash
git add interdiction/lns.py tests/test_lns.py
git commit -m "feat: LNS driver with binding-path windows over the exact master"
```

---

### Task 8: Bound phase

**Files:**
- Create: `interdiction/bound.py`
- Create: `tests/test_bound.py`

**Interfaces:**
- Consumes: `MasterSolver.solve(time_limit=, warm_start=)`.
- Produces: `run_bound(grid, master, incumbent_walls, time_limit) -> SolveResult` (full-map solve, MIP-started from the incumbent; `result.bound` is a valid global upper bound); `gap(incumbent: int, bound: float) -> float`.

- [ ] **Step 1: Write failing tests**

`tests/test_bound.py`:

```python
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
    assert res.bound >= res.maximin
    assert round(res.bound) == res.maximin


def test_gap():
    assert gap(10, 12.0) == 0.2
    assert gap(10, 10.0) == 0.0
    assert math.isinf(gap(0, 5.0))
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `myenv/bin/python -m pytest tests/test_bound.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'interdiction.bound'`

- [ ] **Step 3: Implement `interdiction/bound.py`**

```python
"""Full-map master run: valid global upper bound (and sometimes a better incumbent)."""

from __future__ import annotations

import math


def run_bound(grid, master, incumbent_walls, time_limit):
    """Solve the full map with the incumbent as MIP start.

    The returned result's `bound` (Gurobi ObjBound of the master, which is a
    relaxation of the true problem restricted to generated cuts) is a valid
    upper bound on the true maximin. `walls`/`maximin` may improve on the
    incumbent — the caller should keep the better of the two.
    """
    warm = set(incumbent_walls) if incumbent_walls else None
    return master.solve(time_limit=time_limit, warm_start=warm)


def gap(incumbent, bound):
    """Relative optimality gap; inf when there is no meaningful incumbent."""
    if not incumbent or incumbent <= 0:
        return math.inf
    return (bound - incumbent) / incumbent
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `myenv/bin/python -m pytest tests/test_bound.py -v`
Expected: all PASS

- [ ] **Step 5: Commit**

```bash
git add interdiction/bound.py tests/test_bound.py
git commit -m "feat: full-map bound phase with gap computation"
```

---

### Task 9: CLI

**Files:**
- Create: `interdiction/cli.py`
- Create: `interdiction/__main__.py`
- Create: `tests/test_cli.py`

**Interfaces:**
- Consumes: everything above.
- Produces: `main(argv: list[str] | None = None) -> int`; invocation `myenv/bin/python -m interdiction <map> [--seed SOL] [--time N] [--bound-frac F] [--subsolve-time N] [--rng-seed N] [--out PATH] [--exact] [--eval-only]`.

Behavior:
- Initial walls = map's `preset_walls` ∪ `parse_solution(--seed)` when given.
- `--eval-only`: print per-spawn distances + maximin of the initial walls, exit 0. No Gurobi touched.
- `--exact`: single full-map `master.solve(time_limit=--time)` (for small/mid maps), skip LNS.
- Default: LNS for `--time * (1 - bound_frac)` seconds, then bound run for `--time * bound_frac`, keep the better incumbent.
- Writes solution to `--out` (default: `<map stem>_milp_solution.txt` next to the map) and prints: per-spawn distances, maximin, wall count, upper bound, gap, LNS trajectory.
- Ctrl-C anywhere: current best is still written before exit (Gurobi turns the first SIGINT into status `INTERRUPTED`, which propagates as `interrupted`; a `KeyboardInterrupt` between solves is caught).

- [ ] **Step 1: Write failing tests**

`tests/test_cli.py`:

```python
import os

from interdiction.cli import main
from interdiction.grid import parse_map, parse_solution


def test_eval_only(make_map, capsys):
    path = make_map("""
        S....
        ..W..
        ....T
    """)
    assert main([path, "--eval-only"]) == 0
    out = capsys.readouterr().out
    assert "maximin" in out


def test_exact_solve_writes_solution(make_map, tmp_path, capsys):
    path = make_map("""
        S....
        .....
        ....T
    """)
    out_file = str(tmp_path / "sol.txt")
    assert main([path, "--exact", "--time", "30", "--out", out_file]) == 0
    assert os.path.exists(out_file)
    grid = parse_map(path)
    walls = parse_solution(grid, out_file)
    val, _ = grid.evaluate(walls)
    printed = capsys.readouterr().out
    assert f"maximin: {val}" in printed
    assert "bound:" in printed and "gap:" in printed


def test_lns_pipeline_smoke(make_map, tmp_path, capsys):
    path = make_map("""
        S......
        .......
        .......
        ......T
    """)
    out_file = str(tmp_path / "sol.txt")
    assert main([path, "--time", "20", "--bound-frac", "0.5",
                 "--subsolve-time", "4", "--rng-seed", "1",
                 "--out", out_file]) == 0
    grid = parse_map(path)
    walls = parse_solution(grid, out_file)
    val, _ = grid.evaluate(walls)
    baseline, _ = grid.evaluate(set())
    assert val >= baseline
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `myenv/bin/python -m pytest tests/test_cli.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'interdiction.cli'`

- [ ] **Step 3: Implement**

`interdiction/cli.py`:

```python
"""Command line entry point.

    myenv/bin/python -m interdiction maps/endless.txt \
        --seed maps/endless_annealing_solution.txt --time 3600
"""

from __future__ import annotations

import argparse
import os
import random
import sys

from interdiction.bound import gap, run_bound
from interdiction.grid import parse_map, parse_solution, write_solution
from interdiction.lns import run_lns
from interdiction.master import MasterSolver


def _summary(grid, walls, bound_val=None):
    val, per = grid.evaluate(walls)
    lines = [f"walls: {len(walls)}"]
    for s, d in zip(grid.spawns, per):
        lines.append(f"spawn {s}: distance {d}")
    lines.append(f"maximin: {val}")
    if bound_val is not None:
        lines.append(f"bound: {bound_val:.1f}")
        lines.append(f"gap: {gap(val, bound_val):.3f}")
    return "\n".join(lines)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="interdiction")
    p.add_argument("map")
    p.add_argument("--seed", help="solution file to seed initial walls from")
    p.add_argument("--time", type=float, default=3600.0,
                   help="total wall-clock budget in seconds")
    p.add_argument("--bound-frac", type=float, default=0.25,
                   help="fraction of budget for the final full-map bound run")
    p.add_argument("--subsolve-time", type=float, default=30.0)
    p.add_argument("--rng-seed", type=int, default=0)
    p.add_argument("--out", help="solution output path")
    p.add_argument("--exact", action="store_true",
                   help="single full-map exact solve, no LNS")
    p.add_argument("--eval-only", action="store_true",
                   help="evaluate seed/preset walls and exit")
    args = p.parse_args(argv)

    grid = parse_map(args.map)
    walls = set(grid.preset_walls)
    if args.seed:
        walls |= parse_solution(grid, args.seed)
    val, _ = grid.evaluate(walls)
    if val is None:
        print("error: initial walls disconnect a spawn", file=sys.stderr)
        return 2

    if args.eval_only:
        print(_summary(grid, walls))
        return 0

    rng = random.Random(args.rng_seed)
    master = MasterSolver(grid, rng=rng, gurobi_seed=args.rng_seed,
                          output=args.exact)
    out = args.out or os.path.splitext(args.map)[0] + "_milp_solution.txt"

    best, bound_val = walls, None
    try:
        if args.exact:
            res = master.solve(time_limit=args.time,
                               warm_start=walls or None)
            if res.walls is not None:
                best = res.walls
            bound_val = res.bound
        else:
            lns = run_lns(grid, master, walls,
                          total_time=args.time * (1 - args.bound_frac),
                          subsolve_time=args.subsolve_time, rng=rng)
            best = lns.walls
            for elapsed, it, v in lns.trajectory:
                print(f"[lns] t={elapsed:7.1f}s iter={it:4d} maximin={v}")
            if not lns.interrupted:
                bres = run_bound(grid, master, best,
                                 time_limit=args.time * args.bound_frac)
                bound_val = bres.bound
                if bres.maximin is not None and \
                        bres.maximin > grid.evaluate(best)[0]:
                    best = bres.walls
    except KeyboardInterrupt:
        print("\ninterrupted — writing best solution so far", file=sys.stderr)

    write_solution(grid, best, out)
    print(_summary(grid, best, bound_val))
    print(f"solution written to {out}")
    return 0
```

`interdiction/__main__.py`:

```python
import sys

from interdiction.cli import main

sys.exit(main())
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `myenv/bin/python -m pytest tests/test_cli.py -v`
Expected: all PASS

- [ ] **Step 5: Run the full suite**

Run: `myenv/bin/python -m pytest -v`
Expected: all PASS

- [ ] **Step 6: Commit**

```bash
git add interdiction/cli.py interdiction/__main__.py tests/test_cli.py
git commit -m "feat: CLI with eval-only, exact, and LNS+bound pipelines"
```

---

### Task 10: Benchmarks vs existing solutions (manual, long-running)

**Files:**
- Modify: `README.md`

No new code. Verifies the spec's success criterion. These runs are minutes to
an hour — run them manually, not under pytest.

- [ ] **Step 1: Baseline maximin of the existing solutions**

```bash
myenv/bin/python -m interdiction maps/endless.txt \
    --seed maps/endless_annealing_solution.txt --eval-only
myenv/bin/python -m interdiction maps/endless.txt \
    --seed maps/endless_genetic_solution.txt --eval-only
```

Record both maximin values — these are the numbers to beat.

- [ ] **Step 2: Sanity check the exact engine on basic.txt**

```bash
myenv/bin/python -m interdiction maps/basic.txt --exact --time 120
```

Expected: `OPTIMAL`-quality output in well under the budget, `gap: 0.000`, and
an objective at least matching a run of the old
`myenv/bin/python integer_programming_file.py maps/basic.txt` (which needed
minutes for weaker results).

- [ ] **Step 3: Mid-size benchmark**

```bash
myenv/bin/python -m interdiction maps/smaller_endless.txt --time 900 --rng-seed 1
```

Record maximin, bound, gap.

- [ ] **Step 4: Endless benchmark (the success criterion)**

```bash
myenv/bin/python -m interdiction maps/endless.txt \
    --seed maps/endless_annealing_solution.txt --time 3600 --rng-seed 1
```

Success: final maximin strictly greater than both Step 1 values; bound and
gap reported. If it fails, the LNS knobs (`--subsolve-time`, window sizes,
`STALL_LIMIT`) are the first things to tune — larger subsolve times help once
the cut pool has grown.

- [ ] **Step 5: Update README**

Add to `README.md` under the MILP section: a short paragraph describing the
`interdiction/` package (lazy path-cut master + LNS + bound run), the CLI
invocation, and a results table with Step 1–4 numbers (map, method, maximin,
bound, gap, wall-clock).

- [ ] **Step 6: Commit**

```bash
git add README.md
git commit -m "docs: interdiction solver usage and benchmark results"
```

# Portal-Contraction Window Subsolver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the full-map LNS window subsolves with an exact portal contraction (small weighted graph per window) solved by a dedicated Gurobi lazy-cut model with optional corridor-structure constraints.

**Architecture:** `contract.py` collapses the fixed outside of a window into portal-to-portal shortest-distance edges, producing a ~300-node weighted graph whose shortest paths exactly match the full grid. `window_master.py` runs lazy path cuts over that graph (Dijkstra callback instead of 3,600-cell BFS). `lns.py` switches subsolves to this pair; `master.py` remains the exact full-map engine for `--exact` and the bound phase. Spec: `docs/superpowers/specs/2026-07-04-portal-contraction-design.md`.

**Tech Stack:** Python 3.12 (`myenv/bin/python`), gurobipy 12.0.3, pytest.

## Global Constraints

- Interpreter: `myenv/bin/python` always.
- Wall model stays 1x1 cells (pieces are out of scope per spec).
- Corridor hints: `sum(y) >= 1` and `sum(y) <= 3` over each 2x2 square whose four cells are all free window cells; LNS default ON, `--exact`/bound runs never use hints.
- With hints ON, INFEASIBLE is a normal outcome (`NO_SOLUTION` result); with hints OFF it is a bug (assert).
- Contraction must be exact: contracted shortest distance == full-grid BFS distance for every wall assignment. The LNS asserts this on every candidate.
- Commits: subject + optional body, NO `Co-Authored-By` trailers.
- Existing full-map master (`interdiction/master.py`) is not modified by this plan.

---

### Task 1: `contract.py` — exact portal contraction

**Files:**
- Create: `interdiction/contract.py`
- Test: `tests/test_contract.py`

**Interfaces:**
- Consumes: `GridMap` (fields `spawns`, `target`, `walkable`, `buildable`; methods `neighbors`, `dist_field`, `evaluate`) from `interdiction/grid.py`.
- Produces: `contract(grid, window_cells, outside_walls) -> ContractedWindow`; `ContractedWindow` with attributes `grid`, `window: frozenset[Cell]`, `free: frozenset[Cell]` (window ∩ buildable), `adj: dict[Cell, list[tuple[Cell, int]]]` (symmetric weighted adjacency) and method `dijkstra(window_walls) -> list[tuple[int | None, list[Cell] | None]]` — one `(distance, window_cells_on_path)` entry per spawn, in `grid.spawns` order; `(None, None)` when disconnected.

- [ ] **Step 1: Write the failing tests**

`tests/test_contract.py`:

```python
import random

from interdiction.contract import contract
from interdiction.grid import parse_map


def _window(grid, r0, c0, size):
    return {(r, c)
            for r in range(max(r0, 0), min(r0 + size, grid.rows))
            for c in range(max(c0, 0), min(c0 + size, grid.cols))}


def _random_map(make_map, rng, i):
    while True:
        rows, cols = rng.randint(4, 8), rng.randint(4, 8)
        cells = [["." for _ in range(cols)] for _ in range(rows)]
        for r in range(rows):
            for c in range(cols):
                if rng.random() < 0.12:
                    cells[r][c] = "#"
        cells[0][0] = "S"
        if rng.random() < 0.4:
            cells[rows - 1][0] = "S"
        cells[rows - 1][cols - 1] = "T"
        text = "\n".join("".join(row) for row in cells)
        try:
            return parse_map(make_map(text, name=f"case{i}.txt"))
        except ValueError:
            continue    # unreachable spawn etc. — draw again


def test_contraction_exactness_property(make_map):
    """THE invariant: contracted Dijkstra == full-grid BFS, always."""
    rng = random.Random(42)
    checked = 0
    for i in range(150):
        grid = _random_map(make_map, rng, i)
        window = _window(grid, rng.randint(-1, grid.rows - 2),
                         rng.randint(-1, grid.cols - 2), rng.randint(2, 4))
        free = window & grid.buildable
        walls = {v for v in grid.buildable if rng.random() < 0.25}
        outside_walls = walls - free
        window_walls = walls & free

        cw = contract(grid, window, outside_walls)
        got = cw.dijkstra(window_walls)
        dist = grid.dist_field(walls)
        for k, s in enumerate(grid.spawns):
            expected = dist.get(s)
            assert got[k][0] == expected, (
                f"case {i}: spawn {s} contracted={got[k][0]} bfs={expected}")
            checked += 1
    assert checked >= 150


def test_contraction_path_cells_are_window_cells_of_shortest_path(make_map):
    grid = parse_map(make_map("""
        S....
        .....
        .....
        ....T
    """))
    window = _window(grid, 1, 1, 2)
    cw = contract(grid, window, set())
    (d, cells), = cw.dijkstra(frozenset())
    assert d == 7
    assert all(v in window for v in cells)
    # blocking every reported window cell must not break exactness
    got = cw.dijkstra(frozenset(cells))
    assert got[0][0] == grid.dist_field(set(cells))[grid.spawns[0]]


def test_contraction_with_target_inside_window(make_map):
    grid = parse_map(make_map("""
        S....
        .....
        ....T
    """))
    window = _window(grid, 1, 3, 2)          # covers T
    cw = contract(grid, window, set())
    (d, _), = cw.dijkstra(frozenset())
    assert d == 6


def test_contraction_reports_disconnect(make_map):
    grid = parse_map(make_map("""
        S..
        ...
        ..T
    """))
    window = _window(grid, 0, 0, 3)
    cw = contract(grid, window, set())
    got = cw.dijkstra(frozenset({(0, 1), (1, 0)}))   # seals the spawn
    assert got[0] == (None, None)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `myenv/bin/python -m pytest tests/test_contract.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'interdiction.contract'`

- [ ] **Step 3: Implement `interdiction/contract.py`**

```python
"""Exact portal contraction of a rectangular window against a fixed outside.

Any spawn->target path alternates outside segments (through the fixed part
of the map) and inside segments (through the window). Outside segments are
collapsed into weighted edges between "terminals" — portals (outside cells
adjacent to the window), spawns outside the window, and the target if
outside. Window cells stay explicit. Shortest distances in the contracted
graph therefore equal full-grid BFS distances for every assignment of walls
to window cells.
"""

from __future__ import annotations

import heapq
from collections import deque
from dataclasses import dataclass, field

Cell = tuple[int, int]


@dataclass
class ContractedWindow:
    grid: object
    window: frozenset
    free: frozenset                 # window cells that may hold a wall
    adj: dict = field(default_factory=dict)   # cell -> [(cell, weight)]

    def dijkstra(self, window_walls):
        """Per-spawn (distance, window cells on one shortest path).

        (None, None) when the spawn cannot reach the target. Only window
        cells can be walls — outside walls are already baked into the graph.
        """
        blocked = set(window_walls)
        out = []
        for s in self.grid.spawns:
            dist, prev = self._from(s, blocked)
            d = dist.get(self.grid.target)
            if d is None:
                out.append((None, None))
                continue
            cells = []
            cur = self.grid.target
            while cur != s:
                if cur in self.window:
                    cells.append(cur)
                cur = prev[cur]
            if s in self.window:
                cells.append(s)
            out.append((d, cells))
        return out

    def _from(self, src, blocked):
        dist = {src: 0}
        prev = {}
        pq = [(0, src)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist.get(u, float("inf")):
                continue
            for v, w in self.adj.get(u, ()):
                if v in blocked:
                    continue
                nd = d + w
                if nd < dist.get(v, float("inf")):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))
        return dist, prev


def _bfs(grid, src, allowed):
    dist = {src: 0}
    q = deque([src])
    while q:
        u = q.popleft()
        for v in grid.neighbors(u):
            if v in allowed and v not in dist:
                dist[v] = dist[u] + 1
                q.append(v)
    return dist


def contract(grid, window_cells, outside_walls) -> ContractedWindow:
    window = frozenset(window_cells)
    inside = window & grid.walkable
    free = window & grid.buildable
    outside_walls = frozenset(outside_walls) - free
    out_open = grid.walkable - window - outside_walls

    portals = {n for v in inside for n in grid.neighbors(v) if n in out_open}

    # terminals: endpoints of outside path segments
    spawn_out = {s for s in grid.spawns if s not in window}
    terminals = set(portals) | spawn_out
    if grid.target not in window:
        terminals.add(grid.target)

    dout = {t: _bfs(grid, t, out_open) for t in terminals}

    adj: dict = {}

    def add(a, b, w):
        adj.setdefault(a, []).append((b, w))
        adj.setdefault(b, []).append((a, w))

    for v in inside:
        for n in grid.neighbors(v):
            if n in inside:
                if v < n:
                    add(v, n, 1)
            elif n in portals:
                add(v, n, 1)

    terms = sorted(terminals)
    for i, a in enumerate(terms):
        for b in terms[i + 1:]:
            if a in spawn_out and b in spawn_out:
                continue            # a path never runs spawn -> spawn
            d = dout[a].get(b)
            if d:
                add(a, b, d)

    return ContractedWindow(grid, window, free, adj)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `myenv/bin/python -m pytest tests/test_contract.py -v`
Expected: all PASS (property test runs 150 random cases, a few seconds)

- [ ] **Step 5: Run the whole suite, commit**

Run: `myenv/bin/python -m pytest 2>&1 | tail -1` — expected: all pass.

```bash
git add interdiction/contract.py tests/test_contract.py
git commit -m "feat: exact portal contraction for LNS windows"
```

---

### Task 2: `window_master.py` — lazy cuts on the contracted graph

**Files:**
- Create: `interdiction/window_master.py`
- Test: `tests/test_window_master.py`

**Interfaces:**
- Consumes: `ContractedWindow` (`.grid`, `.free`, `.adj`, `.dijkstra`), `contract` from Task 1; `SolveResult` and `_STATUS` from `interdiction/master.py`; `MasterSolver` (equivalence tests only).
- Produces: `solve_window(cw, *, time_limit=None, warm_start=None, corridor_hint=True, gurobi_seed=0, output=False) -> SolveResult`. NOTE: `result.walls` contains **window walls only** — the caller merges them with the outside.

- [ ] **Step 1: Write the failing tests**

`tests/test_window_master.py`:

```python
import random

import pytest

from interdiction.contract import contract
from interdiction.grid import parse_map
from interdiction.master import MasterSolver
from interdiction.window_master import solve_window


def _window(grid, r0, c0, size):
    return {(r, c)
            for r in range(max(r0, 0), min(r0 + size, grid.rows))
            for c in range(max(c0, 0), min(c0 + size, grid.cols))}


@pytest.mark.parametrize("r0,c0,size", [(0, 0, 3), (1, 1, 3), (0, 2, 4)])
def test_window_solve_matches_master_region_solve(make_map, r0, c0, size):
    grid = parse_map(make_map("""
        S.....
        ......
        ......
        .....T
    """))
    window = _window(grid, r0, c0, size)
    free = window & grid.buildable

    master = MasterSolver(grid, rng=random.Random(0))
    ref = master.solve(free=free, fixed_walls=set(), time_limit=60)

    cw = contract(grid, window, outside_walls=set())
    got = solve_window(cw, time_limit=60, corridor_hint=False)

    assert got.status == "OPTIMAL" and ref.status == "OPTIMAL"
    assert got.maximin == ref.maximin
    # window walls actually achieve the claim on the real grid
    val, _ = grid.evaluate(got.walls)
    assert val == got.maximin


def test_window_solve_with_outside_walls(make_map):
    grid = parse_map(make_map("""
        S.....
        ......
        ......
        .....T
    """))
    outside = {(1, 4), (2, 4)}
    window = _window(grid, 0, 0, 3)
    free = window & grid.buildable

    master = MasterSolver(grid, rng=random.Random(0))
    ref = master.solve(free=free, fixed_walls=outside, time_limit=60)

    cw = contract(grid, window, outside_walls=outside)
    got = solve_window(cw, time_limit=60, corridor_hint=False)

    assert got.maximin == ref.maximin
    val, _ = grid.evaluate(set(got.walls) | outside)
    assert val == got.maximin


def test_hinted_solution_respects_corridor_constraints(make_map):
    grid = parse_map(make_map("""
        S.....
        ......
        ......
        ......
        .....T
    """))
    window = _window(grid, 0, 0, 6)
    cw = contract(grid, window, set())
    res = solve_window(cw, time_limit=60, corridor_hint=True)
    assert res.walls is not None
    for (r, c) in sorted(cw.free):
        square = [(r, c), (r + 1, c), (r, c + 1), (r + 1, c + 1)]
        if all(v in cw.free for v in square):
            n = sum(1 for v in square if v in res.walls)
            assert 1 <= n <= 3


def test_hints_never_beat_unhinted(make_map):
    grid = parse_map(make_map("""
        S....
        .....
        ....T
    """))
    window = _window(grid, 0, 0, 5)
    cw = contract(grid, window, set())
    unhinted = solve_window(cw, time_limit=60, corridor_hint=False)
    hinted = solve_window(cw, time_limit=60, corridor_hint=True)
    assert unhinted.status == "OPTIMAL"
    if hinted.walls is not None:
        assert hinted.maximin <= unhinted.maximin


def test_warm_start_accepted(make_map):
    grid = parse_map(make_map("""
        S....
        .....
        ....T
    """))
    window = _window(grid, 0, 0, 5)
    cw = contract(grid, window, set())
    ref = solve_window(cw, time_limit=60, corridor_hint=False)
    again = solve_window(cw, time_limit=60, corridor_hint=False,
                         warm_start=ref.walls)
    assert again.maximin == ref.maximin
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `myenv/bin/python -m pytest tests/test_window_master.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'interdiction.window_master'`

- [ ] **Step 3: Implement `interdiction/window_master.py`**

```python
"""Gurobi lazy-cut solver over a contracted window.

Same cut logic as the full-map master, but the graph has ~10^2 nodes with
weighted outside edges, so the callback runs Dijkstra instead of a
3,600-cell BFS and cuts touch only window cells. Optional corridor hints
(no fully-open and no fully-walled 2x2 square) prune toward maze-like
solutions; with hints the model may be INFEASIBLE, which callers treat as
"no improvement in this window".
"""

from __future__ import annotations

import gurobipy as gp
from gurobipy import GRB

from interdiction.master import SolveResult, _STATUS


def solve_window(cw, *, time_limit=None, warm_start=None, corridor_hint=True,
                 gurobi_seed=0, output=False) -> SolveResult:
    g = cw.grid
    U = len(g.walkable) - 1
    K = len(g.spawns)
    spawn_set = set(g.spawns)

    m = gp.Model("window_master")
    m.Params.OutputFlag = 1 if output else 0
    m.Params.LazyConstraints = 1
    m.Params.Seed = gurobi_seed
    m.Params.MIPFocus = 1
    m.Params.NodefileStart = 0.5
    m.Params.SoftMemLimit = 8
    if time_limit is not None:
        m.Params.TimeLimit = max(time_limit, 0.01)

    y = {v: m.addVar(vtype=GRB.BINARY, name=f"y_{v[0]}_{v[1]}")
         for v in sorted(cw.free)}

    base = cw.dijkstra(frozenset())
    zvars = []
    for k, s in enumerate(g.spawns):
        d_open = base[k][0]
        assert d_open is not None, \
            "spawn cannot reach target even with the window fully open"
        par = g.manhattan_parity(s)
        q = m.addVar(vtype=GRB.INTEGER, lb=(d_open - par) // 2,
                     ub=(U - par) // 2, name=f"q_{k}")
        zk = m.addVar(vtype=GRB.INTEGER, lb=d_open, ub=U, name=f"z_{k}")
        m.addConstr(zk == 2 * q + par)
        zvars.append(zk)
    z = m.addVar(vtype=GRB.INTEGER, lb=0, ub=U, name="z")
    for zk in zvars:
        m.addConstr(z <= zk)
    m.setObjective(z, GRB.MAXIMIZE)

    # connectivity flow on the contracted graph; outside edges uncapacitated
    arcs = [(u, v) for u in cw.adj for v, _w in cw.adj[u]]
    f = m.addVars(arcs, lb=0.0, name="f")
    for u in cw.adj:
        out_f = gp.quicksum(f[u, v] for v, _w in cw.adj[u])
        in_f = gp.quicksum(f[v, u] for v, _w in cw.adj[u])
        rhs = 1 if u in spawn_set else (-K if u == g.target else 0)
        m.addConstr(out_f - in_f == rhs)
    for v, var in y.items():
        in_f = gp.quicksum(f[w_, v] for w_, _w in cw.adj[v])
        m.addConstr(in_f <= K * (1 - var))

    if corridor_hint:
        for (r, c) in sorted(cw.free):
            square = [(r, c), (r + 1, c), (r, c + 1), (r + 1, c + 1)]
            if all(v in cw.free for v in square):
                total = gp.quicksum(y[v] for v in square)
                m.addConstr(total >= 1)
                m.addConstr(total <= 3)

    if warm_start is not None:
        ws = set(warm_start) & cw.free
        for v, var in y.items():
            var.Start = 1.0 if v in ws else 0.0
        ws_res = cw.dijkstra(ws)
        if all(d is not None for d, _ in ws_res):
            for k, (d, _cells) in enumerate(ws_res):
                zvars[k].Start = d
            z.Start = min(d for d, _cells in ws_res)

    order = sorted(y)

    def cut_expr(k, cells, length):
        hits = gp.quicksum(y[v] for v in cells if v in y)
        return zvars[k] <= length + (U - length) * hits

    def cb(model, where):
        if where != GRB.Callback.MIPSOL:
            return
        yv = model.cbGetSolution([y[v] for v in order])
        walls = {v for v, val in zip(order, yv) if val > 0.5}
        res = cw.dijkstra(walls)
        claims = model.cbGetSolution(zvars)
        for k, (true_d, cells) in enumerate(res):
            assert true_d is not None, \
                "spawn disconnected in incumbent — flow constraints broken"
            if claims[k] > true_d + 0.5:
                model.cbLazy(cut_expr(k, cells, true_d))

    m.optimize(cb)

    if m.Status == GRB.INFEASIBLE:
        m.dispose()
        if corridor_hint:
            return SolveResult("NO_SOLUTION", None, None, None, float("-inf"))
        raise AssertionError(
            "window master infeasible without corridor hints — impossible")

    status = _STATUS.get(m.Status, str(m.Status))
    bound = m.ObjBound
    if m.SolCount == 0:
        m.dispose()
        return SolveResult("NO_SOLUTION", None, None, None, bound)

    walls = {v for v, var in y.items() if var.X > 0.5}
    obj_val = m.ObjVal
    m.dispose()

    res = cw.dijkstra(walls)
    per = tuple(d for d, _cells in res)
    assert all(d is not None for d in per)
    maximin = min(per)
    assert round(obj_val) <= maximin, \
        "window incumbent overclaims shortest path — cut bug"
    return SolveResult(status, walls, maximin, per, bound)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `myenv/bin/python -m pytest tests/test_window_master.py -v`
Expected: all PASS

- [ ] **Step 5: Run the whole suite, commit**

Run: `myenv/bin/python -m pytest 2>&1 | tail -1` — expected: all pass.

```bash
git add interdiction/window_master.py tests/test_window_master.py
git commit -m "feat: contracted-window lazy-cut solver with corridor hints"
```

---

### Task 3: Rewire LNS and CLI

**Files:**
- Modify: `interdiction/lns.py`
- Modify: `interdiction/cli.py`
- Modify: `tests/test_lns.py`
- Modify: `tests/test_cli.py`

**Interfaces:**
- Consumes: `contract`, `solve_window` from Tasks 1–2.
- Produces: `run_lns(grid, seed_walls, *, total_time, subsolve_time=15.0, rng, corridor_hint=True, window_sizes=WINDOW_SIZES) -> LNSResult` — **the `master` parameter is gone**; CLI flags `--no-corridor-hint`, `--window-sizes`, `--subsolve-time` default 15.

- [ ] **Step 1: Update tests first**

In `tests/test_lns.py`, replace the whole file with:

```python
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
```

In `tests/test_cli.py`, `test_lns_pipeline_smoke` gains window-size and hint
flags to exercise the parsing (body otherwise unchanged):

```python
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
                 "--window-sizes", "6,8", "--no-corridor-hint",
                 "--out", out_file]) == 0
    grid = parse_map(path)
    walls = parse_solution(grid, out_file)
    val, _ = grid.evaluate(walls)
    baseline, _ = grid.evaluate(set())
    assert val >= baseline
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `myenv/bin/python -m pytest tests/test_lns.py tests/test_cli.py -v`
Expected: `test_lns_*` FAIL with `TypeError` (old `run_lns` signature takes `master`), CLI smoke FAILS on unknown `--window-sizes` flag.

- [ ] **Step 3: Rewrite `interdiction/lns.py`**

Replace `run_lns` (keep `LNSResult`, `_window_cells`, `_pick_center`, constants; change `WINDOW_SIZES`):

```python
"""Large-neighborhood search: exact contracted-window re-optimization."""

from __future__ import annotations

import time
from dataclasses import dataclass, field

from interdiction.contract import contract
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
            corridor_hint=True, window_sizes=WINDOW_SIZES):
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
            size = window_sizes[-1]
        else:
            size = window_sizes[(it - 1) % len(window_sizes)]
        center = _pick_center(grid, result.walls, result.per_spawn, rng)
        window = _window_cells(grid, center, size)
        free = window & grid.buildable
        outside_walls = result.walls - free

        cw = contract(grid, window, outside_walls)
        res = solve_window(cw, time_limit=min(subsolve_time, remaining),
                           warm_start=result.walls & free,
                           corridor_hint=corridor_hint)
        if res.status == "INTERRUPTED":
            result.interrupted = True
        if res.walls is not None:
            candidate = outside_walls | res.walls
            val, per = grid.evaluate(candidate)
            # contraction exactness: BFS must agree with the contracted claim
            assert val == res.maximin and per == res.per_spawn, \
                "contracted claim disagrees with BFS — contraction bug"
            if val > result.maximin:
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
```

- [ ] **Step 4: Update `interdiction/cli.py`**

Changes only — argparse block gains:

```python
    p.add_argument("--subsolve-time", type=float, default=15.0)
    p.add_argument("--no-corridor-hint", action="store_true",
                   help="disable corridor-structure constraints in LNS windows")
    p.add_argument("--window-sizes", default="12,16,20",
                   help="comma-separated LNS window sizes")
```

(the existing `--subsolve-time` line changes its default from 30.0 to 15.0)

and the LNS call becomes:

```python
            window_sizes = tuple(
                int(x) for x in args.window_sizes.split(","))
            lns = run_lns(grid, walls,
                          total_time=args.time * (1 - args.bound_frac),
                          subsolve_time=args.subsolve_time, rng=rng,
                          corridor_hint=not args.no_corridor_hint,
                          window_sizes=window_sizes)
```

`MasterSolver` stays exactly where it is — `--exact` and `run_bound` still
use it.

- [ ] **Step 5: Run the full suite**

Run: `myenv/bin/python -m pytest -v 2>&1 | tail -15`
Expected: all PASS (LNS smoke tests take ~40 s total)

- [ ] **Step 6: Commit**

```bash
git add interdiction/lns.py interdiction/cli.py tests/test_lns.py tests/test_cli.py
git commit -m "feat: LNS windows via portal contraction with corridor hints"
```

---

### Task 4: Probe + benchmarks + README (manual, long-running)

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Callback-cost probe (same window as the v1 probe)**

```bash
myenv/bin/python - <<'EOF'
import random, time
from interdiction.grid import parse_map, parse_solution
from interdiction.contract import contract
from interdiction.window_master import solve_window
from interdiction.lns import _window_cells

grid = parse_map("maps/endless.txt")
seed = parse_solution(grid, "maps/endless_annealing_solution.txt")
window = _window_cells(grid, (25, 45), 16)
free = window & grid.buildable
t0 = time.monotonic()
cw = contract(grid, window, seed - free)
print(f"contract: {time.monotonic()-t0:.2f}s nodes={len(cw.adj)}")
t0 = time.monotonic()
res = solve_window(cw, time_limit=30, warm_start=seed & free, output=True)
print(f"solve: {time.monotonic()-t0:.1f}s status={res.status} maximin={res.maximin}")
EOF
```

Success signal vs v1 (21.5 s callback / 26k MIPSOL calls / gap stuck at
127%): "time in user-callback" under ~2 s, or the window closes (OPTIMAL /
proven no-improvement) before the limit.

- [ ] **Step 2: smaller_endless from empty, 15 min**

```bash
myenv/bin/python -m interdiction maps/smaller_endless.txt --time 900 --rng-seed 1 \
    --out maps/smaller_endless_milp_solution.txt
```

Expected: maximin ≥ 172 (v1's number). Record value and trajectory.

- [ ] **Step 3: endless from annealing seed, 60 min (the real test)**

```bash
myenv/bin/python -m interdiction maps/endless.txt \
    --seed maps/endless_annealing_solution.txt --time 3600 --rng-seed 1 \
    --out maps/endless_milp_solution.txt
```

Record: final maximin (beat 1408?), iterations completed (v1 managed ~90/h),
bound, gap.

- [ ] **Step 4: endless from empty with hints, 60 min (can corridors build a maze from scratch?)**

```bash
myenv/bin/python -m interdiction maps/endless.txt --time 3600 --rng-seed 1 \
    --out maps/endless_scratch_milp_solution.txt
```

Record final maximin vs annealing's 1408.

- [ ] **Step 5: Update README**

Extend the `interdiction/` section: portal contraction + corridor hints
paragraph (windows now solve a ~300-node contracted graph; corridor
constraints encode the human maze intuition; hints are heuristic and never
applied to `--exact`/bound), new flags, and update the results table with
Steps 2–4 numbers (map, mode, maximin, bound, wall-clock, plus
windows-per-hour compared to v1).

- [ ] **Step 6: Commit**

```bash
git add README.md
git commit -m "docs: portal-contraction results and corridor-hint flags"
```

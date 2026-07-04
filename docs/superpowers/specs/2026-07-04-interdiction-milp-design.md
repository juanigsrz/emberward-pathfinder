# Grid Shortest-Path Interdiction — Lazy-Cut MILP + LNS Design

Date: 2026-07-04
Status: approved

## Problem

Network interdiction on a square grid (Emberward-style tower defense). Enemies
spawn at one or more spawn points and walk orthogonally along the shortest path
to the nexus. The player places walls on buildable cells (unlimited budget) to
maximize that shortest path, but every spawn must always retain a path to the
nexus. Spawns and the nexus occupy 3x3 footprints: center `S`/`T` plus an `X`
ring (walkable, unbuildable). `#` cells are permanent obstacles.

- **Objective:** maximin — maximize the minimum over spawns of the
  shortest-path distance from spawn center to nexus center.
- **Target instance:** `maps/endless.txt` (60x60, 4 spawns, obstacle lattice,
  ~2700 buildable cells).
- **Success criterion:** beat the maximin of
  `maps/endless_annealing_solution.txt` and `maps/endless_genetic_solution.txt`
  within 30–60 min wall clock, and report a valid upper bound and gap.
- **Solver:** Gurobi (full license), Python + gurobipy.

## Why the existing MILPs are slow

All current models (`integer_programming.py`, `integer_programming_file.py`,
`ip_simlpified.py`, `milp/integer_programming.cpp`) use a big-M distance-field
formulation: `d[u] <= d[v] + 1 + M(y[u] + y[v])` with M on the order of the
cell count. The LP relaxation lets tiny fractional `y` values inflate every
`d` to M, so the root bound is approximately M and useless; branch and bound
must fix thousands of binaries before the bound moves. The parent-selection
variables `p` in the original model add pure degeneracy (with a maximization
objective, the distance upper bounds alone already force `d` to the true
shortest-path distance). This is the observed "too degenerate" behavior.

## Chosen approach

Two cooperating layers (approaches B + C from brainstorming; monolithic
big-M model A rejected, kept in repo only as an untouched baseline):

1. **Exact engine — lazy path-cut master problem** (row generation via Gurobi
   callback). Replaces the distance field entirely.
2. **Matheuristic — large-neighborhood search (LNS)** that repeatedly
   re-optimizes windows of the map exactly with the same engine, plus a final
   full-map bound run.

## Architecture

New package `interdiction/`, existing scripts untouched.

| Module | Responsibility |
| --- | --- |
| `grid.py` | Map parsing (`S T # X W .` format from `integer_programming_file.py`), cell sets (walkable, buildable), adjacency, BFS distance fields, maximin evaluator `eval(walls) -> (maximin, per-spawn dists, shortest paths)`. No solver dependency. |
| `master.py` | Lazy-cut Gurobi model. Region interface: `solve(fixed_wall, fixed_open, free, time_limit, warm_start)`; the full map is the special case where everything is free. |
| `lns.py` | LNS driver: seeding, window selection, subsolves, acceptance, trajectory log. |
| `bound.py` | Full-map master run on remaining budget; reports incumbent, upper bound, gap. |
| `cli.py` | `python -m interdiction <map> [--seed sol.txt] [--time N] [--bound-frac 0.25] [--rng-seed N]`. Writes a solution map file in the existing convention. |

## Master model (`master.py`)

Variables:

- `y[v] ∈ {0,1}` — wall at buildable cell `v` (excludes `S`, `T`, `X`, `#`).
- `z_k ∈ Z` — claimed spawn-k shortest-path length (per-spawn variable so
  cuts bind tightly).
- `z` — objective variable, `z <= z_k` for all k, maximize `z`.
- `f[a] >= 0` continuous — flow on directed arcs between adjacent walkable
  cells.

Connectivity (in the model from the start, so infeasibility is impossible):

- Supply 1 at each spawn center, demand K at the nexus, conservation
  elsewhere.
- Node capacity `inflow(v) <= K * (1 - y[v])` for buildable `v`. With
  integral `y`, flow decomposition yields one spawn→nexus path per spawn, so
  every spawn stays connected. Continuous `f` suffices by max-flow/min-cut
  integrality; zero extra binaries.

Lazy cuts (callback on `MIPSOL`):

- BFS each spawn on the incumbent walls → true distance `d_k` and shortest
  path `P_k`.
- For each spawn k with `z_k > d_k`, add
  `z_k <= len(P_k) + (U - len(P_k)) * Σ_{v ∈ P_k ∩ buildable} y[v]`
  where `U` = walkable-cell count (safe upper bound on any simple path).
  If all of `P_k` is free the cut caps `z_k` at `len(P_k)`; if any cell is
  walled the RHS is at least `U` and the cut is inactive. Each cut is sparse
  (|P| nonzeros).
- Diversification: additionally cut 2–3 alternate shortest paths per spawn,
  sampled from the BFS shortest-path DAG, to reduce callback rounds.

Strengthening:

- Parity: the grid is bipartite, so every spawn-k path length is congruent to
  the Manhattan distance `|s_k - t|` mod 2. Encode `z_k = 2 q_k + parity_k`
  with `q_k` integer.
- Initial cuts: pre-seed with empty-grid shortest paths per spawn and the
  binding paths of the seed solution.
- Warm start: `y[v].Start` from seed walls.

## LNS driver (`lns.py`)

- **Seed:** `--seed <solution.txt>` — walls are cells that are `#` in the
  solution file but not `#` in the base map (existing solution-file
  convention). No seed → start with zero walls.
- **Loop** (until time budget):
  1. Evaluate incumbent: BFS all spawns → maximin, binding spawn(s), their
     shortest paths.
  2. Window: rectangle centered on a random cell of a binding path
     (improvement is impossible elsewhere); 20% of iterations use a fully
     random center for diversification. Window size is drawn round-robin
     from {8x8, 12x12, 16x16}; after 20 consecutive rejected iterations the
     draw is biased toward the largest size until the next accept.
  3. Subsolve: full-map master with `y` fixed to the incumbent outside the
     window (Gurobi presolve eliminates fixed variables; same code path as a
     full solve). Per-solve time limit 30 s by default (`--subsolve-time`),
     warm start = incumbent.
  4. Accept iff the global maximin strictly improves. Log the trajectory.
- Cuts reference whole paths; outside-window cells enter as constants. The
  maximin over all spawns is evaluated globally every iteration, so no
  local/global objective mismatch is possible.

## Bound phase (`bound.py`)

Sequential after LNS: full-map master with all cells free, using the
remaining budget (`--bound-frac`, default 25% of total), MIP start = final
incumbent. Report best bound and `gap = (bound - incumbent) / incumbent`.
Known caveat: the bound on `endless.txt` will be loose; it is nevertheless a
valid upper bound and tightens with more time.

## Output

Solution map file in the existing convention plus a stdout summary:
per-spawn distances, maximin, upper bound, gap, wall count, LNS iteration
log.

## Error handling

- Map validation on load: exactly one `T`, at least one `S`, every spawn
  BFS-reachable with zero walls; otherwise fail with a clear message before
  any solve.
- `TIME_LIMIT` with an incumbent is a normal outcome. `INFEASIBLE` is
  impossible by construction — assert and write `model.ilp` if it fires.
- Invariant after every solve and every LNS accept: claimed `z_k` equals the
  BFS-evaluated distance. An overclaim means a cut bug; fail loudly.
- On Ctrl-C, write the best-so-far solution file before exiting.

## Testing

- Unit tests for `grid.py`: every map symbol, BFS distances on hand-checked
  5x5 grids, maximin evaluator.
- Exactness: brute-force enumeration on tiny maps (<= 12 buildable cells,
  4096 wall subsets); the master must match the optimum exactly. Sanity check
  `basic.txt` against the existing script's result.
- Cut validity property test: on random small maps and random wall sets,
  every generated cut must be satisfied by the true optimal configuration.
- Benchmarks: `smaller_endless.txt` and `endless.txt` against the maximin of
  the annealing and genetic solution files; success = beat both within
  30–60 min and report bound/gap.
- `--rng-seed` for reproducible LNS runs.

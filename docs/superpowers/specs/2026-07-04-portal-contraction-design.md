# Portal-Contraction Window Subsolver + Corridor Hints — Design

Date: 2026-07-04
Status: approved
Builds on: `2026-07-04-interdiction-milp-design.md` (v1: lazy path-cut master + LNS)

## Problem

v1's LNS window subsolves run the full-map master with cells outside the
window fixed. Profiling on `maps/endless.txt` (60x60, 4 spawns) showed each
30 s subsolve spends ~21 s in the MIPSOL callback (26k calls, BFS over 3,600
cells each) and generates thousands of near-duplicate 1,400-cell path cuts,
while the LP bound stays vacuous (~3,202). Windows are blind enumeration; a
strong seed (annealing, maximin 1408) is never improved.

Goals:
1. Make window subsolves fast and small via exact portal contraction.
2. Encode the human "corridor maze" mechanic as optional pruning
   constraints.
3. Decide multi-cell wall piece handling.

## Decisions

- **Wall pieces (1x2, 1x4, 1x5, 2x2, L, ...):** keep the 1x1 cell model.
  It is a relaxation (upper bound) of the piece-constrained game; corridor
  walls are long straight runs that pieces tile almost perfectly. Piece
  tiling is a possible later post-processing step, not a solver concern.
- **Corridor hints:** both directions, applied to fully-free 2x2 squares in
  LNS windows, ON by default for LNS, never for `--exact`/bound runs:
  - no fully-open 2x2: `y_a + y_b + y_c + y_d >= 1`
  - no fully-walled 2x2: `y_a + y_b + y_c + y_d <= 3`
  These are heuristic (a true optimum may contain a plaza); window solves
  under hints may be INFEASIBLE, which is a normal "reject window" outcome.
- **Window solver:** Gurobi lazy cuts on the contracted graph (approach A).
  CP-SAT on the same contraction noted as a future swap if windows still
  stall — the contraction module is solver-agnostic.

## Architecture

Two new modules; v1 modules keep their roles.

| Module | Responsibility |
| --- | --- |
| `interdiction/contract.py` | Build an exact small weighted graph for a window given the fixed outside. No solver dependency. |
| `interdiction/window_master.py` | Gurobi lazy-cut solver over a `ContractedWindow`. |
| `interdiction/lns.py` (modified) | Window subsolves go through contract + window_master; global BFS re-evaluation stays as the acceptance check. |
| `interdiction/cli.py` (modified) | `--no-corridor-hint`, `--window-sizes`; new defaults. |
| `interdiction/master.py` (unchanged) | Full-map exact solves and the bound phase. |

## `contract.py`

`contract(grid, window_cells, outside_walls) -> ContractedWindow`

- **Inside nodes:** walkable cells of the window (X/S/T inside the window
  are plain nodes; buildable cells are flagged — they get `y` variables).
- **Portals:** outside walkable cells not in `outside_walls` adjacent to a
  window cell.
- **Outside distances:** BFS per portal and per outside spawn over the
  outside graph (walkable − window − outside_walls): `d(p,p')`, `d(p,T)`,
  `d(s_k,p)`, `d(s_k,T)`.
- **Weighted arcs:** window adjacency (w=1); window-boundary <-> portal
  (w=1); portal <-> portal, spawn <-> portal, portal <-> target,
  spawn <-> target bypass (w = outside BFS distance, only finite ones).
- **Exactness invariant:** for every assignment of walls to window cells,
  shortest spawn->target distance in the contracted graph equals the
  full-grid BFS distance under `outside_walls ∪ window_walls`. (Any real
  path alternates outside segments — collapsed into weighted edges — and
  window segments — represented explicitly.)
- Exposes `dijkstra(window_walls) -> (per_spawn_dist, per_spawn_path)`
  where a path is reported as the sequence of window cells it crosses
  (outside legs are inside the weighted edges; cuts only need window
  cells).
- Cost on endless: ~64 portal BFS x 3,600 cells ≈ 0.2 s per window,
  amortized over a 15-30 s solve.

Degenerate windows (no portals and no spawn/target inside — sealed by
fixed walls) cannot occur for windows centered on a binding path of a
feasible incumbent; assert with context if hit.

## `window_master.py`

`solve_window(cw, *, time_limit, warm_start=None, corridor_hint=True)
-> SolveResult` (same dataclass as `master.py`).

- `y[v]` binary per buildable window cell; `z_k` integer per spawn with the
  v1 parity encoding (contracted edge weights are true grid distances, so
  path-length parity is preserved); `z <= z_k`, maximize `z`.
- Flow connectivity on the contracted graph: supply 1 per spawn node,
  demand K at target, capacity `K(1 - y[v])` on buildable window nodes,
  portal/outside edges uncapacitated.
- MIPSOL callback: Dijkstra on the contracted graph (~300 nodes); for each
  spawn whose claim exceeds its true distance add
  `z_k <= len(P) + (U_w - len(P)) * sum(y[v] for v in P_window_cells)`.
  `U_w` = global U (`len(grid.walkable) - 1`) initially; tighten later only
  if profiling demands.
- Corridor hints as above; with hints ON, INFEASIBLE returns
  `SolveResult("NO_SOLUTION", ...)`. The "infeasible impossible" assertion
  only applies with hints OFF.
- Warm start from incumbent window walls; a start violating the hints is
  silently dropped by Gurobi (LNS acceptance still compares against the
  global incumbent, so nothing is lost).
- Per-window local model and cuts; `dispose()` on every exit; no shared
  pool. `MIPFocus=1`, `NodefileStart=0.5`, `SoftMemLimit=8`.

## LNS integration (`lns.py`)

Per iteration (`free = window ∩ grid.buildable`):
1. `cw = contract(grid, window, incumbent_walls - free)`
2. `res = solve_window(cw, time_limit=subsolve_time,
   warm_start=incumbent_walls & free, corridor_hint=hint_flag)`
3. Candidate = `(incumbent_walls - free) ∪ res_window_walls`;
   re-evaluate globally with BFS. Integration invariant: the BFS maximin
   must equal `res`'s claim (contraction exactness) — assert.
4. Accept iff strict global improvement (unchanged).

Defaults: window sizes `(12, 16, 20)` round-robin with the v1 stall
escalation; `--subsolve-time` default 15 s.

`master.py` remains the engine for `--exact` and the bound phase — those
stay exact and hint-free.

## CLI

- `--no-corridor-hint` — disable hints in LNS windows.
- `--window-sizes 12,16,20` — comma-separated override.
- All v1 flags unchanged.

## Error handling

- `solve_window` INFEASIBLE/`NO_SOLUTION` → reject window, `stall += 1`,
  never crash.
- Degenerate contraction (sealed window) → assert with window/center
  context.
- Integration invariant violation (BFS disagrees with contracted claim) →
  assert loudly: that is a contraction bug, the one class of error this
  design must never let pass silently.

## Testing

- **Contraction exactness property (load-bearing):** 100+ random cases —
  small maps (≤ 8x8), random windows, random wall configs; contracted
  Dijkstra distance == full-grid BFS distance for every spawn.
- **Window-solve equivalence:** hints OFF, small maps: `solve_window`
  objective == `master.solve(free=window, ...)` objective across several
  windows and seeds.
- **Corridor semantics:** a window whose only improvement is a plaza →
  hinted solve returns no improvement (and never crashes); a
  hint-infeasible window → `NO_SOLUTION`.
- **LNS integration:** smoke run improves over empty walls on
  `maps/basic.txt`; invariant assert exercised.
- **Manual benchmarks:** endless 16x16 window probe (expect callback time
  and MIPSOL count down ~10x vs v1's 21.5 s / 26k); smaller_endless from
  empty 15 min (expect ≥ 172); endless from annealing seed 60 min (the
  real test — improve past 1408 or confirm plateau with far more windows
  searched); endless from empty with hints 60 min (can corridors build a
  competitive maze from scratch?).

## Success criteria

Correctness suite green; window subsolve callback cost down an order of
magnitude; endless run either improves on 1408 or demonstrates the plateau
with an order of magnitude more windows explored per hour, reported
honestly in the README.

# emberward-pathfinder
A collection of algorithms to solve the network interdiction problem in a 2d square grid, inspired by tower defense games where the player has to build an optimal maze to defend from enemies.

### MILP
**Mixed-integer linear programming:** formulates the grid as a linear model and finds the optimal answer with a solver (Gurobi). Takes a very long time on +medium grids.

#### `interdiction/` — lazy path-cut master + LNS (current solver)

Replaces the big-M distance-field MILP with a row-generation master problem:
binary wall variables, per-spawn claimed-distance variables (with
bipartite-grid parity), a continuous multi-source flow that keeps every spawn
connected, and shortest-path cuts generated lazily in a Gurobi callback. A
large-neighborhood search re-optimizes windows of the map exactly with the
same master, and a final full-map run produces a valid upper bound and gap.

```
myenv/bin/python -m interdiction maps/endless.txt \
    --seed maps/endless_annealing_solution.txt --time 3600
myenv/bin/python -m interdiction maps/basic.txt --exact --time 120
myenv/bin/python -m interdiction maps/endless.txt --seed sol.txt --eval-only
```

LNS window subsolves use **portal contraction**: the fixed outside of each
window is collapsed into exact portal-to-portal shortest-distance edges, so
each window solves a ~300-node weighted graph instead of the full map (the
callback runs a small Dijkstra instead of a 3,600-cell BFS — roughly 4-5x
more search per window). Windows also apply **corridor hints** by default:
every fully-free 2x2 square gets `sum(y) >= 1` (no open plazas) and
`sum(y) <= 3` (no thick wall blocks), encoding how human players build
mazes. Hints are heuristic — disable with `--no-corridor-hint` (recommended
when polishing a seed that was not built corridor-style, e.g. the annealing
solution: its structure violates the hints and every hinted window would
just reject). `--window-sizes 12,16,20` overrides window sizes; `--exact`
and the bound phase never use hints and stay fully exact.

Objective is maximin over spawns (maximize the worst spawn's shortest path).
Results (Ryzen 5 5600X, Gurobi 12.0.3):

| Map | Mode | Maximin | Bound | Time |
| --- | --- | --- | --- | --- |
| `basic.txt` (7x7) | `--exact` | **28 (proven optimal)** | 28 | 2.8 s |
| `smaller_endless.txt` (37x37) | LNS, empty seed, hints | 228 (from 20; v1: 172) | 971 | 15 min |
| `endless.txt` (60x60) | LNS, annealing seed, no hints | 1408 (= seed) | 3202 | 60 min |
| `endless.txt` (60x60) | LNS, empty seed, hints | 510 (from 20; v1: no progress) | 3202 | 60 min |

Baselines on `endless.txt`: annealing 1408, genetic 213. The annealing
solution is empirically locally optimal with respect to exact rewrites of
every 12-20-cell window tried in an hour — the contraction searched ~4x
more configurations per window than v1 and still found no improving move.
Progress past 1408 would need structurally different moves (much larger
windows, corridor re-routing across windows, or a better global bound).
The bound on large maps is valid but loose: fractional walls defeat the
path-cut LP relaxation (root bound ~3202 regardless of cuts). Where the
solver wins: proven optima on small/mid maps in seconds, strong
from-scratch mazes (20 -> 228 on smaller_endless in 15 min, 20 -> 510 on
endless in 1 h, both still climbing at cutoff), and honest gap reporting.

### Genetic
**Genetic algorithm:** more of an experiment than not, but it may be a good "greedy" approach as it has some resemblance to what a human player does playing the game.

### Annealing
**Simulated annealing:** finds decent solutions quickly depending on the parameters, seems too random to find the *good* ones.

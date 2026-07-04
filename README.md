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

Objective is maximin over spawns (maximize the worst spawn's shortest path).
Results (Ryzen 5 5600X, Gurobi 12.0.3):

| Map | Mode | Maximin | Bound | Time |
| --- | --- | --- | --- | --- |
| `basic.txt` (7x7) | `--exact` | **28 (proven optimal)** | 28 | 2.8 s |
| `smaller_endless.txt` (37x37) | LNS, empty seed | 172 (from 20) | 971 | 15 min |
| `endless.txt` (60x60) | LNS, annealing seed | 1408 (= seed) | 3202 | 60 min |

Baselines on `endless.txt`: annealing 1408, genetic 213. The LNS matches but
does not improve the annealing solution — its 8–16-cell windows cannot
restructure an already-tuned serpentine maze, and the path-cut LP relaxation
is too weak to prune the search (root bound ~3202 regardless of cuts), so
window solves are effectively enumeration. The bound on large maps is valid
but loose for the same reason. Where this solver clearly wins: proven optima
on small/mid maps in seconds, strong from-scratch solutions (20 -> 172 on
smaller_endless in 15 min), and honest gap reporting.

### Genetic
**Genetic algorithm:** more of an experiment than not, but it may be a good "greedy" approach as it has some resemblance to what a human player does playing the game.

### Annealing
**Simulated annealing:** finds decent solutions quickly depending on the parameters, seems too random to find the *good* ones.

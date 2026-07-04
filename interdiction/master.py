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
    GRB.MEM_LIMIT: "MEM_LIMIT",
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
        # keep repeated solves from exhausting RAM: spill the B&B tree to
        # disk past 0.5 GB and stop with MEM_LIMIT instead of getting OOM-killed
        m.Params.NodefileStart = 0.5
        m.Params.SoftMemLimit = 8
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

        # the callback closure keeps the model alive through reference
        # cycles — dispose explicitly or repeated solves leak the C-side
        # model memory until the machine OOMs
        if m.Status == GRB.INFEASIBLE:
            m.computeIIS()
            m.write("master_infeasible.ilp")
            m.dispose()
            raise AssertionError(
                "master infeasible — impossible by construction, see .ilp")

        status = _STATUS.get(m.Status, str(m.Status))
        bound = m.ObjBound
        if m.SolCount == 0:
            m.dispose()
            return SolveResult("NO_SOLUTION", None, None, None, bound)

        walls = {v for v, var in y.items() if var.X > 0.5}
        obj_val = m.ObjVal
        m.dispose()
        maximin, per = g.evaluate(walls)
        # callback guarantees incumbents never overclaim
        assert maximin is not None and round(obj_val) <= maximin, \
            "incumbent overclaims shortest path — cut bug"
        return SolveResult(status, walls, maximin, per, bound)

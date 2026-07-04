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
        if all(d is not None for d, _cells in ws_res):
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

#!/usr/bin/env python3
"""
Grid shortest-path interdiction with parent-selection (patched).
Now enforces shortest-path semantics by:
- upper-bound distance against *all* neighbors
- at least one parent neighbor achieves equality
"""

from gurobipy import Model, GRB, quicksum
from collections import deque

def make_grid_nodes(rows, cols):
    return [(r, c) for r in range(rows) for c in range(cols)]

def neighbours(node, rows, cols):
    r, c = node
    for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols:
            yield (nr, nc)

def build_and_solve(rows=8, cols=8, spawn=(0,0), nucleus=None, time_limit=60):
    if nucleus is None:
        nucleus = (rows-1, cols-1)

    V = make_grid_nodes(rows, cols)
    N_nodes = len(V)
    N_max = N_nodes - 1
    M = N_max

    m = Model("grid_parent_patch")
    m.setParam("OutputFlag", 1)
    m.setParam("TimeLimit", time_limit)

    # Variables
    y = {v: m.addVar(vtype=GRB.BINARY, name=f"y_{v}") for v in V}
    d = {v: m.addVar(vtype=GRB.INTEGER, lb=0, ub=N_max, name=f"d_{v}") for v in V}
    p = {}

    for u in V:
        if u == nucleus:
            continue
        for v in neighbours(u, rows, cols):
            p[(u,v)] = m.addVar(vtype=GRB.BINARY, name=f"p_{u}_{v}")

    # Constraints
    m.addConstr(y[spawn] == 0)
    m.addConstr(y[nucleus] == 0)
    m.addConstr(d[nucleus] == 0)

    for v in V:
        m.addConstr(d[v] <= N_max * (1 - y[v]))

    # Core patch: upper bounds relative to *all* neighbors
    for u in V:
        for v in neighbours(u, rows, cols):
            m.addConstr(d[u] <= d[v] + 1 + M * (y[u] + y[v]), name=f"ub_{u}_{v}")

    # Parent constraints: each free non-nucleus picks one parent
    for u in V:
        if u == nucleus:
            continue
        neighs = list(neighbours(u, rows, cols))
        m.addConstr(quicksum(p[(u,v)] for v in neighs) == 1 - y[u])

        for v in neighs:
            m.addConstr(p[(u,v)] <= 1 - y[v])
            # enforce equality when chosen
            m.addConstr(d[u] - d[v] - 1 <= M * (1 - p[(u,v)]))
            m.addConstr(d[u] - d[v] - 1 >= -M * (1 - p[(u,v)]))

    # Objective
    m.setObjective(d[spawn], GRB.MAXIMIZE)

    m.optimize()

    if m.Status not in (GRB.OPTIMAL, GRB.TIME_LIMIT, GRB.SUBOPTIMAL):
        raise RuntimeError(f"Solver ended with status {m.Status}")

    y_val = {v: int(round(y[v].X)) for v in V}
    d_val = {v: int(round(d[v].X)) for v in V}

    print(f"Objective (d[spawn]) = {d_val[spawn]}")

    print("\nGrid layout (S=spawn, T=nucleus, 1=wall, 0=free):")
    for r in range(rows):
        line = ""
        for c in range(cols):
            v = (r,c)
            if v == spawn: line += "S "
            elif v == nucleus: line += "T "
            else: line += f"{y_val[v]} "
        print(line)

    print("\nDistance map:")
    for r in range(rows):
        line = ""
        for c in range(cols):
            line += f"{d_val[(r,c)]:2d} "
        print(line)

    return d_val[spawn]

if __name__ == "__main__":
    build_and_solve(rows=9, cols=9, spawn=(0,0), nucleus=(8,8), time_limit=120)

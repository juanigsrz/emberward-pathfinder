#!/usr/bin/env python3
"""
Grid shortest-path interdiction, simplified formulation.

This version removes the explicit 'p' (parent-selector) variables.
The maximization objective, combined with the distance upper-bound
constraints, is sufficient to enforce shortest-path semantics, leading
to a model with far fewer variables and constraints.
"""

from gurobipy import Model, GRB, quicksum

def make_grid_nodes(rows, cols):
    """Generates a list of all nodes (cells) in the grid."""
    return [(r, c) for r in range(rows) for c in range(cols)]

def neighbours(node, rows, cols):
    """Yields all valid orthogonal neighbors of a given node."""
    r, c = node
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols:
            yield (nr, nc)

def build_and_solve_simplified(rows=8, cols=8, spawn=(0, 0), nucleus=None, time_limit=60):
    """Builds and solves the simplified interdiction model."""
    if nucleus is None:
        nucleus = (rows - 1, cols - 1)

    V = make_grid_nodes(rows, cols)
    M = rows * cols  # A safe "Big-M" value

    m = Model("grid_simplified")
    m.setParam("OutputFlag", 1)
    m.setParam("TimeLimit", time_limit)

    # --- Variables ---
    # y[v] = 1 if a wall is built at node v, 0 otherwise.
    y = m.addVars(V, vtype=GRB.BINARY, name="y")
    # d[v] = The shortest path distance from node v to the nucleus.
    d = m.addVars(V, vtype=GRB.INTEGER, lb = -1 * M, ub = M, name="d")

    # --- Constraints ---
    # 1. No walls on spawn or nucleus points.
    m.addConstr(y[spawn] == 0, name="no_wall_at_spawn")
    m.addConstr(y[nucleus] == 0, name="no_wall_at_nucleus")

    # 2. The distance from the nucleus to itself is 0.
    m.addConstr(d[nucleus] == 0, name="dist_at_nucleus")

    # 3. Core Logic: The distance at a node 'u' is at most 1 greater than
    #    its neighbor 'v'. This constraint is relaxed using Big-M if either
    #    'u' or 'v' is a wall, effectively removing the path.
    for u in V:
        if u != nucleus:
            m.addConstr((y[u] == 1) >> (d[u] <= -1 * M), name=f"dist_wall_{u}")

            pepe = m.addVar(vtype=GRB.BINARY, name=f"pepe_{u}_{v}")
            m.addGenConstrMax(pepe, [y[x] for x in neighbours(u, rows, cols)])
            m.addConstr((pepe == 1) >> (d[u] <= -1 * M))

            for v in neighbours(u, rows, cols):
                xD = m.addVar(vtype=GRB.BINARY, name=f"xD_{u}_{v}")
                m.addGenConstrMax(xD, [y[u], y[v]])
                m.addConstr((xD == 0) >> (d[u] <= d[v] + 1))


    # --- Objective ---
    # Maximize the shortest-path distance from the spawn point.
    m.setObjective(d[spawn], GRB.MAXIMIZE)
    #m.setObjective(quicksum(d[x] for x in V), GRB.MAXIMIZE)

    m.optimize()

    # --- Print Results ---
    if m.Status in (GRB.OPTIMAL, GRB.TIME_LIMIT, GRB.SUBOPTIMAL):
        y_val = {v: int(round(y[v].X)) for v in V}
        d_val = {v: int(round(d[v].X)) for v in V}

        print(f"\nObjective (d[spawn]) = {d_val[spawn]}")

        print("\nGrid layout (S=spawn, T=nucleus, 1=wall):")
        for r in range(rows):
            line = ""
            for c in range(cols):
                v = (r, c)
                if v == spawn: line += "S "
                elif v == nucleus: line += "T "
                else: line += f"{y_val[v]} "
            print(line.strip())

        print("\nDistance map:")
        for r in range(rows):
            line = ""
            for c in range(cols):
                v = (r, c)
                line += f"{d_val.get(v, 0):2d} "
            print(line.strip())
    else:
        print(f"Solver ended with status code: {m.Status}")

    return m.ObjVal

if __name__ == "__main__":
    build_and_solve_simplified(rows=9, cols=9, spawn=(0, 0), nucleus=(8, 8), time_limit=120)
# Requires: ortools (pip install ortools)
from ortools.sat.python import cp_model

def build_and_solve_grid_maxmin(rows, cols, s_cell, t_cell, time_limit_seconds=30):
    # Helper to index nodes and directed edges
    def node_id(r, c): return r * cols + c
    V = [node_id(r, c) for r in range(rows) for c in range(cols)]
    nV = len(V)
    # Undirected edges: store as (u,v)
    undirected_edges = []
    for r in range(rows):
        for c in range(cols):
            u = node_id(r, c)
            for dr, dc in [(1,0),(0,1)]:  # only down and right to avoid duplicates
                nr, nc = r+dr, c+dc
                if 0 <= nr < rows and 0 <= nc < cols:
                    v = node_id(nr, nc)
                    undirected_edges.append((u, v))
    # Directed edges for flow variables (two directions per undirected edge)
    directed_edges = []
    for (u, v) in undirected_edges:
        directed_edges.append((u, v))
        directed_edges.append((v, u))

    model = cp_model.CpModel()
    # Variables
    z = {}   # z_e = 1 if wall placed on undirected edge e=(u,v)
    for idx, (u, v) in enumerate(undirected_edges):
        z[(u,v)] = model.NewBoolVar(f"z_{u}_{v}")

    # Flow variables f_{uv} for directed edges (0/1)
    f = {}
    for (u, v) in directed_edges:
        f[(u,v)] = model.NewBoolVar(f"f_{u}_{v}")

    # Potentials pi_v : integer 0..M
    M = nV - 1  # safe upper bound on path length
    pi = {}
    for v in V:
        pi[v] = model.NewIntVar(0, M, f"pi_{v}")

    # Flow conservation: supply at s = 1, demand at t = -1, others 0
    s = node_id(*s_cell)
    t = node_id(*t_cell)
    for v in V:
        out_vars = []
        in_vars = []
        for (u2, v2) in directed_edges:
            if u2 == v:
                out_vars.append(f[(u2, v2)])
            if v2 == v:
                in_vars.append(f[(u2, v2)])
        if v == s:
            model.Add(sum(out_vars) - sum(in_vars) == 1)
        elif v == t:
            model.Add(sum(out_vars) - sum(in_vars) == -1)
        else:
            model.Add(sum(out_vars) - sum(in_vars) == 0)

    # Link flow to open edges: if undirected edge (u,v) is blocked (z=1), 
    # neither direction may carry flow.
    # If z==0 (open), each directed flow var can be 0/1.
    for (u, v) in undirected_edges:
        zuv = z[(u, v)]
        # f(u,v) <= 1 - z and f(v,u) <= 1 - z  -> f <= not z
        model.Add(f[(u, v)] <= 1 - zuv)
        model.Add(f[(v, u)] <= 1 - zuv)

    # Dual constraints: for every directed arc (u->v),
    # pi_u - pi_v <= 1 + M * z_{uv_undirected}
    # (i.e., if edge open (z=0) then pi_u - pi_v <= 1, else relaxed)
    for (u, v) in directed_edges:
        # find corresponding undirected edge
        if (u, v) in z:
            zuv = z[(u, v)]
        elif (v, u) in z:
            zuv = z[(v, u)]
        else:
            raise RuntimeError("Edge mapping issue")
        model.Add(pi[u] - pi[v] <= 1 + M * zuv)

    # Objective: maximize pi_s - pi_t
    obj = model.NewIntVar(-M, M, "obj")
    model.Add(obj == pi[s] - pi[t])
    model.Maximize(obj)

    # Optional symmetry breaking / tightening:
    # - bound pi[s] = 0 would force pi measured from s; but we can leave free.
    # It's safe to set pi[s] = 0 to reduce symmetry (potentials are relative).
    model.Add(pi[s] == 0)

    # Solve
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = time_limit_seconds
    solver.parameters.num_search_workers = 12

    result = solver.Solve(model)

    if result in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        z_edges = [(u, v) for (u, v) in undirected_edges if solver.Value(z[(u, v)]) == 1]
        shortest_length = solver.Value(obj)
        # Reconstruct a feasible s-t path (the flow f) — there will be at least one unit flow
        flow_arcs = [(u, v) for (u, v) in directed_edges if solver.Value(f[(u, v)]) == 1]
        return {
            "status": solver.StatusName(result),
            "objective": shortest_length,
            "blocked_edges": z_edges,
            "flow_arcs": flow_arcs,
            "pi": {v: solver.Value(pi[v]) for v in V}
        }
    else:
        return {"status": solver.StatusName(result)}

# Example run: 5x5 grid, source top-left (0,0), sink bottom-right (4,4)
if __name__ == "__main__":
    sol = build_and_solve_grid_maxmin(7, 7, (0,0), (6,6), time_limit_seconds=15)
    print(sol)

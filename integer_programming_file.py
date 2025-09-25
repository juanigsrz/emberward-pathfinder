#!/usr/bin/env python3
"""
Grid shortest-path interdiction with parent-selection (patched).
Now enforces shortest-path semantics by:
- upper-bound distance against *all* neighbors
- at least one parent neighbor achieves equality
"""

from gurobipy import Model, GRB, quicksum
from collections import deque

def read_map_file(filename):
    """
    Reads a grid map from a text file and returns:
      - lines (list of strings)
      - spawns (list of (r,c))
      - target (r,c)
      - obstacles (set of (r,c)) - unremovable obstacles '#'
      - unbuildables (set of (r,c)) - spaces 'X' enemies can move through but player cannot build
      - preset_walls (set of (r,c)) - pre-set walls 'W' that can be changed (warm startup)
      - rows, cols (int)
    """
    with open(filename, "r") as f:
        lines = [line.rstrip("\n") for line in f if line.strip()]

    if not lines:
        raise ValueError("Empty map file")

    rows = len(lines)
    cols = len(lines[0])
    spawns = []
    target = None
    obstacles = set()      # '#' - unremovable obstacles
    unbuildables = set()   # 'X' - enemies can move, player cannot build
    preset_walls = set()   # 'W' - pre-set walls that can be changed

    for r, row in enumerate(lines):
        for c, ch in enumerate(row):
            if ch == "S":
                spawns.append((r, c))
            elif ch == "T":
                target = (r, c)
            elif ch == "#":
                obstacles.add((r, c))
            elif ch == "X":
                unbuildables.add((r, c))
            elif ch == "W":
                preset_walls.add((r, c))

    if not spawns:
        raise ValueError("No spawn (S) found in map file")
    if target is None:
        raise ValueError("No target (T) found in map file")

    return lines, spawns, target, obstacles, unbuildables, preset_walls, rows, cols

def make_grid_nodes(rows, cols):
    return [(r, c) for r in range(rows) for c in range(cols)]

def neighbours(node, rows, cols):
    r, c = node
    for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols:
            yield (nr, nc)

def build_and_solve_from_file(filename, time_limit=120):
    """
    Build and solve the integer programming model from a map file.
    """
    lines, spawns, target, obstacles, unbuildables, preset_walls, rows, cols = read_map_file(filename)
    
    print("Map:")
    for line in lines:
        print(line)
    print(f"Spawns: {spawns}")
    print(f"Target: {target}")
    print(f"Obstacles (#): {len(obstacles)}")
    print(f"Unbuildables (X): {len(unbuildables)}")
    print(f"Preset walls (W): {len(preset_walls)}")
    
    return build_and_solve(rows, cols, spawns[0], target, obstacles, unbuildables, preset_walls, time_limit)

def build_and_solve(rows, cols, spawn, nucleus, obstacles=set(), unbuildables=set(), preset_walls=set(), time_limit=600):
    """
    Build and solve the integer programming model.
    
    Parameters:
    - obstacles: '#' positions that block movement and cannot be built on
    - unbuildables: 'X' positions that enemies can move through but player cannot build on
    - preset_walls: 'W' positions that start as walls but can be changed (warm startup)
    """
    # Only consider positions that are not permanent obstacles
    # This includes spawn, target, unbuildables, preset_walls, and free spaces
    V = []
    for r in range(rows):
        for c in range(cols):
            pos = (r, c)
            # Include positions that are not permanent obstacles (#)
            if pos not in obstacles:
                V.append(pos)
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
            # Only add parent variables for neighbors that are valid positions (in V)
            if v in V:
                p[(u,v)] = m.addVar(vtype=GRB.BINARY, name=f"p_{u}_{v}")

    # Constraints
    m.addConstr(y[spawn] == 0)  # Spawn must be free
    m.addConstr(y[nucleus] == 0)  # Target must be free
    m.addConstr(d[nucleus] == 0)  # Target has distance 0
    
    # Unbuildables must remain free (enemies can move through)
    for pos in unbuildables:
        if pos in V:  # Only if position is in our variable set
            m.addConstr(y[pos] == 0)
    
    # Preset walls: warm startup (initialize as walls but can be changed)
    for pos in preset_walls:
        if pos in V:  # Only if position is in our variable set
            y[pos].Start = 1  # Gurobi warm start hint
    
    # Distance constraint: if a cell is blocked (y=1), its distance should be 0
    for v in V:
        m.addConstr(d[v] <= N_max * (1 - y[v]))

    # Core patch: upper bounds relative to *all* neighbors that allow movement
    for u in V:
        for v in neighbours(u, rows, cols):
            # Only create constraints for neighbors that are not permanent obstacles
            if v not in obstacles:  # Obstacles (#) block all movement
                if v in V:  # Neighbor is in our variable set
                    m.addConstr(d[u] <= d[v] + 1 + M * (y[u] + y[v]), name=f"ub_{u}_{v}")
                else:  # This shouldn't happen given our V construction, but just in case
                    continue

    # Parent constraints: each free non-nucleus picks one parent
    for u in V:
        if u == nucleus:
            continue
        # Only consider neighbors that are not permanent obstacles and are in V
        neighs = [v for v in neighbours(u, rows, cols) if v in V and v not in obstacles]
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

    print("\nSolution grid:")
    for r in range(rows):
        line = ""
        for c in range(cols):
            v = (r, c)
            if v == spawn:
                line += "S"  # Spawn
            elif v == nucleus:
                line += "T"  # Target
            elif v in obstacles:
                line += "#"  # Unremovable obstacle
            elif v in unbuildables:
                line += "X"  # Unbuildable (enemies can move, player cannot build)
            elif v in y_val and y_val[v] == 1:
                line += "W"  # Wall (either preset or placed by solver)
            else:
                line += "."  # Free space
        print(line)

    print("\nDistance map:")
    for r in range(rows):
        line = ""
        for c in range(cols):
            v = (r, c)
            if v in d_val:
                line += f"{d_val[v]:2d} "
            else:
                line += " 0 "  # For obstacles/blocked positions
        print(line)

    return d_val[spawn]

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Usage: python integer_programming.py map.txt")
        sys.exit(1)
    
    try:
        filename = sys.argv[1]
        build_and_solve_from_file(filename, time_limit=600)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

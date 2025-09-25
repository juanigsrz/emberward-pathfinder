import random, math, time
from collections import deque

# ---------------------------
# Map reader
# ---------------------------
def read_map_file(filename):
    with open(filename, "r") as f:
        lines = [line.rstrip("\n") for line in f if line.strip()]

    spawns = []
    target = None
    obstacles = set()
    unbuildables = set()

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

    if not spawns:
        raise ValueError("No spawn (S) found in map file")
    if target is None:
        raise ValueError("No target (T) found in map file")

    return lines, spawns, target, obstacles, unbuildables

# ---------------------------
# Shortest path with BFS
# ---------------------------
def shortest_path(grid, start, goal):
    R, C = len(grid), len(grid[0])
    q = deque([(start, 0)])
    seen = {start}
    while q:
        (r, c), dist = q.popleft()
        if (r, c) == goal:
            return dist
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nr, nc = r+dr, c+dc
            if 0 <= nr < R and 0 <= nc < C and (nr, nc) not in seen:
                if grid[nr][nc] != "#":  # walls block
                    seen.add((nr, nc))
                    q.append(((nr, nc), dist+1))
    return None  # no path

def evaluate(grid, spawns, target):
    dists = []
    for s in spawns:
        d = shortest_path(grid, s, target)
        if d is None:
            return -1e6  # infeasible
        dists.append(d)
    return min(dists)  # worst-case enemy

# ---------------------------
# Simulated Annealing
# ---------------------------
def simulated_annealing(lines, spawns, target, obstacles, unbuildables,
                        max_iter=50000, T0=10.0, alpha=0.9995):
    R, C = len(lines), len(lines[0])

    # Mutable grid
    grid = [list(row) for row in lines]

    # Buildable candidates
    candidates = [(r,c) for r in range(R) for c in range(C)
                  if (lines[r][c] == "." and (r,c) not in obstacles and (r,c) not in unbuildables)]

    # Initial state: no new walls
    best_grid = [row[:] for row in grid]
    best_score = evaluate(grid, spawns, target)

    current_grid = [row[:] for row in grid]
    current_score = best_score

    T = T0
    start_time = time.time()

    for it in range(max_iter):
        # Pick random move
        r, c = random.choice(candidates)
        move_type = random.choice(["add","remove"])

        # Apply
        if move_type == "add" and current_grid[r][c] == ".":
            current_grid[r][c] = "#"
        elif move_type == "remove" and current_grid[r][c] == "#":
            current_grid[r][c] = "."
        else:
            continue  # skip invalid

        score = evaluate(current_grid, spawns, target)

        delta = score - current_score
        if delta >= 0 or random.random() < math.exp(delta / T):
            current_score = score
            if score > best_score:
                best_score = score
                best_grid = [row[:] for row in current_grid]
        else:
            # revert
            current_grid[r][c] = "." if move_type == "add" else "#"

        T *= alpha
        if it % 5000 == 0:
            print(f"Iter {it}, Temp={T:.3f}, Best={best_score}")

    elapsed = time.time() - start_time
    print(f"SA finished in {elapsed:.2f}s — best distance = {best_score}")

    return best_grid, best_score

# ---------------------------
# Main
# ---------------------------
if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Usage: python sa_solver.py map.txt")
        sys.exit(1)

    mapfile = sys.argv[1]
    lines, spawns, target, obstacles, unbuildables = read_map_file(mapfile)

    best_grid, score = simulated_annealing(lines, spawns, target, obstacles, unbuildables)

    print("\nBest solution:")
    for row in best_grid:
        print("".join(row))

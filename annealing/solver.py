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
# Distance computation (reverse BFS)
# ---------------------------
def compute_distances(grid, target):
    R, C = len(grid), len(grid[0])
    dist = [[None]*C for _ in range(R)]
    q = deque([target])
    dist[target[0]][target[1]] = 0

    while q:
        r, c = q.popleft()
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nr, nc = r+dr, c+dc
            if 0 <= nr < R and 0 <= nc < C:
                # walkable cells are: '.', 'S', 'T', 'X' (empty, spawns, target, unbuildable)
                if dist[nr][nc] is None and grid[nr][nc] in ('.','S','T','X'):
                    dist[nr][nc] = dist[r][c] + 1
                    q.append((nr, nc))
    return dist

def evaluate(grid, spawns, target):
    dist = compute_distances(grid, target)
    values = []
    for sr, sc in spawns:
        d = dist[sr][sc]
        if d is None:
            return -1e6
        values.append(d)
    return min(values)

# ---------------------------
# Piece definitions
# ---------------------------
def rotate_offsets(offsets):
    return [(-y, x) for (x,y) in offsets]

def all_rotations(base_offsets):
    rotations = []
    seen = set()
    offsets = base_offsets
    for _ in range(4):
        key = tuple(sorted(offsets))
        if key not in seen:
            seen.add(key)
            rotations.append(offsets)
        offsets = rotate_offsets(offsets)
    return rotations

# Each entry: (symbol, list of orientations)
PIECES = []
# original pieces: 1x5, 1x4, L(3)
# PIECES.append(("1", [[(0,i) for i in range(5)], [(i,0) for i in range(5)]]))
# PIECES.append(("2", [[(0,i) for i in range(4)], [(i,0) for i in range(4)]]))
# PIECES.append(("3", all_rotations([(0,0),(1,0),(1,1)])))

# Add tetrominoes (Tetris pieces) with symbols I,O,T,S,Z,J,L
# Coordinates are given in a small bounding box; rotations are generated
PIECES.append(("I", all_rotations([(0,0),(0,1),(0,2),(0,3)])))  # line 4
PIECES.append(("O", all_rotations([(0,0),(0,1),(1,0),(1,1)])))  # square
PIECES.append(("M", all_rotations([(0,0),(0,1),(0,2),(1,1)])))  # T (T symbol was used, so we gonna go with M)
PIECES.append(("N", all_rotations([(0,1),(0,2),(1,0),(1,1)])))  # S (S symbol was used, so we gonna go with N)
PIECES.append(("Z", all_rotations([(0,0),(0,1),(1,1),(1,2)])))  # Z
PIECES.append(("J", all_rotations([(0,0),(1,0),(1,1),(1,2)])))  # J
PIECES.append(("L", all_rotations([(0,2),(1,0),(1,1),(1,2)])))  # L

# ---------------------------
# Piece operations
# ---------------------------
def can_place(grid, r, c, shape, obstacles, unbuildables):
    R, C = len(grid), len(grid[0])
    coords = []
    for dr,dc in shape:
        rr, cc = r+dr, c+dc
        if not (0 <= rr < R and 0 <= cc < C):
            return None
        # must be empty '.' and not overlap obstacle/unbuildable/spawn/target
        if grid[rr][cc] != "." or (rr,cc) in obstacles or (rr,cc) in unbuildables:
            return None
        coords.append((rr,cc))
    return coords

def place_piece(grid, coords, symbol):
    for (r,c) in coords:
        grid[r][c] = symbol

def remove_piece(grid, coords):
    for (r,c) in coords:
        grid[r][c] = "."

# ---------------------------
# Simulated Annealing with pieces
# ---------------------------
def simulated_annealing(lines, spawns, target, obstacles, unbuildables,
                        max_iter=200000, T0=50.0, alpha=0.9995):
    R, C = len(lines), len(lines[0])

    # Mutable grid
    grid = [list(row) for row in lines]

    best_grid = [row[:] for row in grid]
    best_score = evaluate(grid, spawns, target)

    current_grid = [row[:] for row in grid]
    current_score = best_score

    placed_pieces = []  # list of (coords, symbol)

    T = T0
    start_time = time.time()

    for it in range(max_iter):
        # bias towards adding when few pieces present
        move_type = "add" if not placed_pieces or random.random() < 0.6 else "remove"

        if move_type == "add":
            symbol, orientations = random.choice(PIECES)
            shape = random.choice(orientations)
            r, c = random.randrange(R), random.randrange(C)
            coords = can_place(current_grid, r, c, shape, obstacles, unbuildables)
            if coords is None:
                continue

            # Tentative add
            place_piece(current_grid, coords, symbol)
            score = evaluate(current_grid, spawns, target)

            delta = score - current_score
            if delta >= 0 or random.random() < math.exp(delta / T):
                current_score = score
                placed_pieces.append((coords, symbol))
                if score > best_score:
                    best_score, best_grid = score, [row[:] for row in current_grid]
            else:
                remove_piece(current_grid, coords)

        else:  # remove
            coords, symbol = random.choice(placed_pieces)

            # Tentative remove
            remove_piece(current_grid, coords)
            score = evaluate(current_grid, spawns, target)

            delta = score - current_score
            if delta >= 0 or random.random() < math.exp(delta / T):
                current_score = score
                placed_pieces.remove((coords, symbol))
                if score > best_score:
                    best_score, best_grid = score, [row[:] for row in current_grid]
            else:
                place_piece(current_grid, coords, symbol)

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

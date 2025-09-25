import sys
import matplotlib.pyplot as plt
import numpy as np
from collections import deque

def read_map_file(filename):
    with open(filename, "r") as f:
        lines = [line.rstrip("\n") for line in f if line.strip()]
    return lines

def parse_map(lines):
    spawns = []
    target = None
    obstacles = set()
    unbuildables = set()
    build_walls = set()
    empty = set()

    for r, row in enumerate(lines):
        for c, ch in enumerate(row):
            if ch == "S":
                spawns.append((r, c))
                empty.add((r, c))
            elif ch == "T":
                target = (r, c)
                empty.add((r, c))
            elif ch == "#":
                obstacles.add((r, c))
            elif ch == "X":
                unbuildables.add((r, c))
                empty.add((r, c))
            elif ch == ".":
                empty.add((r, c))
            else:
                # Unexpected char — treat as empty
                empty.add((r, c))
    return spawns, target, obstacles, unbuildables, empty, len(lines), len(lines[0])

def shortest_path(grid, start, goal):
    """ BFS shortest path on walkable grid. Returns list of (r,c) """
    R, C = len(grid), len(grid[0])
    q = deque([(start, [])])
    seen = {start}
    while q:
        (r, c), path = q.popleft()
        if (r, c) == goal:
            return path + [(r, c)]
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nr, nc = r+dr, c+dc
            if 0 <= nr < R and 0 <= nc < C and (nr, nc) not in seen:
                if grid[nr][nc] != "#":  # walls/obstacles block
                    q.append(((nr, nc), path+[(r, c)]))
                    seen.add((nr, nc))
    return None

def visualize_map(lines):
    spawns, target, obstacles, unbuildables, empty, R, C = parse_map(lines)

    # Build color grid
    color_map = {
        "empty": (0.9, 0.9, 0.9),      # light gray
        "wall": (0.2, 0.2, 0.2),       # black
        "spawn": (0.1, 0.6, 0.1),      # green
        "target": (0.8, 0.1, 0.1),     # red
        "unbuild": (0.3, 0.6, 0.9),    # blue
    }

    grid_colors = np.zeros((R, C, 3))
    for r in range(R):
        for c in range(C):
            ch = lines[r][c]
            if ch == "S":
                grid_colors[r, c] = color_map["spawn"]
            elif ch == "T":
                grid_colors[r, c] = color_map["target"]
            elif ch == "#":
                grid_colors[r, c] = color_map["wall"]
            elif ch == "X":
                grid_colors[r, c] = color_map["unbuild"]
            else:
                grid_colors[r, c] = color_map["empty"]

    fig, ax = plt.subplots(figsize=(C/2, R/2))
    ax.imshow(grid_colors, interpolation="none")
    ax.set_xticks(np.arange(-0.5, C, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, R, 1), minor=True)
    ax.grid(which="minor", color="k", linestyle="-", linewidth=0.5)
    ax.set_xticks([])
    ax.set_yticks([])

    # Draw paths from each spawn
    for spawn in spawns:
        path = shortest_path(lines, spawn, target)
        if path:
            y, x = zip(*path)
            ax.plot(x, y, color="yellow", linewidth=2, alpha=0.8)

    plt.show()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python visualize_map.py map.txt")
        sys.exit(1)

    lines = read_map_file(sys.argv[1])
    visualize_map(lines)

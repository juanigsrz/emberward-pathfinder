# map_reader.py
def read_map_file(filename):
    """
    Reads a grid map from a text file and returns:
      - grid (list of strings)
      - spawns (list of (r,c))
      - target (r,c)
      - obstacles (set of (r,c))
      - unbuildables (set of (r,c))
    """
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
            # '.' means normal buildable, so nothing special

    if not spawns:
        raise ValueError("No spawn (S) found in map file")
    if target is None:
        raise ValueError("No target (T) found in map file")

    return lines, spawns, target, obstacles, unbuildables


if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Usage: python map_reader.py map.txt")
        sys.exit(1)

    filename = sys.argv[1]
    grid, spawns, target, obstacles, unbuildables = read_map_file(filename)

    print("Map:")
    for row in grid:
        print(row)
    print("Spawns:", spawns)
    print("Target:", target)
    print("Obstacles:", obstacles)
    print("Unbuildables:", unbuildables)

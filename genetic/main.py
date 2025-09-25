import genetic_algorithm
import map_reader

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Usage: python main.py map.txt")
        sys.exit(1)

    filename = sys.argv[1]
    grid, spawns, target, obstacles, unbuildables = map_reader.read_map_file(filename)
    nrows = len(grid)
    ncols = len(grid[0]) if nrows > 0 else 0

    print("Map:")
    for row in grid:
        print(row)
    print("Spawns:", spawns)
    print("Target:", target)
    print("Obstacles:", obstacles)
    print("Unbuildables:", unbuildables)

    best_chrom, best_score = genetic_algorithm.run_ga(
        nrows, ncols, spawns, target, obstacles, unbuildables,
        pop_size=200, generations=1000, mutation_rate=0.01,
        elite_frac=0.5, tournament_k=5, verbose=True
    )

    print("\nBest solution found:")
    genetic_algorithm.print_grid(best_chrom, nrows, ncols, spawns, target, obstacles, unbuildables)
    print(f"Best score (max distance from spawn to target): {best_score}")
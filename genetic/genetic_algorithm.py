import random, heapq, math, time
from typing import List, Tuple, Optional, Set
import numpy as np

# ---------- Utilities: A* shortest path on grid (orthogonal moves) ----------
def astar_shortest_path_length(walls: np.ndarray, src: Tuple[int,int], dst: Tuple[int,int]) -> Optional[int]:
    nrows, ncols = walls.shape
    (sr, sc), (tr, tc) = src, dst
    if (sr,sc) == (tr,tc):
        return 0
    if walls[sr,sc] == 1 or walls[tr,tc] == 1:
        return None
    open_set = [(abs(tr-sr)+abs(tc-sc), 0, sr, sc)]
    gscore = { (sr,sc): 0 }
    visited = set()
    while open_set:
        f, g, r, c = heapq.heappop(open_set)
        if (r,c) in visited:
            continue
        visited.add((r,c))
        if (r,c) == (tr,tc):
            return g
        for dr,dc in ((1,0),(-1,0),(0,1),(0,-1)):
            nr, nc = r+dr, c+dc
            if 0 <= nr < nrows and 0 <= nc < ncols and walls[nr,nc] == 0:
                ng = g + 1
                if ng < gscore.get((nr,nc), 10**9):
                    gscore[(nr,nc)] = ng
                    heur = abs(tr-nr) + abs(tc-nc)
                    heapq.heappush(open_set, (ng + heur, ng, nr, nc))
    return None

# Shortest path on empty grid (for repair)
def bfs_shortest_path_on_empty(nrows:int,ncols:int, src:Tuple[int,int], dst:Tuple[int,int]):
    from collections import deque
    q = deque([src])
    parent = {src: None}
    while q:
        r,c = q.popleft()
        if (r,c) == dst:
            path = []
            cur = dst
            while cur:
                path.append(cur)
                cur = parent[cur]
            path.reverse()
            return path
        for dr,dc in ((1,0),(-1,0),(0,1),(0,-1)):
            nr,nc = r+dr, c+dc
            if 0 <= nr < nrows and 0 <= nc < ncols and (nr,nc) not in parent:
                parent[(nr,nc)] = (r,c)
                q.append((nr,nc))
    return []

# ---------- Helpers ----------
def chromosome_to_grid(chrom: np.ndarray, nrows:int, ncols:int) -> np.ndarray:
    return chrom.reshape((nrows,ncols))

def apply_masks(chrom: np.ndarray, nrows:int, ncols:int,
                spawns: List[Tuple[int,int]], dst:Tuple[int,int],
                obstacles:Set[Tuple[int,int]], unbuildables:Set[Tuple[int,int]]):
    grid = chrom.reshape((nrows,ncols))
    for r,c in obstacles:
        grid[r,c] = 1
    for r,c in unbuildables:
        grid[r,c] = 0
    for r,c in spawns+[dst]:
        grid[r,c] = 0
    return grid.flatten()

def create_random_chrom(nrows:int,ncols:int, wall_prob:float,
                        spawns:List[Tuple[int,int]], dst:Tuple[int,int],
                        obstacles:Set[Tuple[int,int]], unbuildables:Set[Tuple[int,int]]):
    chrom = (np.random.rand(nrows*ncols) < wall_prob).astype(np.uint8)
    chrom = apply_masks(chrom, nrows, ncols, spawns, dst, obstacles, unbuildables)
    return chrom

def repair_chromosome(chrom: np.ndarray, nrows:int, ncols:int,
                      spawns:List[Tuple[int,int]], dst:Tuple[int,int],
                      obstacles:Set[Tuple[int,int]], unbuildables:Set[Tuple[int,int]]):
    grid = chromosome_to_grid(chrom, nrows, ncols)
    feasible = True
    for s in spawns:
        if astar_shortest_path_length(grid, s, dst) is None:
            feasible = False
            path = bfs_shortest_path_on_empty(nrows,ncols,s,dst)
            for (r,c) in path:
                grid[r,c] = 0
    chrom = grid.flatten()
    chrom = apply_masks(chrom, nrows, ncols, spawns, dst, obstacles, unbuildables)
    return chrom

def fitness(chrom: np.ndarray, nrows:int, ncols:int,
            spawns:List[Tuple[int,int]], dst:Tuple[int,int]) -> float:
    grid = chromosome_to_grid(chrom, nrows, ncols)
    dists = [astar_shortest_path_length(grid, s, dst) for s in spawns]
    if any(d is None for d in dists):
        return -1e6
    return float(min(dists))   # worst-case enemy path

def tournament_selection(pop, pop_fitness, k:int=3):
    inds = random.sample(range(len(pop)), k)
    best = max(inds, key=lambda i: pop_fitness[i])
    return pop[best].copy()

def two_point_crossover(a: np.ndarray, b: np.ndarray):
    L = len(a)
    i,j = sorted(random.sample(range(L), 2))
    child1 = a.copy(); child2 = b.copy()
    child1[i:j] = b[i:j]
    child2[i:j] = a[i:j]
    return child1, child2

def mutate(chrom: np.ndarray, mutation_rate: float):
    flips = np.random.rand(len(chrom)) < mutation_rate
    chrom[flips] = 1 - chrom[flips]
    return chrom

# ---------- GA Loop ----------
def run_ga(nrows:int, ncols:int, spawns:List[Tuple[int,int]], dst:Tuple[int,int],
           obstacles:Set[Tuple[int,int]]=set(), unbuildables:Set[Tuple[int,int]]=set(),
           pop_size:int=80, generations:int=200, wall_prob:float=0.20,
           mutation_rate:float=0.01, elite_frac:float=0.05, tournament_k:int=3,
           seed:int=None, verbose:bool=True):
    if seed is not None:
        random.seed(seed); np.random.seed(seed)
    pop = [create_random_chrom(nrows,ncols,wall_prob,spawns,dst,obstacles,unbuildables)
           for _ in range(pop_size)]
    pop = [repair_chromosome(ind, nrows,ncols,spawns,dst,obstacles,unbuildables) for ind in pop]
    pop_fitness = [fitness(ind,nrows,ncols,spawns,dst) for ind in pop]
    best_idx = int(np.argmax(pop_fitness))
    best = pop[best_idx].copy(); best_score = pop_fitness[best_idx]
    if verbose:
        print(f"Init best distance = {best_score}")
    elite_n = max(1, int(math.ceil(elite_frac * pop_size)))
    for gen in range(1, generations+1):
        newpop = []
        elites_idx = sorted(range(len(pop)), key=lambda i: pop_fitness[i], reverse=True)[:elite_n]
        for i in elites_idx:
            newpop.append(pop[i].copy())
        while len(newpop) < pop_size:
            p1 = tournament_selection(pop, pop_fitness, k=tournament_k)
            p2 = tournament_selection(pop, pop_fitness, k=tournament_k)
            c1,c2 = two_point_crossover(p1,p2)
            mutate(c1, mutation_rate); mutate(c2, mutation_rate)
            c1 = repair_chromosome(c1, nrows,ncols,spawns,dst,obstacles,unbuildables)
            c2 = repair_chromosome(c2, nrows,ncols,spawns,dst,obstacles,unbuildables)
            newpop.append(c1)
            if len(newpop) < pop_size:
                newpop.append(c2)
        pop = newpop
        pop_fitness = [fitness(ind,nrows,ncols,spawns,dst) for ind in pop]
        gen_best_idx = int(np.argmax(pop_fitness))
        gen_best_score = pop_fitness[gen_best_idx]
        if gen_best_score > best_score:
            best_score = gen_best_score; best = pop[gen_best_idx].copy()
        if verbose and (gen % max(1, generations//10) == 0 or gen <= 5):
            mean_f = sum(pop_fitness)/len(pop_fitness)
            print(f"Gen {gen:4d}: best={gen_best_score:.1f}, global_best={best_score:.1f}, mean={mean_f:.2f}")
    return best, best_score

# ---------- Pretty-print ----------
def print_grid(chrom: np.ndarray, nrows:int, ncols:int,
               spawns:List[Tuple[int,int]], dst:Tuple[int,int],
               obstacles:Set[Tuple[int,int]], unbuildables:Set[Tuple[int,int]]):
    grid = chromosome_to_grid(chrom, nrows, ncols)
    out = []
    for r in range(nrows):
        row = []
        for c in range(ncols):
            if (r,c) in spawns:
                row.append("S")
            elif (r,c) == dst:
                row.append("T")
            elif (r,c) in obstacles:
                row.append("X")  # immutable obstacle
            elif (r,c) in unbuildables:
                row.append("~")  # cannot build but walkable
            elif grid[r,c] == 1:
                row.append("#")
            else:
                row.append(".")
        out.append("".join(row))
    print("\n".join(out))



# ---------- Example run ----------
if __name__ == "__main__":
    n = 12
    spawns = [(0,3),(0,8)]
    dst = (11,6)
    obstacles = {(5,5),(5,6),(5,7)}
    unbuildables = {(6,6),(7,6)}
    best, best_score = run_ga(n,n,spawns,dst,obstacles,unbuildables,
                              pop_size=50,generations=100,mutation_rate=0.02,
                              seed=42,verbose=True)
    print(f"\nGA finished — best min distance = {best_score}\n")
    print_grid(best,n,n,spawns,dst,obstacles,unbuildables)

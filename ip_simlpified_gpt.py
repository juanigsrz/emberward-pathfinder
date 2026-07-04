import numpy as np
import networkx as nx
from collections import deque
import time
from itertools import combinations

class GridInterdiction:
    def __init__(self, n, source, sink):
        """
        Initialize grid interdiction problem
        n: grid size (n x n)
        source: (row, col) tuple for source
        sink: (row, col) tuple for sink/nexus
        """
        self.n = n
        self.source = source
        self.sink = sink
        self.directions = [(0,1), (1,0), (0,-1), (-1,0)]  # orthogonal moves
        
    def get_neighbors(self, pos, walls):
        """Get valid neighbors considering walls"""
        neighbors = []
        for dx, dy in self.directions:
            new_pos = (pos[0] + dx, pos[1] + dy)
            if (0 <= new_pos[0] < self.n and 
                0 <= new_pos[1] < self.n and
                new_pos not in walls):
                neighbors.append(new_pos)
        return neighbors
    
    def shortest_path_length(self, walls):
        """BFS to find shortest path length given wall configuration"""
        if self.source in walls or self.sink in walls:
            return float('inf')
            
        queue = deque([(self.source, 0)])
        visited = {self.source}
        
        while queue:
            pos, dist = queue.popleft()
            
            if pos == self.sink:
                return dist
                
            for neighbor in self.get_neighbors(pos, walls):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, dist + 1))
        
        return float('inf')  # No path exists
    
    def greedy_interdiction(self, max_walls=None):
        """
        Greedy approach: Iteratively add walls that maximize path length
        """
        walls = set()
        all_positions = [(i, j) for i in range(self.n) for j in range(self.n)]
        all_positions.remove(self.source)
        all_positions.remove(self.sink)
        
        if max_walls is None:
            max_walls = len(all_positions)
        
        best_path_length = self.shortest_path_length(walls)
        
        for _ in range(max_walls):
            best_wall = None
            best_new_length = best_path_length
            
            for pos in all_positions:
                if pos not in walls:
                    test_walls = walls | {pos}
                    path_length = self.shortest_path_length(test_walls)
                    
                    if path_length != float('inf') and path_length > best_new_length:
                        best_new_length = path_length
                        best_wall = pos
            
            if best_wall is None:
                break  # No improvement possible
                
            walls.add(best_wall)
            best_path_length = best_new_length
            all_positions.remove(best_wall)
            
        return walls, best_path_length
    
    def iterative_deepening_search(self, max_depth=10):
        """
        Iterative deepening to find optimal wall placement
        """
        best_walls = set()
        best_length = self.shortest_path_length(set())
        
        for depth in range(1, max_depth + 1):
            walls, length = self.depth_limited_search(set(), depth, best_length)
            if length > best_length:
                best_length = length
                best_walls = walls
                print(f"Depth {depth}: Found path length {length}")
        
        return best_walls, best_length
    
    def depth_limited_search(self, walls, depth, current_best):
        """
        Recursive depth-limited search with pruning
        """
        if depth == 0:
            return walls, self.shortest_path_length(walls)
        
        best_walls = walls
        best_length = self.shortest_path_length(walls)
        
        all_positions = [(i, j) for i in range(self.n) for j in range(self.n)]
        
        for pos in all_positions:
            if pos not in walls and pos != self.source and pos != self.sink:
                new_walls = walls | {pos}
                path_length = self.shortest_path_length(new_walls)
                
                if path_length != float('inf'):  # Still has valid path
                    if path_length > current_best:  # Promising branch
                        result_walls, result_length = self.depth_limited_search(
                            new_walls, depth - 1, max(current_best, path_length)
                        )
                        if result_length > best_length:
                            best_length = result_length
                            best_walls = result_walls
        
        return best_walls, best_length
    
    def local_search(self, initial_walls=None, max_iterations=100):
        """
        Local search with neighborhood moves
        """
        if initial_walls is None:
            initial_walls, _ = self.greedy_interdiction()
        
        current_walls = initial_walls.copy()
        current_length = self.shortest_path_length(current_walls)
        
        all_positions = [(i, j) for i in range(self.n) for j in range(self.n)]
        all_positions.remove(self.source)
        all_positions.remove(self.sink)
        
        for iteration in range(max_iterations):
            improved = False
            
            # Try swapping walls
            for remove_wall in current_walls:
                for add_wall in all_positions:
                    if add_wall not in current_walls:
                        new_walls = (current_walls - {remove_wall}) | {add_wall}
                        new_length = self.shortest_path_length(new_walls)
                        
                        if new_length != float('inf') and new_length > current_length:
                            current_walls = new_walls
                            current_length = new_length
                            improved = True
                            break
                if improved:
                    break
            
            if not improved:
                break
        
        return current_walls, current_length
    
    def cutting_plane_approach(self, max_iterations=10):
        """
        Simplified cutting plane approach using critical path analysis
        """
        walls = set()
        
        for iteration in range(max_iterations):
            # Find current shortest path
            path = self.find_shortest_path(walls)
            if path is None:
                break
                
            # Find best wall position along or near the path
            best_wall = None
            best_length = len(path) - 1
            
            # Consider positions on and adjacent to the path
            candidates = set()
            for pos in path[1:-1]:  # Exclude source and sink
                candidates.add(pos)
                for neighbor in self.get_neighbors(pos, walls):
                    if neighbor != self.source and neighbor != self.sink:
                        candidates.add(neighbor)
            
            for candidate in candidates:
                if candidate not in walls:
                    test_walls = walls | {candidate}
                    length = self.shortest_path_length(test_walls)
                    
                    if length != float('inf') and length > best_length:
                        best_length = length
                        best_wall = candidate
            
            if best_wall is None:
                break
                
            walls.add(best_wall)
            
        return walls, self.shortest_path_length(walls)
    
    def find_shortest_path(self, walls):
        """Find the actual shortest path (not just length)"""
        if self.source in walls or self.sink in walls:
            return None
            
        queue = deque([(self.source, [self.source])])
        visited = {self.source}
        
        while queue:
            pos, path = queue.popleft()
            
            if pos == self.sink:
                return path
                
            for neighbor in self.get_neighbors(pos, walls):
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        
        return None
    
    def visualize(self, walls):
        """Simple text visualization of the grid"""
        grid = [['.' for _ in range(self.n)] for _ in range(self.n)]
        
        # Mark source and sink
        grid[self.source[0]][self.source[1]] = 'S'
        grid[self.sink[0]][self.sink[1]] = 'T'
        
        # Mark walls
        for wall in walls:
            if wall != self.source and wall != self.sink:
                grid[wall[0]][wall[1]] = '#'
        
        # Find and mark shortest path
        path = self.find_shortest_path(walls)
        if path:
            for pos in path:
                if pos != self.source and pos != self.sink and pos not in walls:
                    grid[pos[0]][pos[1]] = '*'
        
        # Print grid
        print("\nGrid visualization:")
        print("S = Source, T = Sink, # = Wall, * = Path, . = Empty")
        for row in grid:
            print(' '.join(row))
        print(f"Shortest path length: {self.shortest_path_length(walls)}")


# Example usage and comparison
def test_algorithms():
    # Create a 7x7 grid with source at top-left and sink at bottom-right
    n = 7
    source = (0, 0)
    sink = (n-1, n-1)
    
    problem = GridInterdiction(n, source, sink)
    
    print(f"Grid size: {n}x{n}")
    print(f"Source: {source}, Sink: {sink}")
    print(f"Initial shortest path length: {problem.shortest_path_length(set())}")
    print("\n" + "="*50 + "\n")
    
    # Test greedy approach
    print("1. GREEDY APPROACH")
    start = time.time()
    greedy_walls, greedy_length = problem.greedy_interdiction(max_walls=10)
    greedy_time = time.time() - start
    print(f"Path length: {greedy_length}")
    print(f"Walls placed: {len(greedy_walls)}")
    print(f"Time: {greedy_time:.3f}s")
    problem.visualize(greedy_walls)
    
    print("\n" + "="*50 + "\n")
    
    # Test cutting plane approach
    print("2. CUTTING PLANE APPROACH")
    start = time.time()
    cutting_walls, cutting_length = problem.cutting_plane_approach(max_iterations=10)
    cutting_time = time.time() - start
    print(f"Path length: {cutting_length}")
    print(f"Walls placed: {len(cutting_walls)}")
    print(f"Time: {cutting_time:.3f}s")
    problem.visualize(cutting_walls)
    
    print("\n" + "="*50 + "\n")
    
    # Test local search (starting from greedy solution)
    print("3. LOCAL SEARCH (from greedy solution)")
    start = time.time()
    local_walls, local_length = problem.local_search(greedy_walls, max_iterations=50)
    local_time = time.time() - start
    print(f"Path length: {local_length}")
    print(f"Walls placed: {len(local_walls)}")
    print(f"Time: {local_time:.3f}s")
    problem.visualize(local_walls)
    
    print("\n" + "="*50 + "\n")
    
    # For smaller grids, test iterative deepening
    if n <= 5:
        print("4. ITERATIVE DEEPENING SEARCH")
        start = time.time()
        ids_walls, ids_length = problem.iterative_deepening_search(max_depth=5)
        ids_time = time.time() - start
        print(f"Path length: {ids_length}")
        print(f"Walls placed: {len(ids_walls)}")
        print(f"Time: {ids_time:.3f}s")
        problem.visualize(ids_walls)


if __name__ == "__main__":
    test_algorithms()
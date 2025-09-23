class PathFinder {
    constructor() {
        this.directions = [
            { x: 0, y: -1 }, // up
            { x: 1, y: 0 },  // right
            { x: 0, y: 1 },  // down
            { x: -1, y: 0 }  // left
        ];
    }
    
    findPath(grid, start, end) {
        const gridSize = grid.length;
        
        if (!this.isValidPosition(start.x, start.y, gridSize) ||
            !this.isValidPosition(end.x, end.y, gridSize) ||
            grid[start.y][start.x] === 1 || 
            grid[end.y][end.x] === 1) {
            return null;
        }
        
        const openSet = [];
        const closedSet = new Set();
        const cameFrom = new Map();
        const gScore = new Map();
        const fScore = new Map();
        
        const startKey = `${start.x},${start.y}`;
        const endKey = `${end.x},${end.y}`;
        
        openSet.push(start);
        gScore.set(startKey, 0);
        fScore.set(startKey, this.heuristic(start, end));
        
        while (openSet.length > 0) {
            let current = openSet.reduce((min, node) => {
                const currentKey = `${node.x},${node.y}`;
                const minKey = `${min.x},${min.y}`;
                return (fScore.get(currentKey) || Infinity) < (fScore.get(minKey) || Infinity) ? node : min;
            });
            
            const currentKey = `${current.x},${current.y}`;
            
            if (currentKey === endKey) {
                return this.reconstructPath(cameFrom, current);
            }
            
            openSet.splice(openSet.indexOf(current), 1);
            closedSet.add(currentKey);
            
            for (const direction of this.directions) {
                const neighbor = {
                    x: current.x + direction.x,
                    y: current.y + direction.y
                };
                
                const neighborKey = `${neighbor.x},${neighbor.y}`;
                
                if (!this.isValidPosition(neighbor.x, neighbor.y, gridSize) ||
                    grid[neighbor.y][neighbor.x] === 1 ||
                    closedSet.has(neighborKey)) {
                    continue;
                }
                
                const tentativeGScore = (gScore.get(currentKey) || Infinity) + 1;
                
                if (!openSet.some(node => node.x === neighbor.x && node.y === neighbor.y)) {
                    openSet.push(neighbor);
                } else if (tentativeGScore >= (gScore.get(neighborKey) || Infinity)) {
                    continue;
                }
                
                cameFrom.set(neighborKey, current);
                gScore.set(neighborKey, tentativeGScore);
                fScore.set(neighborKey, tentativeGScore + this.heuristic(neighbor, end));
            }
        }
        
        return null;
    }
    
    findAllPaths(grid, spawns, nucleus) {
        if (!nucleus || spawns.length === 0) {
            return [];
        }
        
        const paths = [];
        for (const spawn of spawns) {
            const path = this.findPath(grid, spawn, nucleus);
            if (path) {
                paths.push(path);
            }
        }
        
        return paths;
    }
    
    getMaxDistance(paths) {
        if (!paths || paths.length === 0) {
            return 0;
        }
        
        return Math.max(...paths.map(path => path ? path.length - 1 : 0));
    }
    
    canReachNucleus(grid, spawns, nucleus) {
        if (!nucleus || spawns.length === 0) {
            return false;
        }
        
        for (const spawn of spawns) {
            const path = this.findPath(grid, spawn, nucleus);
            if (path) {
                return true;
            }
        }
        
        return false;
    }
    
    wouldBlockAllPaths(grid, spawns, nucleus, blockX, blockY) {
        if (!nucleus || spawns.length === 0) {
            return false;
        }
        
        if (grid[blockY][blockX] !== 0) {
            return false;
        }
        
        if ((blockX === nucleus.x && blockY === nucleus.y) ||
            spawns.some(spawn => spawn.x === blockX && spawn.y === blockY)) {
            return true;
        }
        
        const testGrid = grid.map(row => [...row]);
        testGrid[blockY][blockX] = 1;
        
        for (const spawn of spawns) {
            const path = this.findPath(testGrid, spawn, nucleus);
            if (path) {
                return false;
            }
        }
        
        return true;
    }
    
    heuristic(a, b) {
        return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
    }
    
    isValidPosition(x, y, gridSize) {
        return x >= 0 && x < gridSize && y >= 0 && y < gridSize;
    }
    
    reconstructPath(cameFrom, current) {
        const path = [current];
        let currentKey = `${current.x},${current.y}`;
        
        while (cameFrom.has(currentKey)) {
            current = cameFrom.get(currentKey);
            path.unshift(current);
            currentKey = `${current.x},${current.y}`;
        }
        
        return path;
    }
    
    floodFill(grid, startX, startY) {
        const gridSize = grid.length;
        const visited = Array(gridSize).fill().map(() => Array(gridSize).fill(false));
        const reachable = [];
        
        const queue = [{ x: startX, y: startY }];
        visited[startY][startX] = true;
        
        while (queue.length > 0) {
            const current = queue.shift();
            reachable.push(current);
            
            for (const direction of this.directions) {
                const neighbor = {
                    x: current.x + direction.x,
                    y: current.y + direction.y
                };
                
                if (this.isValidPosition(neighbor.x, neighbor.y, gridSize) &&
                    !visited[neighbor.y][neighbor.x] &&
                    grid[neighbor.y][neighbor.x] !== 1) {
                    
                    visited[neighbor.y][neighbor.x] = true;
                    queue.push(neighbor);
                }
            }
        }
        
        return reachable;
    }
}
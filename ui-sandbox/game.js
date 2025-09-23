class TowerDefenseGame {
    constructor() {
        this.canvas = document.getElementById('gameCanvas');
        this.renderer = new IsometricRenderer(this.canvas);
        this.pathfinder = new PathFinder();
        
        this.gridSize = 12;
        this.currentMode = 'wall';
        this.showDistances = true;
        
        this.isPanning = false;
        this.isDraggingCells = false;
        this.lastMouseX = 0;
        this.lastMouseY = 0;
        this.dragMode = null;
        
        this.initializeGrid();
        this.setupEventListeners();
        this.setupUI();
        
        this.gameLoop();
    }
    
    initializeGrid() {
        this.grid = Array(this.gridSize).fill().map(() => Array(this.gridSize).fill(0));
        this.spawns = [];
        this.nucleus = null;
        this.paths = [];
        
        this.setDefaultLayout();
        this.updatePaths();
    }
    
    setDefaultLayout() {
        const center = Math.floor(this.gridSize / 2);
        
        this.nucleus = { x: center, y: center };
        this.grid[center][center] = 3;
        
        this.spawns = [
            { x: 0, y: 0 },
            { x: this.gridSize - 1, y: 0 },
            { x: 0, y: this.gridSize - 1 },
            { x: this.gridSize - 1, y: this.gridSize - 1 }
        ];
        
        this.spawns.forEach(spawn => {
            this.grid[spawn.y][spawn.x] = 2;
        });
    }
    
    setupEventListeners() {
        this.canvas.addEventListener('click', (e) => this.handleCanvasClick(e));
        this.canvas.addEventListener('mousemove', (e) => this.handleMouseMove(e));
        this.canvas.addEventListener('mousedown', (e) => this.handleMouseDown(e));
        this.canvas.addEventListener('mouseup', (e) => this.handleMouseUp(e));
        this.canvas.addEventListener('wheel', (e) => this.handleWheel(e));
        this.canvas.addEventListener('contextmenu', (e) => e.preventDefault());
        
        window.addEventListener('keydown', (e) => {
            switch(e.key) {
                case '1': this.setMode('wall'); break;
                case '2': this.setMode('spawn'); break;
                case '3': this.setMode('nucleus'); break;
                case 'c': this.clearAll(); break;
                case 'r': this.addRandomWalls(); break;
                case 'd': this.toggleDistances(); break;
            }
        });
    }
    
    setupUI() {
        const gridSizeSlider = document.getElementById('gridSize');
        const gridSizeValue = document.getElementById('gridSizeValue');
        
        gridSizeSlider.addEventListener('input', (e) => {
            this.setGridSize(parseInt(e.target.value));
            gridSizeValue.textContent = e.target.value;
        });
        
        document.getElementById('wallMode').addEventListener('click', () => this.setMode('wall'));
        document.getElementById('spawnMode').addEventListener('click', () => this.setMode('spawn'));
        document.getElementById('nucleusMode').addEventListener('click', () => this.setMode('nucleus'));
        
        document.getElementById('clearAll').addEventListener('click', () => this.clearAll());
        document.getElementById('randomWalls').addEventListener('click', () => this.addRandomWalls());
        
        this.updateUI();
    }
    
    handleCanvasClick(e) {
        if (this.isPanning || this.isDraggingCells) return;
        
        const rect = this.canvas.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;
        
        const worldPos = this.renderer.isoToWorld(mouseX, mouseY);
        
        if (worldPos.x >= 0 && worldPos.x < this.gridSize && 
            worldPos.y >= 0 && worldPos.y < this.gridSize) {
            
            this.handleCellClick(worldPos.x, worldPos.y, e.ctrlKey || e.metaKey);
        }
    }
    
    handleMouseDown(e) {
        this.lastMouseX = e.clientX;
        this.lastMouseY = e.clientY;
        
        if (e.button === 0) {
            this.isDraggingCells = false;
            this.dragMode = null;
        } else if (e.button === 2) {
            this.isPanning = false;
        }
    }
    
    handleMouseUp(e) {
        if (e.button === 0) {
            this.isDraggingCells = false;
            this.dragMode = null;
        } else if (e.button === 2) {
            this.isPanning = false;
        }
    }
    
    handleWheel(e) {
        e.preventDefault();
        
        const rect = this.canvas.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;
        
        const zoomFactor = e.deltaY > 0 ? 0.9 : 1.1;
        const newZoom = this.renderer.zoom * zoomFactor;
        
        this.renderer.setZoom(newZoom, mouseX, mouseY);
        this.updateUI();
    }
    
    handleMouseMove(e) {
        const rect = this.canvas.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;
        
        const deltaX = e.clientX - this.lastMouseX;
        const deltaY = e.clientY - this.lastMouseY;
        
        if (e.buttons === 2) {
            if (Math.abs(deltaX) > 2 || Math.abs(deltaY) > 2) {
                this.isPanning = true;
                this.renderer.pan(deltaX, deltaY);
            }
            
            this.lastMouseX = e.clientX;
            this.lastMouseY = e.clientY;
        } else if (e.buttons === 1 && this.currentMode === 'wall') {
            const worldPos = this.renderer.isoToWorld(mouseX, mouseY);
            
            if (worldPos.x >= 0 && worldPos.x < this.gridSize && 
                worldPos.y >= 0 && worldPos.y < this.gridSize) {
                
                if (Math.abs(deltaX) > 2 || Math.abs(deltaY) > 2) {
                    this.isDraggingCells = true;
                    
                    if (this.dragMode === null) {
                        const currentCell = this.grid[worldPos.y][worldPos.x];
                        this.dragMode = currentCell === 0 ? 'place' : 'remove';
                    }
                    
                    this.handleCellDrag(worldPos.x, worldPos.y);
                }
            }
            
            this.lastMouseX = e.clientX;
            this.lastMouseY = e.clientY;
        }
        
        const worldPos = this.renderer.isoToWorld(mouseX, mouseY);
        this.hoveredCell = worldPos;
    }
    
    handleCellClick(x, y, isCtrlClick) {
        const currentCell = this.grid[y][x];
        
        if (isCtrlClick) {
            this.grid[y][x] = 0;
            this.removeFromArrays(x, y);
            this.updatePaths();
            return;
        }
        
        switch (this.currentMode) {
            case 'wall':
                if (currentCell === 0) {
                    if (!this.pathfinder.wouldBlockAllPaths(this.grid, this.spawns, this.nucleus, x, y)) {
                        this.grid[y][x] = 1;
                    } else {
                        this.showWarning("Cannot block all paths to nucleus!");
                    }
                } else if (currentCell === 1) {
                    this.grid[y][x] = 0;
                }
                break;
                
            case 'spawn':
                if (currentCell === 0) {
                    this.grid[y][x] = 2;
                    this.spawns.push({ x, y });
                } else if (currentCell === 2) {
                    this.grid[y][x] = 0;
                    this.spawns = this.spawns.filter(spawn => !(spawn.x === x && spawn.y === y));
                }
                break;
                
            case 'nucleus':
                if (currentCell === 0) {
                    if (this.nucleus) {
                        this.grid[this.nucleus.y][this.nucleus.x] = 0;
                    }
                    this.grid[y][x] = 3;
                    this.nucleus = { x, y };
                } else if (currentCell === 3) {
                    this.grid[y][x] = 0;
                    this.nucleus = null;
                }
                break;
        }
        
        this.updatePaths();
    }
    
    handleCellDrag(x, y) {
        const currentCell = this.grid[y][x];
        
        if (this.dragMode === 'place' && currentCell === 0) {
            if (!this.pathfinder.wouldBlockAllPaths(this.grid, this.spawns, this.nucleus, x, y)) {
                this.grid[y][x] = 1;
                this.updatePaths();
            }
        } else if (this.dragMode === 'remove' && currentCell === 1) {
            this.grid[y][x] = 0;
            this.updatePaths();
        }
    }
    
    removeFromArrays(x, y) {
        this.spawns = this.spawns.filter(spawn => !(spawn.x === x && spawn.y === y));
        
        if (this.nucleus && this.nucleus.x === x && this.nucleus.y === y) {
            this.nucleus = null;
        }
    }
    
    setMode(mode) {
        this.currentMode = mode;
        
        document.querySelectorAll('.mode-btn').forEach(btn => btn.classList.remove('active'));
        document.getElementById(mode + 'Mode').classList.add('active');
    }
    
    setGridSize(size) {
        this.gridSize = size;
        this.renderer.setGridSize(size);
        this.initializeGrid();
    }
    
    clearAll() {
        this.grid = Array(this.gridSize).fill().map(() => Array(this.gridSize).fill(0));
        this.spawns = [];
        this.nucleus = null;
        this.paths = [];
        this.updatePaths();
    }
    
    addRandomWalls() {
        const wallCount = Math.floor(this.gridSize * this.gridSize * 0.2);
        let placed = 0;
        
        while (placed < wallCount) {
            const x = Math.floor(Math.random() * this.gridSize);
            const y = Math.floor(Math.random() * this.gridSize);
            
            if (this.grid[y][x] === 0 && 
                !this.pathfinder.wouldBlockAllPaths(this.grid, this.spawns, this.nucleus, x, y)) {
                this.grid[y][x] = 1;
                placed++;
            }
        }
        
        this.updatePaths();
    }
    
    updatePaths() {
        this.paths = this.pathfinder.findAllPaths(this.grid, this.spawns, this.nucleus);
        this.updateUI();
    }
    
    updateUI() {
        const maxDistance = this.pathfinder.getMaxDistance(this.paths);
        const spawnCount = this.spawns.length;
        const zoomPercent = Math.round(this.renderer.zoom * 100);
        
        document.getElementById('maxDistance').textContent = maxDistance;
        document.getElementById('spawnCount').textContent = spawnCount;
        document.getElementById('zoomLevel').textContent = zoomPercent + '%';
    }
    
    toggleDistances() {
        this.showDistances = !this.showDistances;
    }
    
    showWarning(message) {
        const existingWarning = document.querySelector('.warning');
        if (existingWarning) {
            existingWarning.remove();
        }
        
        const warning = document.createElement('div');
        warning.className = 'warning';
        warning.textContent = message;
        warning.style.cssText = `
            position: fixed;
            top: 20px;
            right: 20px;
            background: rgba(255, 68, 68, 0.9);
            color: white;
            padding: 15px 20px;
            border-radius: 8px;
            font-weight: bold;
            z-index: 1000;
            animation: slideIn 0.3s ease-out;
        `;
        
        document.body.appendChild(warning);
        
        setTimeout(() => {
            if (warning.parentElement) {
                warning.style.animation = 'slideOut 0.3s ease-in';
                setTimeout(() => warning.remove(), 300);
            }
        }, 3000);
    }
    
    gameLoop() {
        const gameState = {
            grid: this.grid,
            spawns: this.spawns,
            nucleus: this.nucleus,
            paths: this.paths,
            showDistances: this.showDistances
        };
        
        this.renderer.render(gameState);
        requestAnimationFrame(() => this.gameLoop());
    }
}

const style = document.createElement('style');
style.textContent = `
    @keyframes slideIn {
        from { transform: translateX(100%); opacity: 0; }
        to { transform: translateX(0); opacity: 1; }
    }
    
    @keyframes slideOut {
        from { transform: translateX(0); opacity: 1; }
        to { transform: translateX(100%); opacity: 0; }
    }
`;
document.head.appendChild(style);

document.addEventListener('DOMContentLoaded', () => {
    new TowerDefenseGame();
});
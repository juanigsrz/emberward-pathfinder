class IsometricRenderer {
    constructor(canvas) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.baseTileSize = 24;
        this.baseTileHeight = 12;
        this.tileSize = 24;
        this.tileHeight = 12;
        this.offsetX = 0;
        this.offsetY = 0;
        this.gridSize = 12;
        this.zoom = 1.0;
        this.minZoom = 0.3;
        this.maxZoom = 3.0;
        
        this.colors = {
            floor: '#e8e8e8',
            floorDark: '#d0d0d0',
            wall: '#8B4513',
            wallTop: '#A0522D',
            wallSide: '#654321',
            spawn: '#FF4444',
            spawnGlow: '#FF6666',
            nucleus: '#4444FF',
            nucleusGlow: '#6666FF',
            path: '#00FF00',
            pathGlow: '#66FF66',
            grid: '#cccccc40'
        };
        
        this.resizeCanvas();
        window.addEventListener('resize', () => this.resizeCanvas());
    }
    
    resizeCanvas() {
        const container = this.canvas.parentElement;
        this.canvas.width = container.clientWidth - 40;
        this.canvas.height = container.clientHeight - 40;
        
        this.centerGrid();
    }
    
    centerGrid() {
        const gridPixelWidth = this.gridSize * this.tileSize;
        const gridPixelHeight = this.gridSize * this.tileHeight;
        
        this.offsetX = (this.canvas.width - gridPixelWidth) / 2;
        this.offsetY = (this.canvas.height - gridPixelHeight) / 2 + 50;
    }
    
    setZoom(newZoom, centerX = null, centerY = null) {
        const oldZoom = this.zoom;
        this.zoom = Math.max(this.minZoom, Math.min(this.maxZoom, newZoom));
        
        this.tileSize = this.baseTileSize * this.zoom;
        this.tileHeight = this.baseTileHeight * this.zoom;
        
        if (centerX !== null && centerY !== null) {
            const zoomFactor = this.zoom / oldZoom;
            this.offsetX = centerX - (centerX - this.offsetX) * zoomFactor;
            this.offsetY = centerY - (centerY - this.offsetY) * zoomFactor;
        }
    }
    
    pan(deltaX, deltaY) {
        this.offsetX += deltaX;
        this.offsetY += deltaY;
    }
    
    setGridSize(size) {
        this.gridSize = size;
        this.centerGrid();
    }
    
    worldToIso(x, y) {
        const isoX = (x - y) * (this.tileSize / 2);
        const isoY = (x + y) * (this.tileHeight / 2);
        return { x: isoX + this.offsetX, y: isoY + this.offsetY };
    }
    
    isoToWorld(isoX, isoY) {
        const relX = isoX - this.offsetX;
        const relY = isoY - this.offsetY;
        
        const x = (relX / (this.tileSize / 2) + relY / (this.tileHeight / 2)) / 2;
        const y = (relY / (this.tileHeight / 2) - relX / (this.tileSize / 2)) / 2;
        
        return { 
            x: Math.floor(x), 
            y: Math.floor(y) 
        };
    }
    
    drawTile(x, y, color, height = 0) {
        const iso = this.worldToIso(x, y);
        const ctx = this.ctx;
        
        ctx.save();
        
        const tileTop = [
            [iso.x, iso.y - height],
            [iso.x + this.tileSize / 2, iso.y + this.tileHeight / 2 - height],
            [iso.x, iso.y + this.tileHeight - height],
            [iso.x - this.tileSize / 2, iso.y + this.tileHeight / 2 - height]
        ];
        
        ctx.beginPath();
        ctx.moveTo(tileTop[0][0], tileTop[0][1]);
        for (let i = 1; i < tileTop.length; i++) {
            ctx.lineTo(tileTop[i][0], tileTop[i][1]);
        }
        ctx.closePath();
        
        ctx.fillStyle = color;
        ctx.fill();
        ctx.strokeStyle = '#00000020';
        ctx.lineWidth = 1;
        ctx.stroke();
        
        if (height > 0) {
            const rightSide = [
                [iso.x + this.tileSize / 2, iso.y + this.tileHeight / 2 - height],
                [iso.x, iso.y + this.tileHeight - height],
                [iso.x, iso.y + this.tileHeight],
                [iso.x + this.tileSize / 2, iso.y + this.tileHeight / 2]
            ];
            
            ctx.beginPath();
            ctx.moveTo(rightSide[0][0], rightSide[0][1]);
            for (let i = 1; i < rightSide.length; i++) {
                ctx.lineTo(rightSide[i][0], rightSide[i][1]);
            }
            ctx.closePath();
            
            ctx.fillStyle = this.adjustBrightness(color, -0.2);
            ctx.fill();
            ctx.stroke();
            
            const leftSide = [
                [iso.x - this.tileSize / 2, iso.y + this.tileHeight / 2 - height],
                [iso.x, iso.y + this.tileHeight - height],
                [iso.x, iso.y + this.tileHeight],
                [iso.x - this.tileSize / 2, iso.y + this.tileHeight / 2]
            ];
            
            ctx.beginPath();
            ctx.moveTo(leftSide[0][0], leftSide[0][1]);
            for (let i = 1; i < leftSide.length; i++) {
                ctx.lineTo(leftSide[i][0], leftSide[i][1]);
            }
            ctx.closePath();
            
            ctx.fillStyle = this.adjustBrightness(color, -0.4);
            ctx.fill();
            ctx.stroke();
        }
        
        ctx.restore();
    }
    
    adjustBrightness(color, amount) {
        const num = parseInt(color.replace("#", ""), 16);
        const amt = Math.round(2.55 * amount * 100);
        const R = (num >> 16) + amt;
        const G = (num >> 8 & 0x00FF) + amt;
        const B = (num & 0x0000FF) + amt;
        return "#" + (0x1000000 + (R < 255 ? R < 1 ? 0 : R : 255) * 0x10000 +
            (G < 255 ? G < 1 ? 0 : G : 255) * 0x100 +
            (B < 255 ? B < 1 ? 0 : B : 255)).toString(16).slice(1);
    }
    
    drawGlow(x, y, color, radius = 30) {
        const iso = this.worldToIso(x, y);
        const ctx = this.ctx;
        
        ctx.save();
        const gradient = ctx.createRadialGradient(iso.x, iso.y, 0, iso.x, iso.y, radius);
        gradient.addColorStop(0, color + '80');
        gradient.addColorStop(0.5, color + '40');
        gradient.addColorStop(1, color + '00');
        
        ctx.fillStyle = gradient;
        ctx.beginPath();
        ctx.arc(iso.x, iso.y, radius, 0, Math.PI * 2);
        ctx.fill();
        ctx.restore();
    }
    
    drawPath(path, animated = false) {
        if (!path || path.length < 2) return;
        
        const ctx = this.ctx;
        ctx.save();
        
        const time = Date.now() * 0.005;
        
        for (let i = 0; i < path.length - 1; i++) {
            const current = this.worldToIso(path[i].x, path[i].y);
            const next = this.worldToIso(path[i + 1].x, path[i + 1].y);
            
            const opacity = animated ? 
                (Math.sin(time + i * 0.5) * 0.3 + 0.7) : 0.8;
            
            ctx.strokeStyle = `rgba(0, 255, 0, ${opacity})`;
            ctx.lineWidth = 4;
            ctx.lineCap = 'round';
            
            ctx.beginPath();
            ctx.moveTo(current.x, current.y);
            ctx.lineTo(next.x, next.y);
            ctx.stroke();
            
            if (animated) {
                ctx.strokeStyle = `rgba(255, 255, 255, ${opacity * 0.5})`;
                ctx.lineWidth = 2;
                ctx.stroke();
            }
        }
        
        ctx.restore();
    }
    
    drawDistanceText(x, y, distance) {
        const iso = this.worldToIso(x, y);
        const ctx = this.ctx;
        
        ctx.save();
        ctx.fillStyle = '#FFFFFF';
        ctx.strokeStyle = '#000000';
        ctx.lineWidth = 2;
        ctx.font = 'bold 12px Arial';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        
        const text = distance.toString();
        ctx.strokeText(text, iso.x, iso.y - 5);
        ctx.fillText(text, iso.x, iso.y - 5);
        ctx.restore();
    }
    
    clear() {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        
        this.ctx.fillStyle = 'rgba(0, 0, 0, 0.1)';
        this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
    }
    
    render(gameState) {
        this.clear();
        
        const { grid, spawns, nucleus, paths, showDistances } = gameState;
        
        for (let y = 0; y < this.gridSize; y++) {
            for (let x = 0; x < this.gridSize; x++) {
                const cell = grid[y][x];
                let color = this.colors.floor;
                let height = 0;
                
                if ((x + y) % 2 === 0) {
                    color = this.colors.floorDark;
                }
                
                if (cell === 1) {
                    color = this.colors.wall;
                    height = 20;
                }
                
                this.drawTile(x, y, color, height);
                
                if (cell === 2) {
                    this.drawGlow(x, y, this.colors.spawnGlow, 25);
                    this.drawTile(x, y, this.colors.spawn, 5);
                }
                
                if (cell === 3) {
                    this.drawGlow(x, y, this.colors.nucleusGlow, 30);
                    this.drawTile(x, y, this.colors.nucleus, 8);
                }
            }
        }
        
        if (paths && paths.length > 0) {
            paths.forEach(path => {
                if (path && path.length > 0) {
                    this.drawPath(path, true);
                }
            });
        }
        
        if (showDistances && paths) {
            paths.forEach(path => {
                if (path && path.length > 0) {
                    const lastCell = path[path.length - 1];
                    if (lastCell) {
                        this.drawDistanceText(lastCell.x, lastCell.y, path.length - 1);
                    }
                }
            });
        }
    }
}
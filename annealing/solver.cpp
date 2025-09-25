#include "solver.hpp"
#include <iostream>
#include <fstream>
#include <queue>
#include <random>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <stdexcept>
#include <iomanip>
#include <cstdint>
#include <climits>

MapData AnnealingSolver::readMapFile(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        throw runtime_error("Cannot open file: " + filename);
    }
    
    MapData data;
    string line;
    
    while (getline(file, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (!line.empty()) {
            data.lines.push_back(line);
        }
    }
    
    bool targetFound = false;
    
    for (int r = 0; r < data.lines.size(); r++) {
        for (int c = 0; c < data.lines[r].size(); c++) {
            char ch = data.lines[r][c];
            if (ch == 'S') {
                data.spawns.push_back({r, c});
            } else if (ch == 'T') {
                data.target = {r, c};
                targetFound = true;
            } else if (ch == '#') {
                data.obstacles.insert({r, c});
            } else if (ch == 'X') {
                data.unbuildables.insert({r, c});
            }
        }
    }
    
    if (data.spawns.empty()) {
        throw runtime_error("No spawn (S) found in map file");
    }
    if (!targetFound) {
        throw runtime_error("No target (T) found in map file");
    }
    
    return data;
}

vector<vector<int>> AnnealingSolver::computeDistances(const vector<string>& grid, const pair<int, int>& target) {
    int R = grid.size();
    int C = grid[0].size();
    
    vector<vector<int>> dist(R, vector<int>(C, -1));
    queue<pair<int, int>> q;
    
    q.push(target);
    dist[target.first][target.second] = 0;
    
    int dx[] = {-1, 1, 0, 0};
    int dy[] = {0, 0, -1, 1};
    
    while (!q.empty()) {
        auto [r, c] = q.front();
        q.pop();
        
        for (int i = 0; i < 4; i++) {
            int nr = r + dx[i];
            int nc = c + dy[i];
            
            if (nr >= 0 && nr < R && nc >= 0 && nc < C) {
                if (dist[nr][nc] == -1 && (grid[nr][nc] == '.' || grid[nr][nc] == 'S' || 
                                          grid[nr][nc] == 'T' || grid[nr][nc] == 'X')) {
                    dist[nr][nc] = dist[r][c] + 1;
                    q.push({nr, nc});
                }
            }
        }
    }
    
    return dist;
}

int AnnealingSolver::evaluate(const vector<string>& grid, const vector<pair<int, int>>& spawns, const pair<int, int>& target) {
    auto dist = computeDistances(grid, target);
    int minDist = INT_MAX;
    
    for (auto [sr, sc] : spawns) {
        if (dist[sr][sc] == -1) {
            return -1000000;
        }
        minDist = min(minDist, dist[sr][sc]);
    }
    
    return minDist;
}

vector<pair<int, int>> AnnealingSolver::rotateOffsets(const vector<pair<int, int>>& offsets) {
    vector<pair<int, int>> rotated;
    for (auto [x, y] : offsets) {
        rotated.push_back({-y, x});
    }
    return rotated;
}

vector<vector<pair<int, int>>> AnnealingSolver::allRotations(const vector<pair<int, int>>& baseOffsets) {
    vector<vector<pair<int, int>>> rotations;
    set<vector<pair<int, int>>> seen;
    
    auto offsets = baseOffsets;
    for (int i = 0; i < 4; i++) {
        auto sorted_offsets = offsets;
        sort(sorted_offsets.begin(), sorted_offsets.end());
        
        if (seen.find(sorted_offsets) == seen.end()) {
            seen.insert(sorted_offsets);
            rotations.push_back(offsets);
        }
        offsets = rotateOffsets(offsets);
    }
    
    return rotations;
}

vector<Piece> AnnealingSolver::initializePieces() {
    vector<Piece> pieces;
    
    /*
    pieces.push_back({'I', allRotations({{0,0},{0,1},{0,2},{0,3}})});
    pieces.push_back({'O', allRotations({{0,0},{0,1},{1,0},{1,1}})});
    pieces.push_back({'M', allRotations({{0,0},{0,1},{0,2},{1,1}})});
    pieces.push_back({'N', allRotations({{0,1},{0,2},{1,0},{1,1}})});
    pieces.push_back({'Z', allRotations({{0,0},{0,1},{1,1},{1,2}})});
    pieces.push_back({'J', allRotations({{0,0},{1,0},{1,1},{1,2}})});
    pieces.push_back({'L', allRotations({{0,2},{1,0},{1,1},{1,2}})});
    */

    vector<pair<int, int>> I = {{0,0},{0,1},{0,2},{0,3},{0,4}}; // 1x5 line
    vector<pair<int, int>> L = {{0,0},{1,0},{0,1}}; // 2x2 L shape
    
    pieces.push_back({'I', {I}}); // 1x5
    pieces.push_back({'L', allRotations(L)}); // 1x3
    
    
    return pieces;
}

vector<pair<int, int>> AnnealingSolver::canPlace(const vector<string>& grid, int r, int c, 
                                                const vector<pair<int, int>>& shape,
                                                const set<pair<int, int>>& obstacles,
                                                const set<pair<int, int>>& unbuildables) {
    int R = grid.size();
    int C = grid[0].size();
    vector<pair<int, int>> coords;
    
    for (auto [dr, dc] : shape) {
        int rr = r + dr;
        int cc = c + dc;
        
        if (rr < 0 || rr >= R || cc < 0 || cc >= C) {
            return {};
        }
        
        if (grid[rr][cc] != '.' || obstacles.count({rr, cc}) || unbuildables.count({rr, cc})) {
            return {};
        }
        
        coords.push_back({rr, cc});
    }
    
    return coords;
}

void AnnealingSolver::placePiece(vector<string>& grid, const vector<pair<int, int>>& coords, char symbol) {
    for (auto [r, c] : coords) {
        grid[r][c] = symbol;
    }
}

void AnnealingSolver::removePiece(vector<string>& grid, const vector<pair<int, int>>& coords) {
    for (auto [r, c] : coords) {
        grid[r][c] = '.';
    }
}

pair<vector<string>, int> AnnealingSolver::simulatedAnnealing(const vector<string>& lines,
                                                             const vector<pair<int, int>>& spawns,
                                                             const pair<int, int>& target,
                                                             const set<pair<int, int>>& obstacles,
                                                             const set<pair<int, int>>& unbuildables,
                                                             int maxIter, double T0, double alpha) {
    int R = lines.size();
    int C = lines[0].size();
    
    vector<string> currentGrid = lines;
    vector<string> bestGrid = lines;
    
    int bestScore = evaluate(currentGrid, spawns, target);
    int currentScore = bestScore;
    
    vector<PlacedPiece> placedPieces;
    vector<Piece> pieces = initializePieces();
    
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis(0.0, 1.0);
    uniform_int_distribution<> piece_dis(0, pieces.size() - 1);
    
    double T = T0;
    auto startTime = chrono::high_resolution_clock::now();
    
    for (int it = 0; it < maxIter; it++) {
        bool addMove = placedPieces.empty() || dis(gen) < 0.6;
        
        if (addMove) {
            int pieceIdx = piece_dis(gen);
            auto& piece = pieces[pieceIdx];
            
            uniform_int_distribution<> orient_dis(0, piece.orientations.size() - 1);
            auto& shape = piece.orientations[orient_dis(gen)];
            
            uniform_int_distribution<> r_dis(0, R - 1);
            uniform_int_distribution<> c_dis(0, C - 1);
            int r = r_dis(gen);
            int c = c_dis(gen);
            
            auto coords = canPlace(currentGrid, r, c, shape, obstacles, unbuildables);
            if (coords.empty()) {
                continue;
            }
            
            placePiece(currentGrid, coords, piece.symbol);
            int score = evaluate(currentGrid, spawns, target);
            
            int delta = score - currentScore;
            if (delta >= 0 || dis(gen) < exp(delta / T)) {
                currentScore = score;
                placedPieces.push_back({coords, piece.symbol});
                if (score > bestScore) {
                    bestScore = score;
                    bestGrid = currentGrid;
                }
            } else {
                removePiece(currentGrid, coords);
            }
        } else {
            uniform_int_distribution<> placed_dis(0, placedPieces.size() - 1);
            int idx = placed_dis(gen);
            auto piece = placedPieces[idx];
            
            removePiece(currentGrid, piece.coords);
            int score = evaluate(currentGrid, spawns, target);
            
            int delta = score - currentScore;
            if (delta >= 0 || dis(gen) < exp(delta / T)) {
                currentScore = score;
                placedPieces.erase(placedPieces.begin() + idx);
                if (score > bestScore) {
                    bestScore = score;
                    bestGrid = currentGrid;
                }
            } else {
                placePiece(currentGrid, piece.coords, piece.symbol);
            }
        }
        
        T *= alpha;
        
        if (it % 5000 == 0) {
            cout << "Iter " << it << ", Temp=" << fixed << setprecision(3) << T 
                 << ", Best=" << bestScore << endl;
        }
    }
    
    auto endTime = chrono::high_resolution_clock::now();
    auto elapsed = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count() / 1000.0;
    
    cout << "SA finished in " << fixed << setprecision(2) << elapsed 
         << "s — best distance = " << bestScore << endl;
    
    return {bestGrid, bestScore};
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        cout << "Usage: " << argv[0] << " map.txt" << endl;
        return 1;
    }
    
    try {
        string mapfile = argv[1];
        auto data = AnnealingSolver::readMapFile(mapfile);
        
        auto [bestGrid, score] = AnnealingSolver::simulatedAnnealing(
            data.lines, data.spawns, data.target, data.obstacles, data.unbuildables, 10000000, 10.0, 0.9999);
        
        cout << "\nBest solution:" << endl;
        for (const auto& row : bestGrid) {
            cout << row << endl;
        }
        
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }
    
    return 0;
}
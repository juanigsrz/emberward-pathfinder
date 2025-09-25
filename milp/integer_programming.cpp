#include "integer_programming.hpp"
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <iomanip>
#include <cmath>
#include <algorithm>

MapData IntegerProgrammingSolver::readMapFile(const string& filename) {
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
    
    if (data.lines.empty()) {
        throw runtime_error("Empty map file");
    }
    
    data.rows = data.lines.size();
    data.cols = data.lines[0].size();
    
    bool targetFound = false;
    
    for (int r = 0; r < data.rows; r++) {
        for (int c = 0; c < (int)data.lines[r].size(); c++) {
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

vector<pair<int, int>> IntegerProgrammingSolver::makeGridNodes(int rows, int cols) {
    vector<pair<int, int>> nodes;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            nodes.push_back({r, c});
        }
    }
    return nodes;
}

vector<pair<int, int>> IntegerProgrammingSolver::getNeighbors(const pair<int, int>& node, int rows, int cols) {
    vector<pair<int, int>> neighbors;
    int r = node.first, c = node.second;
    
    vector<pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    
    for (auto [dr, dc] : directions) {
        int nr = r + dr, nc = c + dc;
        if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
            neighbors.push_back({nr, nc});
        }
    }
    
    return neighbors;
}

void IntegerProgrammingSolver::printSolutionGrid(const vector<string>& originalGrid,
                                               const map<pair<int, int>, int>& y_val,
                                               const pair<int, int>& /* spawn */,
                                               const pair<int, int>& /* target */) {
    cout << "\nSolution grid:" << endl;
    
    for (int r = 0; r < (int)originalGrid.size(); r++) {
        string line = "";
        for (int c = 0; c < (int)originalGrid[r].size(); c++) {
            pair<int, int> pos = {r, c};
            char originalChar = originalGrid[r][c];
            
            if (originalChar == 'S' || originalChar == 'T' || originalChar == '#' || originalChar == 'X') {
                line += originalChar;
            } else if (y_val.count(pos) && y_val.at(pos) == 1) {
                line += "1";  // Wall placed by solver
            } else {
                line += ".";  // Free space
            }
        }
        cout << line << endl;
    }
}

void IntegerProgrammingSolver::printDistanceMap(const map<pair<int, int>, int>& d_val, int rows, int cols) {
    cout << "\nDistance map:" << endl;
    for (int r = 0; r < rows; r++) {
        string line = "";
        for (int c = 0; c < cols; c++) {
            int dist = d_val.count({r, c}) ? d_val.at({r, c}) : 0;
            line += to_string(dist);
            if (dist < 10) line += " ";
            line += " ";
        }
        cout << line << endl;
    }
}

int IntegerProgrammingSolver::buildAndSolveFromFile(const string& filename, int timeLimit) {
    MapData mapData = readMapFile(filename);
    return buildAndSolve(mapData, timeLimit);
}

int IntegerProgrammingSolver::buildAndSolve(const MapData& mapData, int timeLimit) {
    int rows = mapData.rows;
    int cols = mapData.cols;
    pair<int, int> spawn = mapData.spawns[0];  // Use first spawn
    pair<int, int> target = mapData.target;
    
    // Only consider free spaces as potential wall locations
    vector<pair<int, int>> V;
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            pair<int, int> pos = {r, c};
            char ch = mapData.lines[r][c];
            if (ch == '.' || ch == 'S' || ch == 'T') {
                V.push_back(pos);
            }
        }
    }
    
    int N_nodes = V.size();
    int N_max = N_nodes - 1;
    int M = N_max;
    
    try {
        GRBEnv env = GRBEnv();
        GRBModel model = GRBModel(env);
        
        model.set(GRB_StringAttr_ModelName, "grid_parent_patch");
        model.set(GRB_IntParam_OutputFlag, 1);
        model.set(GRB_DoubleParam_TimeLimit, timeLimit);
        
        // Variables
        map<pair<int, int>, GRBVar> y, d;
        map<pair<pair<int, int>, pair<int, int>>, GRBVar> p;
        
        for (auto v : V) {
            string y_name = "y_" + to_string(v.first) + "_" + to_string(v.second);
            string d_name = "d_" + to_string(v.first) + "_" + to_string(v.second);
            
            y[v] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, y_name);
            d[v] = model.addVar(0.0, N_max, 0.0, GRB_INTEGER, d_name);
        }
        
        for (auto u : V) {
            if (u == target) continue;
            
            auto neighbors = getNeighbors(u, rows, cols);
            for (auto v : neighbors) {
                // Only add parent variables for neighbors that are in V (valid positions)
                if (find(V.begin(), V.end(), v) != V.end()) {
                    string p_name = "p_" + to_string(u.first) + "_" + to_string(u.second) + 
                                   "_" + to_string(v.first) + "_" + to_string(v.second);
                    p[{u, v}] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, p_name);
                }
            }
        }
        
        // Constraints
        model.addConstr(y[spawn] == 0);
        model.addConstr(y[target] == 0);
        model.addConstr(d[target] == 0);
        
        for (auto v : V) {
            model.addConstr(d[v] <= N_max * (1 - y[v]));
        }
        
        // Core patch: upper bounds relative to all neighbors
        for (auto u : V) {
            auto neighbors = getNeighbors(u, rows, cols);
            for (auto v : neighbors) {
                string ub_name = "ub_" + to_string(u.first) + "_" + to_string(u.second) + 
                               "_" + to_string(v.first) + "_" + to_string(v.second);
                model.addConstr(d[u] <= d[v] + 1 + M * (y[u] + y[v]), ub_name);
            }
        }
        
        // Parent constraints: each free non-target picks one parent
        for (auto u : V) {
            if (u == target) continue;
            
            auto neighs = getNeighbors(u, rows, cols);
            
            GRBLinExpr sum_p;
            for (auto v : neighs) {
                if (find(V.begin(), V.end(), v) != V.end()) {
                    sum_p += p[{u, v}];
                }
            }
            model.addConstr(sum_p == 1 - y[u]);
            
            for (auto v : neighs) {
                if (find(V.begin(), V.end(), v) != V.end()) {
                    model.addConstr(p[{u, v}] <= 1 - y[v]);
                    // enforce equality when chosen
                    model.addConstr(d[u] - d[v] - 1 <= M * (1 - p[{u, v}]));
                    model.addConstr(d[u] - d[v] - 1 >= -M * (1 - p[{u, v}]));
                }
            }
        }
        
        // Objective
        GRBLinExpr obj = d[spawn];
        model.setObjective(obj, GRB_MAXIMIZE);
        
        model.optimize();
        
        int status = model.get(GRB_IntAttr_Status);
        if (status != GRB_OPTIMAL && status != GRB_TIME_LIMIT && status != GRB_SUBOPTIMAL) {
            throw runtime_error("Solver ended with status " + to_string(status));
        }
        
        map<pair<int, int>, int> y_val, d_val;
        
        for (auto v : V) {
            y_val[v] = (int)round(y[v].get(GRB_DoubleAttr_X));
            d_val[v] = (int)round(d[v].get(GRB_DoubleAttr_X));
        }
        
        cout << "Objective (d[spawn]) = " << d_val[spawn] << endl;
        
        printSolutionGrid(mapData.lines, y_val, spawn, target);
        printDistanceMap(d_val, rows, cols);
        
        return d_val[spawn];
        
    } catch (GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
        throw;
    }
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        cout << "Usage: " << argv[0] << " map.txt" << endl;
        return 1;
    }
    
    try {
        string mapfile = argv[1];
        IntegerProgrammingSolver::buildAndSolveFromFile(mapfile, 120);
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }
    
    return 0;
}
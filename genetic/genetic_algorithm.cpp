#include "genetic_algorithm.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <random>
#include <queue>
#include <cmath>
#include <climits>
#include <map>
#include <numeric>

MapData GeneticAlgorithm::readMapFile(const string& filename) {
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

optional<int> GeneticAlgorithm::astarShortestPathLength(const vector<vector<int>>& walls, 
                                                       pair<int, int> src, pair<int, int> dst) {
    int nrows = walls.size();
    int ncols = walls[0].size();
    auto [sr, sc] = src;
    auto [tr, tc] = dst;
    
    if (src == dst) return 0;
    if (walls[sr][sc] == 1 || walls[tr][tc] == 1) return nullopt;
    
    // Priority queue: (f_score, g_score, r, c)
    priority_queue<tuple<int, int, int, int>, 
                   vector<tuple<int, int, int, int>>, 
                   greater<tuple<int, int, int, int>>> openSet;
    
    map<pair<int, int>, int> gScore;
    set<pair<int, int>> visited;
    
    int heuristic = abs(tr - sr) + abs(tc - sc);
    openSet.push({heuristic, 0, sr, sc});
    gScore[{sr, sc}] = 0;
    
    vector<pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    
    while (!openSet.empty()) {
        auto [f, g, r, c] = openSet.top();
        openSet.pop();
        
        if (visited.count({r, c})) continue;
        visited.insert({r, c});
        
        if (r == tr && c == tc) return g;
        
        for (auto [dr, dc] : directions) {
            int nr = r + dr, nc = c + dc;
            if (nr >= 0 && nr < nrows && nc >= 0 && nc < ncols && walls[nr][nc] == 0) {
                int ng = g + 1;
                int currentG = gScore.count({nr, nc}) ? gScore[{nr, nc}] : INT_MAX;
                if (ng < currentG) {
                    gScore[{nr, nc}] = ng;
                    int heur = abs(tr - nr) + abs(tc - nc);
                    openSet.push({ng + heur, ng, nr, nc});
                }
            }
        }
    }
    
    return nullopt;
}

vector<pair<int, int>> GeneticAlgorithm::bfsShortestPathOnEmpty(int nrows, int ncols, 
                                                               pair<int, int> src, pair<int, int> dst) {
    queue<pair<int, int>> q;
    map<pair<int, int>, pair<int, int>> parent;
    
    q.push(src);
    parent[src] = {-1, -1}; // sentinel
    
    vector<pair<int, int>> directions = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    
    while (!q.empty()) {
        auto [r, c] = q.front();
        q.pop();
        
        if (make_pair(r, c) == dst) {
            vector<pair<int, int>> path;
            pair<int, int> cur = dst;
            while (cur != make_pair(-1, -1)) {
                path.push_back(cur);
                cur = parent[cur];
            }
            reverse(path.begin(), path.end());
            return path;
        }
        
        for (auto [dr, dc] : directions) {
            int nr = r + dr, nc = c + dc;
            if (nr >= 0 && nr < nrows && nc >= 0 && nc < ncols && !parent.count({nr, nc})) {
                parent[{nr, nc}] = {r, c};
                q.push({nr, nc});
            }
        }
    }
    
    return {};
}

vector<vector<int>> GeneticAlgorithm::chromosomeToGrid(const vector<int>& chrom, int nrows, int ncols) {
    vector<vector<int>> grid(nrows, vector<int>(ncols));
    for (int i = 0; i < nrows * ncols; i++) {
        grid[i / ncols][i % ncols] = chrom[i];
    }
    return grid;
}

vector<int> GeneticAlgorithm::applyMasks(vector<int> chrom, int nrows, int ncols,
                                        const vector<pair<int, int>>& spawns,
                                        const pair<int, int>& target,
                                        const set<pair<int, int>>& obstacles,
                                        const set<pair<int, int>>& unbuildables) {
    auto grid = chromosomeToGrid(chrom, nrows, ncols);
    
    // Apply obstacles as walls
    for (auto [r, c] : obstacles) {
        grid[r][c] = 1;
    }
    
    // Apply unbuildables as free space
    for (auto [r, c] : unbuildables) {
        grid[r][c] = 0;
    }
    
    // Keep spawns and target as free space
    for (auto [r, c] : spawns) {
        grid[r][c] = 0;
    }
    auto [tr, tc] = target;
    grid[tr][tc] = 0;
    
    // Flatten back to chromosome
    for (int i = 0; i < nrows * ncols; i++) {
        chrom[i] = grid[i / ncols][i % ncols];
    }
    
    return chrom;
}

vector<int> GeneticAlgorithm::createRandomChrom(int nrows, int ncols, double wallProb,
                                               const vector<pair<int, int>>& spawns,
                                               const pair<int, int>& target,
                                               const set<pair<int, int>>& obstacles,
                                               const set<pair<int, int>>& unbuildables) {
    static random_device rd;
    static mt19937 gen(rd());
    uniform_real_distribution<> dis(0.0, 1.0);
    
    vector<int> chrom(nrows * ncols);
    for (int i = 0; i < nrows * ncols; i++) {
        chrom[i] = dis(gen) < wallProb ? 1 : 0;
    }
    
    return applyMasks(chrom, nrows, ncols, spawns, target, obstacles, unbuildables);
}

vector<int> GeneticAlgorithm::repairChromosome(const vector<int>& chrom, int nrows, int ncols,
                                              const vector<pair<int, int>>& spawns,
                                              const pair<int, int>& target,
                                              const set<pair<int, int>>& obstacles,
                                              const set<pair<int, int>>& unbuildables) {
    auto grid = chromosomeToGrid(chrom, nrows, ncols);
    
    for (auto spawn : spawns) {
        if (!astarShortestPathLength(grid, spawn, target)) {
            auto path = bfsShortestPathOnEmpty(nrows, ncols, spawn, target);
            for (auto [r, c] : path) {
                grid[r][c] = 0;
            }
        }
    }
    
    // Flatten and apply masks
    vector<int> repairedChrom(nrows * ncols);
    for (int i = 0; i < nrows * ncols; i++) {
        repairedChrom[i] = grid[i / ncols][i % ncols];
    }
    
    return applyMasks(repairedChrom, nrows, ncols, spawns, target, obstacles, unbuildables);
}

double GeneticAlgorithm::fitness(const vector<int>& chrom, int nrows, int ncols,
                                const vector<pair<int, int>>& spawns,
                                const pair<int, int>& target) {
    auto grid = chromosomeToGrid(chrom, nrows, ncols);
    vector<optional<int>> distances;
    
    for (auto spawn : spawns) {
        auto dist = astarShortestPathLength(grid, spawn, target);
        if (!dist) return -1e6;
        distances.push_back(dist);
    }
    
    int minDist = INT_MAX;
    for (auto dist : distances) {
        minDist = min(minDist, dist.value());
    }
    
    return static_cast<double>(minDist);
}

vector<int> GeneticAlgorithm::tournamentSelection(const vector<vector<int>>& pop,
                                                 const vector<double>& popFitness, int k) {
    static random_device rd;
    static mt19937 gen(rd());
    uniform_int_distribution<> dis(0, pop.size() - 1);
    
    vector<int> indices;
    for (int i = 0; i < k; i++) {
        indices.push_back(dis(gen));
    }
    
    int best = *max_element(indices.begin(), indices.end(), 
                           [&](int a, int b) { return popFitness[a] < popFitness[b]; });
    
    return pop[best];
}

pair<vector<int>, vector<int>> GeneticAlgorithm::twoPointCrossover(const vector<int>& a, const vector<int>& b) {
    static random_device rd;
    static mt19937 gen(rd());
    uniform_int_distribution<> dis(0, a.size() - 1);
    
    int i = dis(gen), j = dis(gen);
    if (i > j) swap(i, j);
    
    vector<int> child1 = a, child2 = b;
    for (int idx = i; idx < j; idx++) {
        child1[idx] = b[idx];
        child2[idx] = a[idx];
    }
    
    return {child1, child2};
}

void GeneticAlgorithm::mutate(vector<int>& chrom, double mutationRate) {
    static random_device rd;
    static mt19937 gen(rd());
    uniform_real_distribution<> dis(0.0, 1.0);
    
    for (int i = 0; i < (int)chrom.size(); i++) {
        if (dis(gen) < mutationRate) {
            chrom[i] = 1 - chrom[i];
        }
    }
}

pair<vector<int>, double> GeneticAlgorithm::runGA(int nrows, int ncols,
                                                 const vector<pair<int, int>>& spawns,
                                                 const pair<int, int>& target,
                                                 const set<pair<int, int>>& obstacles,
                                                 const set<pair<int, int>>& unbuildables,
                                                 int popSize, int generations,
                                                 double wallProb, double mutationRate,
                                                 double eliteFrac, int tournamentK,
                                                 int seed, bool verbose) {
    if (seed >= 0) {
        srand(seed);
    }
    
    // Initialize population
    vector<vector<int>> pop;
    for (int i = 0; i < popSize; i++) {
        auto chrom = createRandomChrom(nrows, ncols, wallProb, spawns, target, obstacles, unbuildables);
        pop.push_back(repairChromosome(chrom, nrows, ncols, spawns, target, obstacles, unbuildables));
    }
    
    vector<double> popFitness(popSize);
    for (int i = 0; i < popSize; i++) {
        popFitness[i] = fitness(pop[i], nrows, ncols, spawns, target);
    }
    
    int bestIdx = max_element(popFitness.begin(), popFitness.end()) - popFitness.begin();
    vector<int> best = pop[bestIdx];
    double bestScore = popFitness[bestIdx];
    
    if (verbose) {
        cout << "Init best distance = " << bestScore << endl;
    }
    
    int eliteN = max(1, (int)ceil(eliteFrac * popSize));
    
    for (int gen = 1; gen <= generations; gen++) {
        vector<vector<int>> newPop;
        
        // Elite selection
        vector<int> eliteIndices(popSize);
        iota(eliteIndices.begin(), eliteIndices.end(), 0);
        sort(eliteIndices.begin(), eliteIndices.end(), 
             [&](int a, int b) { return popFitness[a] > popFitness[b]; });
        
        for (int i = 0; i < eliteN; i++) {
            newPop.push_back(pop[eliteIndices[i]]);
        }
        
        // Generate offspring
        while ((int)newPop.size() < popSize) {
            auto p1 = tournamentSelection(pop, popFitness, tournamentK);
            auto p2 = tournamentSelection(pop, popFitness, tournamentK);
            auto [c1, c2] = twoPointCrossover(p1, p2);
            
            mutate(c1, mutationRate);
            mutate(c2, mutationRate);
            
            c1 = repairChromosome(c1, nrows, ncols, spawns, target, obstacles, unbuildables);
            c2 = repairChromosome(c2, nrows, ncols, spawns, target, obstacles, unbuildables);
            
            newPop.push_back(c1);
            if ((int)newPop.size() < popSize) {
                newPop.push_back(c2);
            }
        }
        
        pop = newPop;
        for (int i = 0; i < popSize; i++) {
            popFitness[i] = fitness(pop[i], nrows, ncols, spawns, target);
        }
        
        int genBestIdx = max_element(popFitness.begin(), popFitness.end()) - popFitness.begin();
        double genBestScore = popFitness[genBestIdx];
        
        if (genBestScore > bestScore) {
            bestScore = genBestScore;
            best = pop[genBestIdx];
        }
        
        if (verbose && (gen % max(1, generations / 10) == 0 || gen <= 5)) {
            double meanFitness = 0;
            for (double f : popFitness) meanFitness += f;
            meanFitness /= popSize;
            
            cout << "Gen " << gen << ": best=" << genBestScore 
                 << ", global_best=" << bestScore 
                 << ", mean=" << meanFitness << endl;
        }
    }
    
    return {best, bestScore};
}

void GeneticAlgorithm::printGrid(const vector<int>& chrom, int nrows, int ncols,
                                const vector<pair<int, int>>& spawns,
                                const pair<int, int>& target,
                                const set<pair<int, int>>& obstacles,
                                const set<pair<int, int>>& unbuildables) {
    auto grid = chromosomeToGrid(chrom, nrows, ncols);
    
    for (int r = 0; r < nrows; r++) {
        string row = "";
        for (int c = 0; c < ncols; c++) {
            if (find(spawns.begin(), spawns.end(), make_pair(r, c)) != spawns.end()) {
                row += "S";
            } else if (make_pair(r, c) == target) {
                row += "T";
            } else if (obstacles.count({r, c})) {
                row += "X";
            } else if (unbuildables.count({r, c})) {
                row += "~";
            } else if (grid[r][c] == 1) {
                row += "#";
            } else {
                row += ".";
            }
        }
        cout << row << endl;
    }
}

int GeneticAlgorithm::runGAFromFile(const string& filename, int popSize, int generations,
                                   double mutationRate, double eliteFrac, int tournamentK, bool verbose) {
    auto mapData = readMapFile(filename);
    
    cout << "Map:" << endl;
    for (const auto& row : mapData.lines) {
        cout << row << endl;
    }
    cout << "Spawns: ";
    for (auto [r, c] : mapData.spawns) {
        cout << "(" << r << "," << c << ") ";
    }
    cout << endl;
    cout << "Target: (" << mapData.target.first << "," << mapData.target.second << ")" << endl;
    cout << "Obstacles: " << mapData.obstacles.size() << endl;
    cout << "Unbuildables: " << mapData.unbuildables.size() << endl;
    
    auto [bestChrom, bestScore] = runGA(mapData.rows, mapData.cols, mapData.spawns, mapData.target,
                                       mapData.obstacles, mapData.unbuildables,
                                       popSize, generations, 0.20, mutationRate,
                                       eliteFrac, tournamentK, -1, verbose);
    
    cout << "\nBest solution found:" << endl;
    printGrid(bestChrom, mapData.rows, mapData.cols, mapData.spawns, mapData.target,
             mapData.obstacles, mapData.unbuildables);
    cout << "Best score (max distance from spawn to target): " << bestScore << endl;
    
    return static_cast<int>(bestScore);
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        cout << "Usage: " << argv[0] << " map.txt" << endl;
        return 1;
    }
    
    try {
        string filename = argv[1];
        GeneticAlgorithm::runGAFromFile(filename, 200, 1000, 0.01, 0.5, 5, true);
    } catch (const exception& e) {
        cerr << "Error: " << e.what() << endl;
        return 1;
    }
    
    return 0;
}
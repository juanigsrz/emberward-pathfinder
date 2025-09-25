#ifndef GENETIC_ALGORITHM_HPP
#define GENETIC_ALGORITHM_HPP

#include <vector>
#include <utility>
#include <set>
#include <string>
#include <optional>

using namespace std;

struct MapData {
    vector<string> lines;
    vector<pair<int, int>> spawns;
    pair<int, int> target;
    set<pair<int, int>> obstacles;
    set<pair<int, int>> unbuildables;
    int rows, cols;
};

class GeneticAlgorithm {
public:
    // Map reading
    static MapData readMapFile(const string& filename);
    
    // Pathfinding utilities
    static optional<int> astarShortestPathLength(const vector<vector<int>>& walls, 
                                                pair<int, int> src, pair<int, int> dst);
    static vector<pair<int, int>> bfsShortestPathOnEmpty(int nrows, int ncols, 
                                                        pair<int, int> src, pair<int, int> dst);
    
    // Chromosome operations
    static vector<vector<int>> chromosomeToGrid(const vector<int>& chrom, int nrows, int ncols);
    static vector<int> applyMasks(vector<int> chrom, int nrows, int ncols,
                                 const vector<pair<int, int>>& spawns, 
                                 const pair<int, int>& target,
                                 const set<pair<int, int>>& obstacles,
                                 const set<pair<int, int>>& unbuildables);
    static vector<int> createRandomChrom(int nrows, int ncols, double wallProb,
                                        const vector<pair<int, int>>& spawns,
                                        const pair<int, int>& target,
                                        const set<pair<int, int>>& obstacles,
                                        const set<pair<int, int>>& unbuildables);
    static vector<int> repairChromosome(const vector<int>& chrom, int nrows, int ncols,
                                       const vector<pair<int, int>>& spawns,
                                       const pair<int, int>& target,
                                       const set<pair<int, int>>& obstacles,
                                       const set<pair<int, int>>& unbuildables);
    static double fitness(const vector<int>& chrom, int nrows, int ncols,
                         const vector<pair<int, int>>& spawns, 
                         const pair<int, int>& target);
    
    // Genetic operators
    static vector<int> tournamentSelection(const vector<vector<int>>& pop, 
                                          const vector<double>& popFitness, int k = 3);
    static pair<vector<int>, vector<int>> twoPointCrossover(const vector<int>& a, const vector<int>& b);
    static void mutate(vector<int>& chrom, double mutationRate);
    
    // Main GA
    static pair<vector<int>, double> runGA(int nrows, int ncols,
                                          const vector<pair<int, int>>& spawns,
                                          const pair<int, int>& target,
                                          const set<pair<int, int>>& obstacles = {},
                                          const set<pair<int, int>>& unbuildables = {},
                                          int popSize = 80, int generations = 200,
                                          double wallProb = 0.20, double mutationRate = 0.01,
                                          double eliteFrac = 0.05, int tournamentK = 3,
                                          int seed = -1, bool verbose = true);
    
    // Utility
    static void printGrid(const vector<int>& chrom, int nrows, int ncols,
                         const vector<pair<int, int>>& spawns,
                         const pair<int, int>& target,
                         const set<pair<int, int>>& obstacles,
                         const set<pair<int, int>>& unbuildables);
    
    static int runGAFromFile(const string& filename, int popSize = 200, int generations = 1000,
                            double mutationRate = 0.01, double eliteFrac = 0.5, 
                            int tournamentK = 5, bool verbose = true);
};

#endif // GENETIC_ALGORITHM_HPP
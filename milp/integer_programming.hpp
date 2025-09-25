#ifndef INTEGER_PROGRAMMING_HPP
#define INTEGER_PROGRAMMING_HPP

#include <vector>
#include <utility>
#include <map>
#include <set>
#include <string>
#include "gurobi_c++.h"

using namespace std;

struct MapData {
    vector<string> lines;
    vector<pair<int, int>> spawns;
    pair<int, int> target;
    set<pair<int, int>> obstacles;
    set<pair<int, int>> unbuildables;
    int rows, cols;
};

class IntegerProgrammingSolver {
public:
    static MapData readMapFile(const string& filename);
    static vector<pair<int, int>> makeGridNodes(int rows, int cols);
    static vector<pair<int, int>> getNeighbors(const pair<int, int>& node, int rows, int cols);
    static int buildAndSolveFromFile(const string& filename, int timeLimit = 120);
    static int buildAndSolve(const MapData& mapData, int timeLimit = 120);

private:
    static void printSolutionGrid(const vector<string>& originalGrid,
                                const map<pair<int, int>, int>& y_val,
                                const pair<int, int>& spawn,
                                const pair<int, int>& target);
    static void printDistanceMap(const map<pair<int, int>, int>& d_val, int rows, int cols);
};

#endif // INTEGER_PROGRAMMING_HPP
#ifndef SOLVER_HPP
#define SOLVER_HPP

#include <vector>
#include <string>
#include <set>
#include <utility>

using namespace std;

struct MapData {
    vector<string> lines;
    vector<pair<int, int>> spawns;
    pair<int, int> target;
    set<pair<int, int>> obstacles;
    set<pair<int, int>> unbuildables;
};

struct Piece {
    char symbol;
    vector<vector<pair<int, int>>> orientations;
};

struct PlacedPiece {
    vector<pair<int, int>> coords;
    char symbol;
};

class AnnealingSolver {
public:
    static MapData readMapFile(const string& filename);
    static vector<vector<int>> computeDistances(const vector<string>& grid, const pair<int, int>& target);
    static int evaluate(const vector<string>& grid, const vector<pair<int, int>>& spawns, const pair<int, int>& target);
    
    static vector<pair<int, int>> rotateOffsets(const vector<pair<int, int>>& offsets);
    static vector<vector<pair<int, int>>> allRotations(const vector<pair<int, int>>& baseOffsets);
    static vector<Piece> initializePieces();
    
    static vector<pair<int, int>> canPlace(const vector<string>& grid, int r, int c, 
                                          const vector<pair<int, int>>& shape,
                                          const set<pair<int, int>>& obstacles,
                                          const set<pair<int, int>>& unbuildables);
    static void placePiece(vector<string>& grid, const vector<pair<int, int>>& coords, char symbol);
    static void removePiece(vector<string>& grid, const vector<pair<int, int>>& coords);
    
    static pair<vector<string>, int> simulatedAnnealing(const vector<string>& lines,
                                                       const vector<pair<int, int>>& spawns,
                                                       const pair<int, int>& target,
                                                       const set<pair<int, int>>& obstacles,
                                                       const set<pair<int, int>>& unbuildables,
                                                       int maxIter = 200000,
                                                       double T0 = 50.0,
                                                       double alpha = 0.9995);
};

#endif // SOLVER_HPP
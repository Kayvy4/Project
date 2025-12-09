#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <queue>
#include <algorithm>
#include <chrono>

//hpp files
///
#include "algo_dijkstra.hpp"
#include "algo_astar.hpp"



using namespace std;

//-----------------------------------------------------------------------------------------------------------
//Setep 1 grid and map ldr
//TODO"2d grid in 1d array
//
//map gen
//
struct Grid {
    int width = 0;
    int height = 0;
    int startRow = 0, startCol = 0;
    int goalRow = 0, goalCol = 0;
    vector<char> cells; // 1D array: row * width + col

    bool InBounds(int r, int c) const {
        return r >= 0 && r < height && c >= 0 && c < width;
    }

    bool IsBlocked(int r, int c) const {
        return cells[r * width + c] == '#';
    }

    int ToId(int r, int c) const {
        return r * width + c;
    }

    pair<int,int> FromId(int id) const {
        return {id / width, id % width};
    }

    vector<int> GetNeighbors(int r, int c) const {
        vector<int> neighbors;
        int dr[4] = {-1,1,0,0};
        int dc[4] = {0,0,-1,1};
        for(int i=0;i<4;i++){
            int nr = r + dr[i];
            int nc = c + dc[i];
            if(InBounds(nr,nc) && !IsBlocked(nr,nc))
                neighbors.push_back(ToId(nr,nc));
        }
        return neighbors;
    }

    void Print() const {
        for(int r=0;r<height;r++){
            for(int c=0;c<width;c++){
                if(r==startRow && c==startCol) cout << 'S';
                else if(r==goalRow && c==goalCol) cout << 'G';
                else cout << cells[r*width + c];
            }
            cout << "\n";
        }
    }
};

bool LoadMap(const string &filename, Grid &grid){
    ifstream file(filename);
    if (!file) {
        cout << "Could not open map file: " << filename << "\n";
        return false;
    }

    string line;
    while(getline(file, line)){
        if(line.empty() || line[0]=='#') continue; // skip comments
        istringstream iss(line);
        string key;
        if(iss >> key){
            if(key=="WIDTH") iss >> grid.width;
            else if(key=="HEIGHT") iss >> grid.height;
            else if(key=="START") iss >> grid.startRow >> grid.startCol;
            else if(key=="GOAL") iss >> grid.goalRow >> grid.goalCol;
            else break; // reached grid body
        }
    }

    grid.cells.reserve(grid.width * grid.height);
    do { // read grid lines
        if(line.empty() || line[0]=='#') continue;
        for(char c : line) if(c=='.' || c=='#') grid.cells.push_back(c);
    } while(getline(file, line));

    // simple validation
    if(grid.cells.size() != grid.width * grid.height) return false;
    if(grid.IsBlocked(grid.startRow, grid.startCol)) return false;
    if(grid.IsBlocked(grid.goalRow, grid.goalCol)) return false;

    return true;
}



//------------------------------------------------------------------------------------------------------
//bff stp3
//
//make the core of the project
//queue

struct BFSResult {
    bool found = false;
    int pathLen = 0;
    int visitedCount = 0;
    long long timeUS = 0;
    vector<int> path;
    vector<bool> visited; //New, stores visited
};
//revised
BFSResult BFS(const Grid &grid) {
    using namespace std::chrono;

    auto t0 = high_resolution_clock::now();

    int start = grid.ToId(grid.startRow, grid.startCol);
    int goal  = grid.ToId(grid.goalRow, grid.goalCol);

    vector<bool> visited(grid.width * grid.height, false);
    vector<int> parent(grid.width * grid.height, -1);

    queue<int> q;
    q.push(start);
    visited[start] = true;

    int visitedCount = 0;
    bool found = false;

    while(!q.empty()){
        int current = q.front(); 
        q.pop();
        visitedCount++;

        if(current == goal){
            found = true;
            break;
        }

        auto [r, c] = grid.FromId(current);
        for(int neighbor : grid.GetNeighbors(r, c)){
            if(!visited[neighbor]){
                visited[neighbor] = true;
                parent[neighbor] = current;
                q.push(neighbor);
            }
        }
    }

    // Reconstruct path
    vector<int> path;
    if(found){
        int node = goal;
        while(node != -1){
            path.push_back(node);
            node = parent[node];
        }
        reverse(path.begin(), path.end());
    }

    auto t1 = high_resolution_clock::now();
    long long us = duration_cast<microseconds>(t1 - t0).count();

    BFSResult result;
    result.found = found;
    result.pathLen = (int)path.size();
    result.visitedCount = visitedCount;
    result.timeUS = us;
    result.path = path;
    result.visited = visited;
    return result;
}

//Original
/*
BFSResult BFS(const Grid &grid) {
    int start = grid.ToId(grid.startRow, grid.startCol);
    int goal  = grid.ToId(grid.goalRow, grid.goalCol);

    vector<bool> visited(grid.width * grid.height, false);
    vector<int> parent(grid.width * grid.height, -1);

    queue<int> q;
    q.push(start);
    visited[start] = true;

    int visitedCount = 0;
    bool found = false;

    while(!q.empty()){
        int current = q.front(); q.pop();
        visitedCount++;

        if(current == goal){
            found = true;
            break;
        }

        auto [r, c] = grid.FromId(current);
        for(int neighbor : grid.GetNeighbors(r,c)){
            if(!visited[neighbor]){
                visited[neighbor] = true;
                parent[neighbor] = current;
                q.push(neighbor);
            }
        }
    }
//recon
    vector<int> path;
    if(found){
        int node = goal;
        while(node != -1){
            path.push_back(node);
            node = parent[node];
        }
        reverse(path.begin(), path.end());
    }

    return BFSResult{found, (int)path.size(), visitedCount, path};
}
*/

//------------------------------------------------------------------------------------------------------------------
//
//
void OverlayAndPrint(const Grid &grid, const BFSResult &result, const string &algoName){
    vector<char> display = grid.cells;

    // mark visited correctly
    for(int id = 0; id < (int)display.size(); id++){
        if(result.visited[id] &&
           id != grid.ToId(grid.startRow, grid.startCol) &&
           id != grid.ToId(grid.goalRow, grid.goalCol) &&
           display[id] == '.')
        {
            display[id] = '+';
        }
    }

    // mark final path
    for(int id : result.path){
        if(id != grid.ToId(grid.startRow, grid.startCol) &&
           id != grid.ToId(grid.goalRow, grid.goalCol) &&
           display[id] != '#')
        {
            display[id] = '*';
        }
    }

    cout << "=== " << algoName << " ===\n";
    for(int r = 0; r < grid.height; r++){
        for(int c = 0; c < grid.width; c++){
            int id = r * grid.width + c;
            if(r == grid.startRow && c == grid.startCol) cout << 'S';
            else if(r == grid.goalRow && c == grid.goalCol) cout << 'G';
            else cout << display[id];
        }
        cout << "\n";
    }

    cout << "Found: " << (result.found ? "yes" : "no")
         << " | Visited: " << result.visitedCount
         << " | PathLen: " << result.pathLen
         << " | Cost: " << result.pathLen
         << " | Time(us): " << result.timeUS
         << "\n\n";
}


    //=------------------------------------------------------------------------------------------
    //Main here
    //TODO: Arguments argv(c)
    //deal with flags

int main(int argc, char** argv){
//ORIGINAL - USE FOR TESTING
//Basic errer handling
/*
    	if(argc < 2){
        cout << "Usage: " << argv[0] << " <map_file>\n";
        return 1;
    }

    Grid grid;
    if(!LoadMap(argv[1], grid)){
        cout << "Failed to load map or invalid map.\n";
        return 1;
    }

    cout << "Map loaded successfully:\n";
    grid.Print();
*/
 string mapFile;
    string algo = "bfs"; // default

    for(int i = 1; i < argc; i++){
        string arg = argv[i];
        if(arg == "--map" && i + 1 < argc) mapFile = argv[++i];
        else if(arg == "--algo" && i + 1 < argc) algo = argv[++i];
        else if(arg == "--help") {
            cout << "Usage: app --map <file> --algo {bfs|dijkstra|astar|all}\n";
            return 0;
        } else {
            cout << "Unknown argument: " << arg << "\n";
            return 1;
        }
    }

    if(mapFile.empty()){
        cout << "Error: No map file provided.\n";
        return 1;
    }

    Grid grid;
    if(!LoadMap(mapFile, grid)){
        cout << "Failed to load map.\n";
        return 2;
    }

    cout << "=== Loaded Grid ===\n";
    grid.Print();

//    cout << "\nSelected algorithm: " << algo << "\n";
    // BFS/Dijkstra/A* will go here

//Hope you dont need notes on what this does

 if(algo=="bfs" || algo=="all"){
        BFSResult bfsResult = BFS(grid);
        OverlayAndPrint(grid, bfsResult, "BFS");
    }
if(algo=="dijkstra" || algo=="all"){
    RunResult dijResult = RunDijkstra(grid);
    OverlayAndPrint(grid, dijResult, "Dijkstra");
}

return 0;
}










#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <queue>
#include <algorithm>
#include <chrono>

// hpp files
#include "algo_dijkstra.hpp"
#include "algo_astar.hpp"
#include "algo_bfs.hpp"
#include "render.hpp"
#include "algo_common.hpp"

using namespace std;

//----------------------------------------------------------------------------------------------------------
// Main here
// TODO: Arguments argv(c)
// deal with flags

int main(int argc, char** argv){
    // ORIGINAL - USE FOR TESTING
    // Basic error handling
    /*
    if(argc < 2){
        cout << "Usage: " << argv[0] << " <map_file>\n";
        return 1;
    }
    */

    string mapFile;
    string algo = "bfs"; // default

    // Parse command-line arguments
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

    // Load grid from file
    Grid grid;
    if(!grid.LoadFromFile(mapFile)){
        cout << "Failed to load map.\n";
        return 2;
    }

    cout << "=== Loaded Grid ===\n";
    //grid.print();
    OverlayAndPrint(grid, RunResult{}, "Loaded Grid");

    //----------------------------------------------------------------------------------------
    // Run algorithms

    if(algo=="bfs" || algo=="all"){
        RunResult bfsResult = RunBFS(grid);
        OverlayAndPrint(grid, bfsResult, "BFS");
    }

    if(algo=="dijkstra" || algo=="all"){
        RunResult dijResult = RunDijkstra(grid);
        OverlayAndPrint(grid, dijResult, "Dijkstra");
    }

    if(algo=="astar" || algo=="all"){
        RunResult astarResult = RunAStar(grid);
        OverlayAndPrint(grid, astarResult, "A*");
    }

    return 0;
}

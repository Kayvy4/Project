// algo_bfs.cpp
#include "algo_bfs.hpp"
#include <queue>
#include <algorithm>
#include <chrono>
//------------------------------------------------------------------------------------------------------
//bff stp3
//
//make the core of the project
//queue

//Original, had in main originally. Copied and edited to fit assignment deets
/*
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
///*
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
//*/
*/
RunResult RunBFS(const Grid &grid) {
    auto start_time = std::chrono::high_resolution_clock::now();

    int start = grid.StartId();
    int goal = grid.GoalId();

    int H = grid.Height();
    int W = grid.Width();
    std::vector<bool> visited(H*W, false);
    std::vector<int> parent(H*W, -1);

    std::queue<int> q;
    q.push(start);
    visited[start] = true;

    int explored_count = 0;
    bool found = false;

    int neighbors[4];
    while(!q.empty()){
        int current = q.front(); q.pop();
        explored_count++;

        if(current == goal){
            found = true;
            break;
        }

        int n_count = grid.GetNeighbors(current, neighbors);
        for(int i=0;i<n_count;i++){
            int nb = neighbors[i];
            if(!visited[nb]){
                visited[nb] = true;
                parent[nb] = current;
                q.push(nb);
            }
        }
    }

    // reconstruct path
    std::vector<int> path_ids;
    if(found){
        int node = goal;
        while(node != -1){
            path_ids.push_back(node);
            node = parent[node];
        }
        std::reverse(path_ids.begin(), path_ids.end());
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    long long micros = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    RunResult rr;
    rr.found = found;
    rr.path_ids = path_ids;
    rr.visited_mask.resize(H*W);
    for(int i=0;i<H*W;i++) rr.visited_mask[i] = visited[i] ? 1 : 0;
    rr.explored_count = explored_count;
    rr.path_cost = path_ids.size();
    rr.micros = micros;

    return rr;
}


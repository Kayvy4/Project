// src/algo_dijkstra.cpp
#include "algo_dijkstra.hpp"
#include <queue>
#include <vector>
#include <chrono>
#include <algorithm>

//TODO: implement RunDijksra
//
//
RunResult RunDijkstra(const Grid& grid) {
    auto start_time = std::chrono::high_resolution_clock::now();

    int N = grid.Height() * grid.Width();
    std::vector<int> dist(N, 1e9);      // distances (large number = infinity)
    std::vector<int> parent(N, -1);     // for path reconstruction
    std::vector<char> visited_mask(N, 0);

    int start = grid.StartId();
    int goal  = grid.GoalId();
    dist[start] = 0;

    using PQNode = std::pair<int,int>; // {distance, node_id}
    std::priority_queue<PQNode, std::vector<PQNode>, std::greater<>> pq;
    pq.push({0, start});

    int explored_count = 0;
    bool found = false;

    while(!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();

        if(visited_mask[u]) continue;
        visited_mask[u] = 1;
        explored_count++;

        if(u == goal) {
            found = true;
            break;
        }

        int neighbors[4];
        int ncount = grid.GetNeighbors(u, neighbors);
        for(int i=0;i<ncount;i++){
            int v = neighbors[i];
            if(grid.IsBlocked(grid.FromId(v).row, grid.FromId(v).col)) continue;
            int new_dist = d + 1; // unit cost
            if(new_dist < dist[v]) {
                dist[v] = new_dist;
                parent[v] = u;
                pq.push({dist[v], v});
            }
        }
    }

    // Reconstruct path
    std::vector<int> path;
    if(found) {
        int node = goal;
        while(node != -1) {
            path.push_back(node);
            node = parent[node];
        }
        std::reverse(path.begin(), path.end());
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    long long micros = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    RunResult rr;
    rr.found = found;
    rr.path_ids = path;
    rr.visited_mask = visited_mask;
    rr.explored_count = explored_count;
    rr.path_cost = path.size() ? (int)path.size()-1 : 0;
    rr.micros = micros;

    return rr;
}

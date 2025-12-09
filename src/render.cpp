//------------------------------------------------------------------------------------------------------------------
//
//
// render.cpp
#include "render.hpp"
#include <iostream>

void OverlayAndPrint(const Grid &grid, const RunResult &rr, const std::string &algo_name){
    int H = grid.Height();
    int W = grid.Width();
    std::vector<char> display(H*W);
    for(int r=0;r<H;r++)
        for(int c=0;c<W;c++)
            display[r*W+c] = grid.At(r,c);

    // mark visited
    for(int id=0;id<H*W;id++)
        if(rr.visited_mask[id] && display[id]=='.') display[id] = '+';

    // mark path
    for(int id : rr.path_ids)
        if(display[id]=='.' || display[id]=='+') display[id] = '*';

    auto start = grid.Start();
    auto goal  = grid.Goal();
    display[start.row*W+start.col] = 'S';
    display[goal.row*W+goal.col] = 'G';

    std::cout << "=== " << algo_name << " ===\n";
    for(int r=0;r<H;r++){
        for(int c=0;c<W;c++)
            std::cout << display[r*W + c];
        std::cout << "\n";
    }

    std::cout << "Found: " << (rr.found?"yes":"no")
              << " | Visited: " << rr.explored_count
              << " | PathLen: " << rr.path_cost
              << " | Cost: " << rr.path_cost
              << " | Time(us): " << rr.micros
              << "\n\n";
}


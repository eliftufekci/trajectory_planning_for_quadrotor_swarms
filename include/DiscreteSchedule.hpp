#pragma once
#include <vector>
class Graph;
struct SubdividedSchedule;

struct DiscreteSchedule{
    std::vector<std::vector<int>> waypoint; // robot -> path of vertex ids
    int K; //makespan

    DiscreteSchedule( const std::vector<std::vector<int>>& waypoint, int K);


    SubdividedSchedule subdivide(const Graph& graph);
    
};

#pragma once
#include <vector>

struct DiscreteSchedule{
    std::vector<std::vector<int>> waypoint; // (robot → path of vertex ids)
    int K; //makespan

    DiscreteSchedule( const std::vector<std::vector<int>>& waypoint, int K)
        : waypoint(waypoint), K(K) {}
};
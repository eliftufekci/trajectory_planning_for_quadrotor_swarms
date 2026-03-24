#pragma once
#include <vector>

struct DiscreteSchedule{
    std::vector<std::vector<int>> waypoint; // (robot → path of vertex ids)
    double K; //makespan

    DiscreteSchedule( const std::vector<std::vector<int>>& waypoint, double K)
        : waypoint(waypoint), K(K) {}
};
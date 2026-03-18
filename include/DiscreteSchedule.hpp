#pragma once
#include <vector>

struct DiscreteSchedule{
    std::vector<std::vector<std::vector<int>>> waypoint; // (robot → timestep → vertex id)
    double K; //makespan

    DiscreteSchedule( std::vector<std::vector<std::vector<int>>> waypoint
                    , double K)
        : waypoint(waypoint)
        , K(K) {}
};
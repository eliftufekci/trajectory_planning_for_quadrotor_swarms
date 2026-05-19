#pragma once
#include <cstddef>
#include <vector>
#include "Graph.hpp"
#include <Eigen/Dense>

struct SubdividedSchedule{
    std::vector<std::vector<Eigen::Vector3d>> positions; // robot -> position list
    int K; // makespan*2 from the discrete schedule

    SubdividedSchedule(int num_robots, int makespan);
};



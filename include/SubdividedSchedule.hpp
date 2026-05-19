#pragma once
#include <cstddef>
#include <vector>
#include "Graph.hpp"
#include <Eigen/Dense>

struct SubdividedSchedule{
    std::vector<std::vector<Eigen::Vector3d>> positions; // robot -> position list
    int K; // discrete schedule'dan gelen makespan*2

    SubdividedSchedule(int num_robots, int makespan);
};




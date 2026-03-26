#pragma once
#include <Eigen/Dense>

struct HyperPlane{
    Eigen::Vector3d normal_vector;  // normalized
    double d;                       // offset: n^T x = d
    size_t agent_i;
    size_t agent_j;
    int timestep;

    HyperPlane( Eigen::Vector3d normal_vector, 
                double d,
                size_t agent_i,
                size_t agent_j,
                int timestep )

        : normal_vector(normal_vector), d(d), agent_i(agent_i), agent_j(agent_j), timestep(timestep) {}

};

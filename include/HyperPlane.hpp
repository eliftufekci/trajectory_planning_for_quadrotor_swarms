#pragma once
#include <Eigen/Dense>

struct HyperPlane{
    Eigen::Vector3d normal_vector;  // normalized
    double d;                       // offset: n^T x = d

    HyperPlane(Eigen::Vector3d normal_vector, double d)
        : normal_vector(normal_vector), d(d) {}

};

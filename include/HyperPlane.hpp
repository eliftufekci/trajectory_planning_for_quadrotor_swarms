#pragma once
#include <Eigen/Dense>

struct HyperPlane{
    Eigen::Vector3d normal_vector;  // normalized
    double d;                       // offset: n^T x = d
    double ellipsoid_offset;        // = ||E * normal_vector||2

    HyperPlane(Eigen::Vector3d normal_vector, double d, double ellipsoid_offset)
        : normal_vector(normal_vector), d(d), ellipsoid_offset(ellipsoid_offset) {}

};

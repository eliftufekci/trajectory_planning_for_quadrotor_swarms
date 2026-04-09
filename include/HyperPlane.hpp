#pragma once
#include <Eigen/Dense>

struct HyperPlane{
    Eigen::Vector3d normal_vector;  // normalized
    double d;                       // offset: n^T x = d
    double ellipsoid_offset;        // = ||E * normal_vector||2

    int separated_from_type;        // 0: Robot, 1: Obstacle
    int separated_from_id;          // ID of the other robot or obstacle

    HyperPlane(Eigen::Vector3d normal_vector, double d, double ellipsoid_offset, int type = -1, int from_id = -1)
        : normal_vector(normal_vector), d(d), ellipsoid_offset(ellipsoid_offset)
        , separated_from_type(type), separated_from_id(from_id) {}

};

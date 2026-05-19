#pragma once
#include <Eigen/Dense>


// Paper Eq.(3): RR(q) = { Ex + q : ||x||2 <= 1 }
// E = diag(rx, ry, rz)
// For quadrotors: rx = ry << rz (downwash model)
struct RobotModel {
    // Robot volume for environment collisions (against surrounding obstacles).
    double radius;       // single radius (sphere assumption, paper Sec. IV-A)

    // Ellipsoid radii for robot-robot collisions (downwash).
    double rx, ry, rz;  // rx == ry << rz

    // Default: similar to a Crazyflie nano-quadrotor.
    RobotModel();
    RobotModel(double radius, double rx, double ry, double rz);

    // Do two robots collide? (ellipsoid-ellipsoid)
    // Simple method: there is a collision if the normalized distance is < 1.
    bool collides(const Eigen::Vector3d& qi, const Eigen::Vector3d& qj) const;
};

#pragma once
#include <Eigen/Dense>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/continuous_collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>
#include <vector>
#include <memory>
#include "RobotModel.hpp"
#include "Environment.hpp"


class FCLCollisionChecker {
public:
    const Environment& env;
    RobotModel robot;

    std::vector<std::shared_ptr<fcl::Boxd>> obsBoxObjects;
    std::vector<std::shared_ptr<fcl::CollisionObjectd>> obsCollisionObjects;

    // Sphere geometry is created once and shared by each isEdgeFree call.
    std::shared_ptr<fcl::Sphered> agent_sphere_;

    FCLCollisionChecker(const Environment& env, const RobotModel& robot);

    void createObsObjects(const std::vector<struct AABB>& obstacles);

    bool isEdgeFree(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const;

    bool isOccupied(const Eigen::Vector3d& pos) const;
};

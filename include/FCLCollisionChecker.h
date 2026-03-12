#pragma once
#include "Environment.h"
#include "RobotModel.h"
#include <Eigen/Dense>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/continuous_collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>
#include <vector>
#include <memory>

class FCLCollisionChecker {
public:
    const Environment& env;
    RobotModel robot;

    std::vector<std::shared_ptr<fcl::Boxd>> obsBoxObjects;
    std::vector<std::shared_ptr<fcl::CollisionObjectd>> obsCollisionObjects;

    // Sphere geometry bir kez oluşturulur, her isEdgeFree çağrısında paylaşılır
    std::shared_ptr<fcl::Sphered> agent_sphere_;

    FCLCollisionChecker(const Environment& env, const RobotModel& robot)
        : env(env), robot(robot)
    {
        agent_sphere_ = std::make_shared<fcl::Sphered>(robot.radius);
        createObsObjects(env.obstacles);
    }

    void createObsObjects(const std::vector<AABB>& obstacles) {
        for (const auto& obstacle : obstacles) {
            Eigen::Vector3d size = obstacle.max - obstacle.min;
            Eigen::Vector3d center = (obstacle.min + obstacle.max) * 0.5;

            auto box = std::make_shared<fcl::Boxd>(size.x(), size.y(), size.z());

            fcl::Transform3d tf = fcl::Transform3d::Identity();
            tf.translation() = center;

            auto obj = std::make_shared<fcl::CollisionObjectd>(box, tf);

            obsBoxObjects.push_back(box);
            obsCollisionObjects.push_back(obj);
        }
    }

    bool isEdgeFree(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const {
        // Robot başlangıç transform'u
        fcl::Transform3d tf_start = fcl::Transform3d::Identity();
        tf_start.translation() = a;

        // Robot bitiş transform'u
        fcl::Transform3d tf_end = fcl::Transform3d::Identity();
        tf_end.translation() = b;

        // Robot CollisionObject — başlangıç pozisyonunda
        fcl::CollisionObjectd sphere_obj(agent_sphere_, tf_start);

        // Obstacle bitiş transform'u — sabit, kendi tf'i ile aynı
        for (const auto& obs_obj : obsCollisionObjects) {
            fcl::ContinuousCollisionRequestd request;
            fcl::ContinuousCollisionResultd result;

            fcl::continuousCollide(
                &sphere_obj, tf_end,                    // robot: start→end hareket ediyor
                obs_obj.get(), obs_obj->getTransform(), // obstacle: sabit, başlangıç = bitiş
                request, result
            );

            if (result.is_collide) {
                return false;
            }
        }

        return true;
    }

    bool isOccupied(const Eigen::Vector3d& pos) const {
        // Robot başlangıç transform'u
        fcl::Transform3d tf = fcl::Transform3d::Identity();
        tf.translation() = pos;

        // Robot CollisionObject — başlangıç pozisyonunda
        fcl::CollisionObjectd sphere_obj(agent_sphere_, tf);

        for (const auto& obs_obj : obsCollisionObjects) {
            fcl::CollisionRequestd request;
            fcl::CollisionResultd result;

            fcl::collide(&sphere_obj, obs_obj.get(), request, result);

            if (result.isCollision()) {
                return true;
            }
        }

        return false;
    }
};
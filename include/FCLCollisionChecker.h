#pragma once
#include "Environment.h"
#include "RobotModel.h"
#include <Eigen/Dense>
#include <Eigen/Dense>
#include <fcl/fcl.h>
#include <vector>

class FCLCollisionChecker{
public:

    std::vector<std::shared_ptr<fcl::CollisionObjectd>> obsCollisionObjects;
    std::vector<std::shared_ptr<fcl::Boxd>> obsBoxObjects;

    std::vector<std::shared_ptr<fcl::CollisionObjectd>> agentCollisionObjects;
    std::vector<std::shared_ptr<fcl::Sphered>> agentSphereObjects;
    
    const Environment& env;
    RobotModel robot;

    FCLCollisionChecker(const Environment& env, RobotModel robot){
        createObsObjects(env.obstacles);
        createAgentObjects(env.agents);

        env(env);
        robot(robot);
    }

    void createObsObjects(const std::vector<AABB> obstacles){
        for(const auto& obstacle : obstacles){
            const auto& min = obstacle.min;
            const auto& max = obstacle.max;

            // full side lengths
            double fsl_x = (max - min).x();
            double fsl_y = (max - min).y();
            double fsl_z = (max - min).z();

            auto center = (min + max) * 0.5;

            fcl::Boxd obs_box(fsl_x, fsl_y, fsl_z);
            
            fcl::Transform3d tf_obs = fcl::Transform3d::Identity();
            tf_obs.translation() << center;

            fcl::CollisionObjectd obs_collision_obj(obs_box, tf_obs);

            obsBoxObjects.push_back(obs_box);
            obsCollisionObjects.push_back(obs_collision_obj);

        }
    }

    void createAgentObjects(const std::vector<Agent> agents){
        for(const auto& agent : agents){
            const auto& start = agent.start;
            const auto& robot_radius = robot.radius;

            fcl::Sphered agent_sphere(robot_radius);
            
            fcl::Transform3d tf_agent = fcl::Transform3d::Identity();
            tf_agent.translation() << start;

            fcl::CollisionObjectd agent_collision_obj(agent_sphere, tf_agent);

            agentSphereObjects.push_back(agent_sphere);
            agentCollisionObjects.push_back(agent_collision_obj);
        }
    }

    // bool checkCollision(const fcl::CollisionObjectd obj1, const fcl::CollisionObjectd obj2){
    //     fcl::CollisionRequestd request;
    //     fcl::CollisionResultd result;

    //     return fcl::collide(&obj1, &obj2, request, result);
    // }
};
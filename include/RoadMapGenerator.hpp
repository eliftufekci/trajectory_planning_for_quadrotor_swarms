#pragma once
#include "Environment.hpp"
#include "FCLCollisionChecker.hpp"
#include "Graph.hpp"
#include "RobotModel.hpp"

class RoadMapGenerator{
public:
    virtual ~RoadMapGenerator() = default;

    virtual Graph createRoadMap() = 0;

protected:
    RoadMapGenerator(const Environment& env,
                     const FCLCollisionChecker& collisionChecker,
                     const RobotModel& robotModel);
    
    const RobotModel& robotModel;
    const Environment& env;
    const FCLCollisionChecker& collisionChecker;
};

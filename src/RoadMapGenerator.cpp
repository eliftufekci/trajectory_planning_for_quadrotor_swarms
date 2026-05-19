#include "RoadMapGenerator.hpp"
#include "Environment.hpp"
#include "FCLCollisionChecker.hpp"
#include "Graph.hpp"

RoadMapGenerator::RoadMapGenerator(const Environment& env,
                    const FCLCollisionChecker& collisionChecker,
                    const RobotModel& robotModel)
    : env(env)
    , collisionChecker(collisionChecker)
    , robotModel(robotModel) {}


#pragma once
#include "Environment.hpp"
#include "FCLCollisionChecker.hpp"
#include "Graph.hpp"

class RoadMapGenerator{
public:
    virtual ~RoadMapGenerator() = default;

    virtual Graph createRoadMap() = 0;

    std::tuple<std::vector<int>, std::vector<int>> start_goal_vertices;

protected:
    RoadMapGenerator(const Environment& env,
                     const FCLCollisionChecker& collisionChecker)
        : env(env)
        , collisionChecker(collisionChecker){}

    const Environment& env;
    const FCLCollisionChecker& collisionChecker;
};

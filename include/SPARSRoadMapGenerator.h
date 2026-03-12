#pragma once
#include <map>
#include <tuple>
#include <vector>
#include <limits>
#include <Eigen/Dense>
#include "Environment.h"
#include "FCLCollisionChecker.h"
#include "Graph.h"

class SPARSRoadMapGenerator{
public:
    const Environment& env;
    const FCLCollisionChecker& collisionChecker;

    SPARSRoadMapGenerator(const Environment& env, const FCLCollisionChecker& collisionChecker)
        : env(env), collisionChecker(collisionChecker) {}

    Graph createRoadMap() {
    }


};

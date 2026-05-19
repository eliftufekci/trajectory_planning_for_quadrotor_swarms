#pragma once
#include <map>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include "Environment.hpp"
#include "FCLCollisionChecker.hpp"
#include "Graph.hpp"
#include "RoadMapGenerator.hpp"
#include "RobotModel.hpp"
#include <iosfwd>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class SPARSRoadMapGenerator : public RoadMapGenerator{
public:
    double sparseDeltaFraction;

    // sparseDeltaFraction: SPARS delta parameter (as a ratio of the space diagonal).
    //                      OMPL uses it as sparseDelta = fraction * space_diagonal.
    //                      connectRadius: radius used to connect starts/goals.
    //                      If left negative, it is automatically set to sparseDelta.
    SPARSRoadMapGenerator(const Environment& env, const RobotModel& robot,
                          const FCLCollisionChecker& collisionChecker,
                          double sparseDeltaFraction = 0.05,
                          double connectRadius = -1.0);

    Graph createRoadMap() override;

    bool isStateValid(const ob::State* state);

private:
    double connectRadius_;   // start/goal connection radius

    // Compute a search radius consistent with SPARS's own delta.
    // In OMPL: sparseDelta = sparseDeltaFraction * space_diagonal.
    // If connectRadius is negative, use the same formula to keep it consistent.
    double computeSearchRadius() const;

    void addVertices(Graph& graph,
                     std::map<unsigned int, int>& vertexMap,
                     ob::PlannerData& data);
    
    
    void addEdges(Graph& graph,
                  std::map<unsigned int, int>& vertexMap,
                  ob::PlannerData& data);


    void connectAgents(Graph& graph);

    // Paper Section IV: "connect to up to six neighbors within a search radius
    //                     if the edge could be traversed without collision"
    // Returns the vertex ID on success, or -1 on failure.
    int connectPoint(const Eigen::Vector3d& point, Graph& graph);
};

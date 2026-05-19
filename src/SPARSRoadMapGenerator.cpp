#include "SPARSRoadMapGenerator.hpp"
#include <map>
#include <vector>
#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <Eigen/Dense>
#include "Environment.hpp"
#include "FCLCollisionChecker.hpp"
#include "Graph.hpp"
#include "RobotModel.hpp"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;


// sparseDeltaFraction: SPARS delta parameter (as a ratio of the space diagonal).
//                      OMPL uses it as sparseDelta = fraction * space_diagonal.
//                      connectRadius: radius used to connect starts/goals.
//                      If left negative, it is automatically set to sparseDelta.
SPARSRoadMapGenerator::SPARSRoadMapGenerator(const Environment& env, const RobotModel& robot,
                        const FCLCollisionChecker& collisionChecker,
                        double sparseDeltaFraction,
                        double connectRadius)
    : RoadMapGenerator(env, collisionChecker, robot)
    , sparseDeltaFraction(sparseDeltaFraction)
    , connectRadius_(connectRadius) {}

Graph SPARSRoadMapGenerator::createRoadMap() {
    Graph roadMap;
    std::map<unsigned int, int> vertexMap;

    // Safety margin (inflation) based on robot dimensions.
    
    double inflation = robotModel.radius;

    auto space = std::make_shared<ob::RealVectorStateSpace>(3);
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0,  env.world_min.x() + inflation); bounds.setHigh(0, env.world_max.x() - inflation);
    bounds.setLow(1,  env.world_min.y() + inflation); bounds.setHigh(1, env.world_max.y() - inflation);
    bounds.setLow(2,  env.world_min.z() + inflation); bounds.setHigh(2, env.world_max.z() - inflation);
    space->setBounds(bounds);

    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker([this](const ob::State* s) {
        return isStateValid(s);
    });
    si->setup();

    // Giving SPARS a start/goal only affects the search path;
    // roadmap construction is independent of it. However, multiple start/goal
    // states can improve coverage.
    // If there are no agents, use opposite corners of the environment; a single
    // point can cause SPARS to produce a very sparse roadmap.
    ob::ScopedState<ob::RealVectorStateSpace> startState(space);
    ob::ScopedState<ob::RealVectorStateSpace> goalState(space);

    if (!env.agents.empty() && !env.goal_positions.empty()) {
        startState[0] = env.agents[0].start.x();
        startState[1] = env.agents[0].start.y();
        startState[2] = env.agents[0].start.z();
        goalState[0]  = env.goal_positions[0].x();
        goalState[1]  = env.goal_positions[0].y();
        goalState[2]  = env.goal_positions[0].z();
    } else {
        // Opposite corners give SPARS a chance to explore the whole space.
        startState[0] = env.world_min.x();
        startState[1] = env.world_min.y();
        startState[2] = env.world_min.z();
        goalState[0]  = env.world_max.x();
        goalState[1]  = env.world_max.y();
        goalState[2]  = env.world_max.z();
    }

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->addStartState(startState);
    pdef->setGoalState(goalState);

    auto planner = std::make_shared<og::SPARS>(si);
    planner->setSparseDeltaFraction(sparseDeltaFraction);
    planner->setProblemDefinition(pdef);
    planner->setup();

    // solve() both builds the roadmap and searches for a path.
    // Even if it fails, a roadmap may have been built; getPlannerData() is enough.
    ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(10.0);
    planner->solve(ptc);

    ob::PlannerData data(si);
    planner->getPlannerData(data);

    if (data.numVertices() == 0) {
        std::cerr << "SPARS: Roadmap is empty!\n";
        return roadMap;
    }

    std::cout << "SPARS: " << data.numVertices() << " vertex, "
                << data.numEdges() << " edges created.\n";

    addVertices(roadMap, vertexMap, data);
    addEdges(roadMap, vertexMap, data);

    connectAgents(roadMap);

    return roadMap;
}

bool SPARSRoadMapGenerator::isStateValid(const ob::State* state) {
    const auto* pos = state->as<ob::RealVectorStateSpace::StateType>();
    Eigen::Vector3d point(pos->values[0], pos->values[1], pos->values[2]);
    return !collisionChecker.isOccupied(point);
}


// Compute a search radius consistent with SPARS's own delta.
// In OMPL: sparseDelta = sparseDeltaFraction * space_diagonal.
// If connectRadius is negative, use the same formula to keep it consistent.
double SPARSRoadMapGenerator::computeSearchRadius() const {
    double dx = env.world_max.x() - env.world_min.x();
    double dy = env.world_max.y() - env.world_min.y();
    double dz = env.world_max.z() - env.world_min.z();
    double sparseDelta = std::sqrt(dx*dx + dy*dy + dz*dz) * sparseDeltaFraction;

    if (connectRadius_ > 0.0)
        return connectRadius_;   // user-defined
    return sparseDelta;          // same as SPARS delta, consistent with dispersion
}

void SPARSRoadMapGenerator::addVertices(Graph& graph,
                    std::map<unsigned int, int>& vertexMap,
                    ob::PlannerData& data){
                    
    for (unsigned int i = 0; i < data.numVertices(); i++) {
        const ob::PlannerDataVertex& v = data.getVertex(i);
        if (!v.getState()) continue;   
        const auto* s = v.getState()->as<ob::RealVectorStateSpace::StateType>();
        Eigen::Vector3d pos(s->values[0], s->values[1], s->values[2]);
        vertexMap[i] = graph.addVertex(pos);
    }
}

void SPARSRoadMapGenerator::addEdges(Graph& graph,
                std::map<unsigned int, int>& vertexMap,
                ob::PlannerData& data){
                
    for (unsigned int i = 0; i < data.numVertices(); i++) {
        std::vector<unsigned int> edgeList;
        data.getEdges(i, edgeList);
        for (unsigned int nb : edgeList) {
            if (vertexMap.count(i) && vertexMap.count(nb))
                graph.addEdge(vertexMap.at(i), vertexMap.at(nb));
        }
    }
}

void SPARSRoadMapGenerator::connectAgents(Graph& graph) {
    for (const auto& agent : env.agents) {
        int start_id = connectPoint(agent.start, graph);
        if (start_id < 0) {
            throw std::runtime_error("Agent " + std::to_string(agent.id) + " start point could not be connected to the roadmap!");
        }
    }
    for (size_t i = 0; i < env.goal_positions.size(); ++i) {
        int goal_id = connectPoint(env.goal_positions[i], graph);
        if (goal_id < 0) {
            throw std::runtime_error("Goal " + std::to_string(i) + " point could not be connected to the roadmap!");
        }
    }
}

// Paper Section IV: "connect to up to six neighbors within a search radius
// Returns the vertex ID on success, or -1 on failure.
int SPARSRoadMapGenerator::connectPoint(const Eigen::Vector3d& point, Graph& graph) {
    int id = graph.addVertex(point);
    double search_radius = computeSearchRadius();

    std::vector<std::pair<double, int>> candidates;
    for (const auto& v : graph.getVertices()) {
        if (v.id == id) continue;
        double dist = (v.pos - point).norm();
        if (dist <= search_radius && collisionChecker.isEdgeFree(point, v.pos))
            candidates.push_back({dist, v.id});
    }

    std::sort(candidates.begin(), candidates.end());

    int connected = 0;
    for (const auto& [dist, vid] : candidates) {
        if (connected >= 6) break;
        graph.addEdge(id, vid);
        connected++;
    }

    if (connected == 0) {
        // Fallback: If no node is found within the radius, find the closest collision-free
        // neighbor and connect to it. This is a pragmatic solution to prevent
        // disconnection in cases where the SPARS graph is locally very sparse.
        std::cerr << "Warning: No neighbor found within search_radius (" << search_radius
                    << ") for point [" << point.transpose()
                    << "]. Fallback: searching for the nearest neighbor.\n";

        std::pair<double, int> best_candidate = {std::numeric_limits<double>::max(), -1};
        for (const auto& v : graph.getVertices()) {
            if (v.id == id) continue;
            double dist = (v.pos - point).norm();
            if (dist < best_candidate.first && collisionChecker.isEdgeFree(point, v.pos)) {
                best_candidate = {dist, v.id};
            }
        }

        if (best_candidate.second != -1) {
            graph.addEdge(id, best_candidate.second);
            connected++;
        }
    }

    if (connected == 0) { // Check again after fallback
        std::cerr << "ERROR: Point [" << point.transpose()
                    << "] cannot be connected to the roadmap in any way!\n";
        // Report failure instead of leaving this point isolated.
        return -1;
    }
    return id; // Successful connection.

}
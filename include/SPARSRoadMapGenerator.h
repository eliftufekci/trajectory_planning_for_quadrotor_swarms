#pragma once
#include <map>
#include <tuple>
#include <vector>
#include <limits>
#include <Eigen/Dense>
#include "Environment.h"
#include "FCLCollisionChecker.h"
#include "Graph.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerTerminationCondition.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class SPARSRoadMapGenerator{
public:
    const Environment& env;
    const FCLCollisionChecker& collisionChecker;
    double sparseDeltaFraction;


    SPARSRoadMapGenerator(  const Environment& env, 
                            const FCLCollisionChecker& collisionChecker, 
                            double sparseDeltaFraction = 0.05)
        : env(env), collisionChecker(collisionChecker), sparseDeltaFraction(sparseDeltaFraction) {}

    Graph createRoadMap() {
        Graph roadMap;
        std::map<unsigned int, int> vertexMap;

        auto space(std::make_shared<ob::RealVectorStateSpace>(3)); // Example: 3D Euclidean space
        ob::RealVectorBounds bounds(3); //Degree of Freedom
        bounds.setLow(0, env.world_min.x()); bounds.setHigh(0, env.world_max.x());
        bounds.setLow(1, env.world_min.y()); bounds.setHigh(1, env.world_max.y());
        bounds.setLow(2, env.world_min.z()); bounds.setHigh(2, env.world_max.z());
        space->setBounds(bounds);

        auto si(std::make_shared<ob::SpaceInformation>(space));
        si->setStateValidityChecker([this](const ob::State* s) {
            return isStateValid(s);
        });
        si->setup();

        auto planner = std::make_shared<og::SPARS>(si);
        planner->setSparseDeltaFraction(sparseDeltaFraction);

        // SPARS planlayıcısının çalışabilmesi için bir ProblemDefinition nesnesi gereklidir.
        
        ob::ScopedState<ob::RealVectorStateSpace> startState(space);

        auto pdef = std::make_shared<ob::ProblemDefinition>(si);
        startState[0] = env.agents[0].start.x();
        startState[1] = env.agents[0].start.y();
        startState[2] = env.agents[0].start.z();

        goalState[0] = env.agents[0].goal.x();
        goalState[1] = env.agents[0].goal.y();
        goalState[2] = env.agents[0].goal.z();

        pdef->addStartState(startState);
        planner->setProblemDefinition(pdef);
        
        planner->setProblemDefinition(pdef);

        // Planlayıcıyı belirli bir süre (örneğin 10 saniye) çalıştıktan sonra durdur.
        // SPARS gibi yol haritası oluşturucular için bu genellikle yeterlidir.
        ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(10.0);
        ob::PlannerStatus solved = planner->solve(ptc);

        if (!solved) {
            std::cerr << "Cannot create SPARS RoadMap!\n";
            return roadMap;
        }

        // Roadmap'i bir veri yapısına aktar
        ob::PlannerData data(si);
        planner->getPlannerData(data);

        addVertices(roadMap, vertexMap, data);
        addEdges(roadMap, vertexMap, data);
        connectAgents(roadMap, vertexMap);

        return roadMap;

    }


    bool isStateValid(const ob::State *state) {
        const auto *pos = state->as<ob::RealVectorStateSpace::StateType>();
        double x = pos->values[0], y = pos->values[1], z = pos->values[2];
    
        Eigen::Vector3d point(x, y, z);
    
        return !collisionChecker.isOccupied(point);
    }

private:
    double computeSearchRadius(){
        double xmax_xmin = env.world_max.x() - env.world_min.x();
        double ymax_ymin = env.world_max.y() - env.world_min.y();
        double zmax_zmin = env.world_max.z() - env.world_min.z();
        
        double max_extent = sqrt( pow((xmax_xmin),2)
                                + pow((ymax_ymin),2)
                                + pow((zmax_zmin),2));

        return max_extent * sparseDeltaFraction;
    }

    void addVertices(Graph& graph, std::map<unsigned int, int>& vertexMap, ob::PlannerData& data){
        for(unsigned int i=0; i < data.numVertices(); i++){
            const auto& vertex = data.getVertex(i);
            
            const auto *state = vertex.getState()->as<ob::RealVectorStateSpace::StateType>();
            double x = state->values[0];
            double y = state->values[1];
            double z = state->values[2];

            Eigen::Vector3d pos(x, y, z);

            vertexMap[i] = graph.addVertex(pos); 
        }
    }

    void addEdges(Graph& graph, std::map<unsigned int, int>& vertexMap, ob::PlannerData& data){
        for(unsigned int i=0; i < data.numVertices(); i++){
            std::vector<unsigned int> edgeList;
            data.getEdges(i, edgeList);

            for(unsigned int neighborIndex : edgeList)
                graph.addEdge(vertexMap.at(i), vertexMap.at(neighborIndex));
            
        }
    }

    void connectAgents(Graph& graph, std::map<unsigned int, int>& vertexMap){
        for (const auto& agent : env.agents) {
            connectPoint(agent.start, graph, vertexMap);
            connectPoint(agent.goal,  graph, vertexMap);
        }
    }

    void connectPoint(  const Eigen::Vector3d& point,
                        Graph& graph,
                        std::map<unsigned int, int>& vertexMap){
        
        int id = graph.addVertex(point);

        double search_radius = computeSearchRadius();
        for (const auto& vertex : graph.getVertices()){
            if(id == vertex.id) continue;

            double dist = (vertex.pos - point).norm();
            if(dist <= search_radius && collisionChecker.isEdgeFree(point, vertex.pos)){
                graph.addEdge(id, vertex.id);
            }
        }
    }
};

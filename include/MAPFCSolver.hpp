#pragma once
#include "ConflictAnnotation.hpp"
#include "libMultiRobotPlanning/ecbs.hpp"
#include "DiscreteSchedule.hpp"
#include "Graph.hpp"
#include "MAPFCTypes.hpp"

class MPAFCSolver{
public:
    const Graph& graph;
    const ConflictAnnotation& annotation;
    
    MPAFCSolver(const Graph& graph, const ConflictAnnotation& annotation)
        : graph(graph), annotation(annotation) {}

    DiscreteSchedule solve(){
        std::vector<State> startStates = getStartStates();
        MAPFCEnvironment env(graph, annotation);

        using ECBS_t = libMultiRobotPlanning::ECBS< State, Action, int, 
                                                    Conflict, Constraints, 
                                                    MAPFCEnvironment>;        

        auto w = 1.3; // suboptimality factor
        ECBS_t ecbs(env, w);

        // `w` ne kadar küçükse o kadar optimal ama yavaş. 1.0 = optimal CBS, 1.3-1.5 pratikte iyi denge.

        std::vector<PlanResult<State, Action, int>> solution;
        bool success = ecbs.search(startStates, solution);

        std::vector<std::vector<int>> waypoints;
        if (success) {
            waypoints.resize(solution.size());
            for(size_t i = 0; i < solution.size(); i++){
                for(const auto& state_pair : solution[i].states){
                    waypoints[i].push_back(state_pair.first.vertex_id);
                }
            }
        }
        return DiscreteSchedule(waypoints, ecbs.getMakespan());
    }

private:

    std::vector<State> getStartStates(){
        std::vector<State> startStates;
        for(size_t i=0; i < graph.start_vertices.size(); i++)
            startStates.emplace_back(0, graph.start_vertices[i]);
        
        return startStates;
    }

};

#pragma once
#include "ConflictAnnotation.hpp"
#include "libMultiRobotPlanning/ecbs.hpp"
#include "DiscreteSchedule.hpp"
#include "Graph.hpp"
#include "MAPFCTypes.hpp"
#include "MAPFCEnvironment.hpp"

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

        std::vector<libMultiRobotPlanning::PlanResult<State, Action, int>> solution;
        bool success = ecbs.search(startStates, solution);

        int makespan = 0;
        std::vector<std::vector<int>> waypoints;
        if (success) {
            waypoints.resize(solution.size());
            for(size_t i = 0; i < solution.size(); i++){
                makespan = std::max(makespan, (int)solution[i].states.size() - 1);
                for(const auto& state_pair : solution[i].states){
                    waypoints[i].push_back(state_pair.first.vertex_id);
                }
            }

            std::ofstream paths_file("paths.csv");
            paths_file << "agent_id,timestep,vertex_id,x,y,z\n";
            for (size_t i = 0; i < solution.size(); ++i) {
                for (size_t t = 0; t < solution[i].states.size(); ++t) {
                    const auto& state = solution[i].states[t].first;
                    const auto& pos = graph.vertices[state.vertex_id].pos;
                    paths_file << i << "," << t << "," << state.vertex_id << ","
                               << pos.x() << "," << pos.y() << "," << pos.z() << "\n";
                }
            }
            std::cout << "Kaydedildi: paths.csv\n";
        }
        return DiscreteSchedule(waypoints, makespan);
    }

private:

    std::vector<State> getStartStates(){
        std::vector<State> startStates;
        for(size_t i=0; i < graph.start_vertices.size(); i++)
            startStates.emplace_back(0, graph.start_vertices[i]);
        
        return startStates;
    }

};

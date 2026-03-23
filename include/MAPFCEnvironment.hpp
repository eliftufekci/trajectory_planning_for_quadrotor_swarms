#pragma once
#include "ConflictAnnotation.hpp"
#include "Graph.hpp"
#include "MAPFCTypes.hpp"


class MAPFCEnvironment{
public:
    size_t m_agentIdx; // şu anda hangi agent için low-level serach yapılıyor 
    const Constraints* m_constraints; // o agent için aktif constraint seti
    int m_lastGoalConstraint;

    void setLowLevelContext(size_t agentIdx, const Constraints* constraints) {
        m_agentIdx = agentIdx;
        m_constraints = constraints;
        m_lastGoalConstraint = -1;
        for (const auto& vc : constraints->vertexConstraints) {
            if (vc.vertex_id == graph.goal_vertices[m_agentIdx]) {
                m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
            }
        }
    }

    int admissibleHeuristic(const State& s) {
        return (graph.vertices[s.vertex_id].pos - graph.goal_vertices[s.vertex_id].pos).norm();
    }


    bool isSolution(const State& s) {
        return s.vertex_id == graph.goal_vertices[m_agentIdx] && s.time > m_lastGoalConstraint;
    }


    void getNeighbors(const State& s, std::vector<Neighbor<State, Action, int> >& neighbors) {
        neighbors.clear();
        
        State n(s.time + 1, s.vertex_id);
        for( auto& adj_id : graph.adjacency[s.vertex_id]){
            int edge_id = graph.getEdgeId(s.vertex_id, adj_id);

            auto next_state = State(s.time + 1, adj_id);

            if(stateValid(next_state) && transitionValid(s.time, edge_id)){
                neighbors.emplace_back(next_state, Action(edge_id), 1);
            }

            next_state = State(s.time, vertex_id);
            if(stateValid(next_state)){
                neighbors.emplace_back(next_state, Action(-1), 1);
            }

        }
    }

    bool getFirstConflict( const std::vector<PlanResult<State, Action, int> >& solution, Conflict& result) {
        // int max_t = 0;
        // for (const auto& sol : solution) {
        //     max_t = std::max<int>(max_t, sol.states.size() - 1);
        // }

        // for (int t = 0; t <= max_t; ++t) {
        //     // check drive-drive vertex collisions
        //     for (size_t i = 0; i < solution.size(); ++i) {
        //         State state1 = getState(i, solution, t);
        //         for (size_t j = i + 1; j < solution.size(); ++j) {
        //         State state2 = getState(j, solution, t);
        //         if (state1.equalExceptTime(state2)) {
        //             result.time = t;
        //             result.agent1 = i;
        //             result.agent2 = j;
        //             result.type = Conflict::Vertex;
        //             result.x1 = state1.x;
        //             result.y1 = state1.y;
        //             // std::cout << "VC " << t << "," << state1.x << "," << state1.y <<
        //             // std::endl;
        //             return true;
        //         }
        //         }
        //     }
        //     // drive-drive edge (swap)
        //     for (size_t i = 0; i < solution.size(); ++i) {
        //         State state1a = getState(i, solution, t);
        //         State state1b = getState(i, solution, t + 1);
        //         for (size_t j = i + 1; j < solution.size(); ++j) {
        //         State state2a = getState(j, solution, t);
        //         State state2b = getState(j, solution, t + 1);
        //         if (state1a.equalExceptTime(state2b) &&
        //             state1b.equalExceptTime(state2a)) {
        //             result.time = t;
        //             result.agent1 = i;
        //             result.agent2 = j;
        //             result.type = Conflict::Edge;
        //             result.x1 = state1a.x;
        //             result.y1 = state1a.y;
        //             result.x2 = state1b.x;
        //             result.y2 = state1b.y;
        //             return true;
        //         }
        //         }
        //     }
        // }

        // return false;
    }

private:
    const Graph& graph;
    const ConflictAnnotation& annotation;

    bool stateValid(const State& s) {
        const auto& con = m_constraints->vertexConstraints;
        return con.find(VertexConstraint(s.time, s.vertex_id)) == con.end();
    }

    bool transitionValid(const State& s1, const State& s2) {
        const auto& con = m_constraints->edgeConstraints;
        for (const auto& ec : con) {
            if (ec.edge_id == graph.getEdgeId(s1.vertex_id, s2.vertex_id)) {
                return false;
            }
        }
        return true;
    }

};
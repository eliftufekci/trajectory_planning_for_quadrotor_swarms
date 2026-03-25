#pragma once
#include "libMultiRobotPlanning/neighbor.hpp"
#include "libMultiRobotPlanning/planresult.hpp"
#include "ConflictAnnotation.hpp"
#include "Graph.hpp"
#include "MAPFCTypes.hpp"


class MAPFCEnvironment{
public:
    MAPFCEnvironment(const Graph& graph, const ConflictAnnotation& annotation)
        : graph(graph), annotation(annotation), m_highLevelExpanded(0), m_lowLevelExpanded(0), m_agentIdx(0), m_constraints(nullptr), m_lastGoalConstraint(-1) {}


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
        return static_cast<int>((graph.vertices[s.vertex_id].pos - graph.vertices[graph.goal_vertices[m_agentIdx]].pos).norm());
    }

    bool isSolution(const State& s) {
        return s.vertex_id == graph.goal_vertices[m_agentIdx] && s.time > m_lastGoalConstraint;
    }

    void getNeighbors(const State& s, std::vector<libMultiRobotPlanning::Neighbor<State, Action, int> >& neighbors) {
        neighbors.clear();
        
        // Move actions
        for( const auto& adj_id : graph.neighbors(s.vertex_id)){
            int edge_id = graph.getEdgeId(s.vertex_id, adj_id);

            auto next_state = State(s.time + 1, adj_id);

            if(stateValid(next_state) && transitionValid(s.time, edge_id)){
                neighbors.emplace_back(next_state, Action(edge_id), 1);
            }

        }

        // Wait action
        State wait_state(s.time + 1, s.vertex_id);
        if(stateValid(wait_state)){
            neighbors.emplace_back(wait_state, Action(-1), 1); // Wait action has edge_id -1
        }
    }

    bool getFirstConflict( const std::vector<libMultiRobotPlanning::PlanResult<State, Action, int> >& solution, Conflict& result) {
        int max_t = 0;
        for (const auto& sol : solution) {
            max_t = std::max<int>(max_t, sol.states.size() - 1);
        }

        for (int t = 0; t <= max_t; ++t) {
            for (size_t i = 0; i < solution.size(); ++i) {
                State state1 = getState(i, solution, t);
                
                for (size_t j = i + 1; j < solution.size(); ++j) {
                    State state2 = getState(j, solution, t);
                    // P4: classic vertex collision
                    if (state1.equalExceptTime(state2)) {
                        result.time = t;
                        result.agent1 = i;
                        result.agent2 = j;
                        result.type = Conflict::vertex;
                        result.vertex_id = state1.vertex_id;
                        return true;
                    }

                    // P6: conVV
                    if(annotation.conVV.count(state1.vertex_id) &&
                       annotation.conVV.at(state1.vertex_id).count(state2.vertex_id)){
                        result.time = t;
                        result.agent1 = i;
                        result.agent2 = j;
                        result.type = Conflict::conVV;
                        result.vertex_id = state1.vertex_id;
                        return true;
                    }

                    bool i_moving = t < solution[i].actions.size();
                    bool j_moving = t < solution[j].actions.size();

                    if(i_moving && j_moving){
                        auto e1 = solution[i].actions[t].first.edge_id;
                        auto e2 = solution[j].actions[t].first.edge_id;

                        // P5: classic edge swap
                        // i: u->v, j:v->u
                        if(e1 == graph.getEdgeId(state1.vertex_id, state2.vertex_id) &&
                           e2 == graph.getEdgeId(state2.vertex_id, state1.vertex_id)){
                            result.time = t;
                            result.agent1 = i;
                            result.agent2 = j;
                            result.type = Conflict::edge;
                            result.edge_id1 = e1;
                            result.edge_id2 = e2;
                            return true;
                        }

                        // P7: conEE
                        if(annotation.conEE.count(e1) &&
                           annotation.conEE.at(e1).count(e2)){
                            result.time = t;
                            result.agent1 = i;
                            result.agent2 = j;
                            result.type = Conflict::conEE;
                            result.edge_id1 = e1;
                            result.edge_id2 = e2;
                            return true;
                        }
                    }

                    // P8: conEV
                    if(i_moving && !j_moving){
                        auto e1 = solution[i].actions[t].first.edge_id;
                        if(annotation.conEV.count(e1) &&
                            annotation.conEV.at(e1).count(state2.vertex_id)){
                            result.time = t;
                            result.agent1 = i;
                            result.agent2 = j;
                            result.type = Conflict::conEV;
                            result.edge_id1 = e1;
                            result.vertex_id = state2.vertex_id;
                            return true;
                        }
                    }
                    if(!i_moving && j_moving){
                        auto e2 = solution[j].actions[t].first.edge_id;
                        if(annotation.conEV.count(e2) &&
                            annotation.conEV.at(e2).count(state1.vertex_id)){
                            result.time = t;
                            result.agent1 = i;
                            result.agent2 = j;
                            result.type = Conflict::conEV;
                            result.edge_id1 = e2;
                            result.vertex_id = state1.vertex_id;
                            return true;
                        }
                    }
                }
            }
        }

        return false;
    }

    void createConstraintsFromConflict(
        const Conflict& conflict, std::map<size_t, Constraints>& constraints) {
        if (conflict.type == Conflict::vertex || conflict.type == Conflict::conVV) {
            Constraints c1;
            c1.vertexConstraints.emplace(
                VertexConstraint(conflict.time, conflict.vertex_id));
            constraints[conflict.agent1] = c1;
            constraints[conflict.agent2] = c1;
        } else if (conflict.type == Conflict::edge || conflict.type == Conflict::conEE) {
            Constraints c1;
            c1.edgeConstraints.emplace(EdgeConstraint(
                conflict.time, conflict.edge_id1));
            constraints[conflict.agent1] = c1;  
            Constraints c2;
            c2.edgeConstraints.emplace(EdgeConstraint(conflict.time, conflict.edge_id2));
            constraints[conflict.agent2] = c2;
        } else if (conflict.type == Conflict::conEV) {
            Constraints c1;
            c1.edgeConstraints.emplace(EdgeConstraint(
                conflict.time, conflict.edge_id1));
            constraints[conflict.agent1] = c1;  
            Constraints c2;
            // Constrain the waiting agent at the next timestep to resolve the conflict over the interval [t, t+1]
            c2.vertexConstraints.emplace(VertexConstraint(conflict.time + 1, conflict.vertex_id));
            constraints[conflict.agent2] = c2;
        }
    }

    // low-level
    int focalStateHeuristic(
        const State& s, int /*gScore*/,
        const std::vector<libMultiRobotPlanning::PlanResult<State, Action, int> >& solution) {
        int numConflicts = 0;
        for (size_t i = 0; i < solution.size(); ++i) {
            if (i != m_agentIdx && !solution[i].states.empty()) {
                State state2 = getState(i, solution, s.time);
                if (s.equalExceptTime(state2)) {
                    ++numConflicts;
                }
            }
        }
        return numConflicts;
    }

    // low-level
    int focalTransitionHeuristic(
        const State& s1a, const State& s1b, int /*gScoreS1a*/, int /*gScoreS1b*/,
        const std::vector<libMultiRobotPlanning::PlanResult<State, Action, int> >& solution) {
        int numConflicts = 0;
        for (size_t i = 0; i < solution.size(); ++i) {
            if (i != m_agentIdx && !solution[i].states.empty()) {
                State s2a = getState(i, solution, s1a.time);
                State s2b = getState(i, solution, s1b.time);
                if (s1a.equalExceptTime(s2b) && s1b.equalExceptTime(s2a)) {
                    ++numConflicts;
                }
            }
        }
        return numConflicts;
    }

    // Count all conflicts
    int focalHeuristic(
        const std::vector<libMultiRobotPlanning::PlanResult<State, Action, int> >& solution) {
        int numConflicts = 0;

        int max_t = 0;
        for (const auto& sol : solution) {
            max_t = std::max<int>(max_t, sol.states.size() - 1);
        }

        for (int t = 0; t < max_t; ++t) {
            // check drive-drive vertex collisions
            for (size_t i = 0; i < solution.size(); ++i) {
                State state1 = getState(i, solution, t);
                for (size_t j = i + 1; j < solution.size(); ++j) {
                    State state2 = getState(j, solution, t);
                    // P4: classic vertex collision
                    if (state1.equalExceptTime(state2)) {
                        ++numConflicts;
                    }

                    // P6: conVV
                    if(annotation.conVV.count(state1.vertex_id) &&
                       annotation.conVV.at(state1.vertex_id).count(state2.vertex_id)){
                        ++numConflicts;
                    }

                    bool i_moving = t < solution[i].actions.size();
                    bool j_moving = t < solution[j].actions.size();

                    if(i_moving && j_moving){
                        auto e1 = solution[i].actions[t].first.edge_id;
                        auto e2 = solution[j].actions[t].first.edge_id;

                        // P5: classic edge swap
                        // i: u->v, j:v->u
                        if(e1 == graph.getEdgeId(state1.vertex_id, state2.vertex_id) &&
                           e2 == graph.getEdgeId(state2.vertex_id, state1.vertex_id)){
                            ++numConflicts;
                        }

                        // P7: conEE
                        if(annotation.conEE.count(e1) &&
                           annotation.conEE.at(e1).count(e2)){
                            ++numConflicts;
                        }
                    }

                    // P8: conEV
                    if(i_moving && !j_moving){
                        auto e1 = solution[i].actions[t].first.edge_id;
                        if(annotation.conEV.count(e1) &&
                            annotation.conEV.at(e1).count(state2.vertex_id)){
                            ++numConflicts;
                        }
                    }
                    if(!i_moving && j_moving){
                        auto e2 = solution[j].actions[t].first.edge_id;
                        if(annotation.conEV.count(e2) &&
                            annotation.conEV.at(e2).count(state1.vertex_id)){
                            ++numConflicts;
                        }
                    }
                }
            }
        }
        return numConflicts;
    }

    void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

    void onExpandLowLevelNode(const State& /*s*/, int /*fScore*/, int /*gScore*/) { m_lowLevelExpanded++; }

    int highLevelExpanded() { return m_highLevelExpanded; }

    int lowLevelExpanded() const { return m_lowLevelExpanded; }

private:
    const Graph& graph;
    const ConflictAnnotation& annotation;
    size_t m_agentIdx; // şu anda hangi agent için low-level serach yapılıyor 
    const Constraints* m_constraints; // o agent için aktif constraint seti
    int m_lastGoalConstraint;
    int m_highLevelExpanded;
    int m_lowLevelExpanded;

    State getState(size_t agentIdx, const std::vector<libMultiRobotPlanning::PlanResult<State, Action, int> >& solution, size_t t) {
        if (t < solution[agentIdx].states.size()) {
            return solution[agentIdx].states[t].first;
        }

        return solution[agentIdx].states.back().first; // goalde bekliyor
    }

    bool stateValid(const State& s) {
        const auto& con = m_constraints->vertexConstraints;
        return con.find(VertexConstraint(s.time, s.vertex_id)) == con.end();
    }

    bool transitionValid(int time, int edge_id) {
        const auto& con = m_constraints->edgeConstraints;
        return con.find(EdgeConstraint(time, edge_id)) == con.end();
    }

};
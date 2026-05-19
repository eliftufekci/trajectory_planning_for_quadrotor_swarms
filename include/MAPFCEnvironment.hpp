#pragma once
#include "libMultiRobotPlanning/neighbor.hpp"
#include "libMultiRobotPlanning/planresult.hpp"
#include "ConflictAnnotation.hpp"
#include "Graph.hpp"
#include "MAPFCTypes.hpp"


class MAPFCEnvironment{
public:
    MAPFCEnvironment(const Graph& graph, const ConflictAnnotation& annotation);

    void setLowLevelContext(size_t agentIdx, const Constraints* constraints);

    int admissibleHeuristic(const State& s);

    bool isSolution(const State& s);

    void getNeighbors(const State& s, std::vector<libMultiRobotPlanning::Neighbor<State, Action, int> >& neighbors);

    bool getFirstConflict( const std::vector<libMultiRobotPlanning::PlanResult<State, Action, int> >& solution, Conflict& result);
    
    void createConstraintsFromConflict(
        const Conflict& conflict, std::map<size_t, Constraints>& constraints);

    // low-level
    int focalStateHeuristic(
        const State& s, int /*gScore*/,
        const std::vector<libMultiRobotPlanning::PlanResult<State, Action, int> >& solution);

    // low-level
    int focalTransitionHeuristic(
        const State& s1a, const State& s1b, int /*gScoreS1a*/, int /*gScoreS1b*/,
        const std::vector<libMultiRobotPlanning::PlanResult<State, Action, int> >& solution);

    // Count all conflicts
    int focalHeuristic(const std::vector<libMultiRobotPlanning::PlanResult<State, Action, int> >& solution);

    void onExpandHighLevelNode(int /*cost*/);

    void onExpandLowLevelNode(const State& /*s*/, int /*fScore*/, int /*gScore*/);

    int highLevelExpanded();

    int lowLevelExpanded() const;

private:
    const Graph& graph;
    const ConflictAnnotation& annotation;
    size_t m_agentIdx; // şu anda hangi agent için low-level serach yapılıyor 
    const Constraints* m_constraints; // o agent için aktif constraint seti
    int m_lastGoalConstraint;
    int m_highLevelExpanded;
    int m_lowLevelExpanded;

    State getState(size_t agentIdx, const std::vector<libMultiRobotPlanning::PlanResult<State, Action, int> >& solution, size_t t);

    bool stateValid(const State& s);

    bool transitionValid(int time, int edge_id);
};
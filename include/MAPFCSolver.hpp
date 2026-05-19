#pragma once
#include <iostream>
#include "ConflictAnnotation.hpp"
#include "libMultiRobotPlanning/ecbs.hpp"
#include "DiscreteSchedule.hpp"
#include "Graph.hpp"
#include "MAPFCTypes.hpp"
#include "MAPFCEnvironment.hpp"

class MAPFCSolver{
public:
    const Graph& graph;
    const ConflictAnnotation& annotation;
    
    MAPFCSolver(const Graph& graph, const ConflictAnnotation& annotation);

    DiscreteSchedule solve();

private:

    std::vector<State> getStartStates();

};

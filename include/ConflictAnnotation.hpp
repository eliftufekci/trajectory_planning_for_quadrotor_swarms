#pragma once
#include <map>
#include <set>
#include "Graph.hpp"
#include "RobotModel.hpp"

class ConflictAnnotation {
public:
    virtual ~ConflictAnnotation() = default;

    virtual void annotate() = 0;

    std::map<int, std::set<int>> conVV;
    std::map<int, std::set<int>> conEV;
    std::map<int, std::set<int>> conEE;

protected:
    ConflictAnnotation(const Graph& graph, const RobotModel& robotModel)
        : graph(graph), robotModel(robotModel) {}

    const Graph& graph;
    const RobotModel& robotModel;
};
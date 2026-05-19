#pragma once
#include <map>
#include <set>

class Graph;
struct RobotModel;

class ConflictAnnotation {
public:
    virtual ~ConflictAnnotation() = default;

    virtual void annotate() = 0;

    std::map<int, std::set<int>> conVV;
    std::map<int, std::set<int>> conEV;
    std::map<int, std::set<int>> conEE;

protected:
    ConflictAnnotation(const Graph& graph, const RobotModel& robotModel);

    const Graph& graph;
    const RobotModel& robotModel;

    // Shared method for finding vertex-vertex conflicts.
    std::map<int, std::set<int>> findBaseConVV();
};

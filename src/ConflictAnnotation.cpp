#include <map>
#include <set>
#include "Graph.hpp"
#include "RobotModel.hpp"
#include "ConflictAnnotation.hpp"

ConflictAnnotation::ConflictAnnotation(const Graph& graph, const RobotModel& robotModel)
        : graph(graph), robotModel(robotModel) {}

std::map<int, std::set<int>> ConflictAnnotation::findBaseConVV() {
    std::map<int, std::set<int>> conVVResult;

    for(size_t i = 0; i < graph.vertices.size(); i++){
        for(size_t j = i + 1; j < graph.vertices.size(); j++){
            const auto& v = graph.vertices[i];
            const auto& u = graph.vertices[j];

            if(robotModel.collides(v.pos, u.pos)){
                conVVResult[v.id].insert(u.id);
                conVVResult[u.id].insert(v.id);
            }
        }
    }
    return conVVResult;
}
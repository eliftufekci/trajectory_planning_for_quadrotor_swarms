#pragma once
#include <Eigen/src/Core/Matrix.h>
#include <map>
#include <vector>
#include <set>
#include <Eigen/Dense>
#include <algorithm>
#include "Graph.h"
#include "RobotModel.h"

struct ConflictAnnotation{
    const Graph& graph;
    const RobotModel& robotModel;
    
    ConflictAnnotation(const Graph& graph, const RobotModel& robotModel) 
            : graph(graph), robotModel(robotModel) {}
};

inline Eigen::Vector3d segment_to_point(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& p){
    Eigen::Vector3d ab = b - a;
    double len_sq = ab.squaredNorm();
    if (len_sq < 1e-8) {
        return a;
    }
    double t = (p - a).dot(ab) / len_sq;
    t = std::clamp(t, 0.0, 1.0);
    return a + t * ab;
}

// map<vertex_id, set<vertex_id>>
std::map<int, std::set<int>> findCovVV(Graph& graph, const RobotModel& robotModel){
    std::map<int, std::set<int>> conVV;

    for(int i = 0; i < graph.vertices.size(); i++){
        for(int j = i + 1; j < graph.vertices.size(); j++){
            const auto v = graph.vertices[i];
            const auto u = graph.vertices[j];

            if(robotModel.collides(v.pos, u.pos)){
                conVV[v.id].insert(u.id);
                conVV[u.id].insert(v.id);
            }

        }
    }

    return conVV;
}  

// map<edge_id,   set<vertex_id>>
std::map<int, std::set<int>> findConEV(Graph& graph, const RobotModel& robotModel){
    std::map<int, std::set<int>> conEV;

    for(int i = 0; i < graph.edges.size(); i++){
        for(int j = 0; j < graph.vertices.size(); j++){
            const auto e = graph.edges[i];
            const auto v = graph.vertices[j];

            auto point = segment_to_point(e.from.pos, e.to.pos, v.pos);

            if(robotModel.collides(v.pos, point)){
                conEV[e.id].insert(v.id);
            }

        }
    }
    return conEV;
}



// map<edge_id, set<edge_id>> 
std::map<int, std::set<int>> findConEE(Graph& graph, const RobotModel& robotModel){
    std::map<int, std::set<int>> conEE;

    for(int i = 0; i < graph.edges.size(); i++){
        for(int j = i + 1; j < graph.edges.size(); j++){
            const auto e = graph.edges[i];
            const auto f = graph.edges[j];


        }
    }
    return conEE;
}

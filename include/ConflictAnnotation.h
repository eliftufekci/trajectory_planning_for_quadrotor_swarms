#pragma once
#include <Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <map>
#include <vector>
#include <tuple>
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

inline std::tuple<Eigen::Vector3d, Eigen::Vector3d> segment_to_segment(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3, const Eigen::Vector3d& p4){
    Eigen::Vector3d u = p2 - p1;
    Eigen::Vector3d v = p4 - p3;
    Eigen::Vector3d w = p1 - p3;

    double a = u.squaredNorm();
    double b = u.dot(v);
    double c = v.squaredNorm();
    double d = u.dot(w);
    double e = v.dot(w);

    double denom = a*c - b*b; // paralel mi
    double s = 0.0, t = 0.0;

    if (a < 1e-8 && c < 1e-8) {
        s = 0.0;
        t = 0.0;
    } else if (a < 1e-8) {
        s = 0.0;
        t = std::clamp(e / c, 0.0, 1.0);
    } else if (c < 1e-8) {
        t = 0.0;
        s = std::clamp(-d / a, 0.0, 1.0);
    } else {
        if (denom > 1e-8) {
            s = std::clamp((b*e - c*d) / denom, 0.0, 1.0);
        } else {
            s = 0.0; // Paralellerse s'i sabitleyip t'yi buluyoruz
        }

        t = (b*s + e) / c;

        if (t < 0.0) {
            t = 0.0;
            s = std::clamp(-d / a, 0.0, 1.0);
        } else if (t > 1.0) {
            t = 1.0;
            s = std::clamp((b - d) / a, 0.0, 1.0);
        }
    }

    Eigen::Vector3d point1 = p1 + s*u;
    Eigen::Vector3d point2 = p3 + t*v;

    return {point1, point2};
}

// map<vertex_id, set<vertex_id>>
std::map<int, std::set<int>> findCovVV(Graph& graph, const RobotModel& robotModel){
    std::map<int, std::set<int>> conVV;

    for(int i = 0; i < graph.vertices.size(); i++){
        for(int j = i + 1; j < graph.vertices.size(); j++){
            const auto& v = graph.vertices[i];
            const auto& u = graph.vertices[j];

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
            const auto& e = graph.edges[i];
            const auto& v = graph.vertices[j];

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
            const auto& e = graph.edges[i];
            const auto& f = graph.edges[j];

            auto [point1, point2] = segment_to_segment(e.from.pos, e.to.pos, f.from.pos, f.to.pos);
            if(robotModel.collides(point1, point2)){
                conEE[e.id].insert(f.id);
                conEE[f.id].insert(e.id);
            }
            
        }
    }
    return conEE;
}

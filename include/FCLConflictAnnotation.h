#pragma once
#include <map>
#include <set>
#include "Graph.h"
#include "RobotModel.h"
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/continuous_collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/geometry/shape/ellipsoid.h>

struct FCLConflictAnnotation{
    const Graph& graph;
    const RobotModel& robotModel;

    std::map<int, std::set<int>> conVV;
    std::map<int, std::set<int>> conEV;
    std::map<int, std::set<int>> conEE;

    FCLConflictAnnotation(const Graph& graph, const RobotModel& robotModel) 
            : graph(graph), robotModel(robotModel) {}
};

// map<vertex_id, set<vertex_id>>
std::map<int, std::set<int>> findConVV(Graph& graph, const RobotModel& robotModel){
    std::map<int, std::set<int>> conVV;
    auto ellipsoid_geom = std::make_shared<fcl::Ellipsoidd>(robotModel.rx, robotModel.ry, robotModel.rz);

    for(int i = 0; i < graph.vertices.size(); i++){
        for(int j = i + 1; j < graph.vertices.size(); j++){
            const auto& v = graph.vertices[i];
            const auto& u = graph.vertices[j];

            fcl::Transform3d v_position = fcl::Transform3d::Identity();
            v_position.translation() = v.pos;

            fcl::Transform3d u_position = fcl::Transform3d::Identity();
            u_position.translation() = u.pos;

            fcl::CollisionObjectd v_obj(ellipsoid_geom, v_position);
            fcl::CollisionObjectd u_obj(ellipsoid_geom, u_position);
            
            fcl::CollisionRequestd request;
            fcl::CollisionResultd result;

            fcl::collide(&v_obj, &u_obj, request, result);

            if (result.isCollision()) {
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
    auto ellipsoid_geom = std::make_shared<fcl::Ellipsoidd>(robotModel.rx, robotModel.ry, robotModel.rz);

    for (const auto& e : graph.getEdges()) {
        for (const auto& v : graph.getVertices()) {
            // Kenarın kendi başlangıç/bitiş noktalarıyla çakışmasını kontrol etme
            if (e.from == v.id || e.to == v.id) continue;

            fcl::Transform3d e_start = fcl::Transform3d::Identity();
            e_start.translation() = graph.vertices[e.from].pos;
            fcl::Transform3d e_end = fcl::Transform3d::Identity();
            e_end.translation() = graph.vertices[e.to].pos;
            fcl::CollisionObjectd e_obj(ellipsoid_geom, e_start);
            
            fcl::Transform3d v_position = fcl::Transform3d::Identity();
            v_position.translation() = v.pos;
            fcl::CollisionObjectd v_obj(ellipsoid_geom, v_position);

            fcl::ContinuousCollisionRequestd request;
            fcl::ContinuousCollisionResultd result;

            fcl::continuousCollide(
                &e_obj, e_end,                    
                &v_obj, v_position, 
                request, result
            );

            if (result.is_collide) {
                conEV[e.id].insert(v.id);
            }

        }
    }

    return conEV;
}


// map<edge_id, set<edge_id>> 
std::map<int, std::set<int>> findConEE(Graph& graph, const RobotModel& robotModel){
    std::map<int, std::set<int>> conEE;
    auto ellipsoid_geom = std::make_shared<fcl::Ellipsoidd>(robotModel.rx, robotModel.ry, robotModel.rz);

    for (size_t i = 0; i < graph.edges.size(); ++i) {
        for (size_t j = i + 1; j < graph.edges.size(); ++j) {
            const auto& e = graph.edges[i];
            const auto& f = graph.edges[j];

            fcl::Transform3d e_start = fcl::Transform3d::Identity();
            e_start.translation() = graph.vertices[e.from].pos;
            fcl::Transform3d e_end = fcl::Transform3d::Identity();
            e_end.translation() = graph.vertices[e.to].pos;
            fcl::CollisionObjectd e_obj(ellipsoid_geom, e_start);
            
            fcl::Transform3d f_start = fcl::Transform3d::Identity();
            f_start.translation() = graph.vertices[f.from].pos;
            fcl::Transform3d f_end = fcl::Transform3d::Identity();
            f_end.translation() = graph.vertices[f.to].pos;
            fcl::CollisionObjectd f_obj(ellipsoid_geom, f_start);

            fcl::ContinuousCollisionRequestd request;
            fcl::ContinuousCollisionResultd result;

            fcl::continuousCollide(
                &e_obj, e_end,                    
                &f_obj, f_end, 
                request, result
            );

            if (result.is_collide) {
                conEE[e.id].insert(f.id);
                conEE[f.id].insert(e.id);
            }

        }
    }
    
    return conEE;
}

FCLConflictAnnotation annotate(Graph& graph, RobotModel& robotModel){
    FCLConflictAnnotation fclConflictAnnotation(graph, robotModel);
    fclConflictAnnotation.conVV = findConVV(graph, robotModel);
    fclConflictAnnotation.conEV = findConEV(graph, robotModel);
    fclConflictAnnotation.conEE = findConEE(graph, robotModel);

    return fclConflictAnnotation;
}

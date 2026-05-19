#include "FCLConflictAnnotation.hpp"
#include "ConflictAnnotation.hpp"
#include "Graph.hpp"
#include "RobotModel.hpp"
#include <fcl/narrowphase/continuous_collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/geometry/shape/ellipsoid.h>


FCLConflictAnnotation::FCLConflictAnnotation(const Graph& graph, const RobotModel& robotModel) 
        : ConflictAnnotation(graph, robotModel) {}

void FCLConflictAnnotation::annotate() {
    conVV = findBaseConVV(); // Call the shared method.
    conEV = findConEV();
    conEE = findConEE();
}

// map<edge_id,   set<vertex_id>>
std::map<int, std::set<int>> FCLConflictAnnotation::findConEV(){
    std::map<int, std::set<int>> conEV;
    auto ellipsoid_geom = std::make_shared<fcl::Ellipsoidd>(robotModel.rx, robotModel.ry, robotModel.rz);

    for (const auto& e : graph.getEdges()) {
        for (const auto& v : graph.getVertices()) {
            // Do not check collision with the edge's own start/end vertices.
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
std::map<int, std::set<int>> FCLConflictAnnotation::findConEE(){
    std::map<int, std::set<int>> conEE;
    auto ellipsoid_geom = std::make_shared<fcl::Ellipsoidd>(robotModel.rx, robotModel.ry, robotModel.rz);

    for (size_t i = 0; i < graph.edges.size(); ++i) {
        for (size_t j = i + 1; j < graph.edges.size(); ++j) {
            const auto& e = graph.edges[i];
            const auto& f = graph.edges[j];

            // Motion definition for edge 'e' (from -> to).
            fcl::Transform3d e_tf_start = fcl::Transform3d::Identity();
            e_tf_start.translation() = graph.vertices[e.from].pos;
            fcl::Transform3d e_tf_end = fcl::Transform3d::Identity();
            e_tf_end.translation() = graph.vertices[e.to].pos;
            fcl::CollisionObjectd e_obj(ellipsoid_geom, e_tf_start);
            
            // Motion definitions for edge 'f'.
            fcl::Transform3d f_tf_start = fcl::Transform3d::Identity();
            f_tf_start.translation() = graph.vertices[f.from].pos;
            fcl::Transform3d f_tf_end = fcl::Transform3d::Identity();
            f_tf_end.translation() = graph.vertices[f.to].pos;
            fcl::CollisionObjectd f_obj(ellipsoid_geom, f_tf_start);

            fcl::ContinuousCollisionRequestd request;
            fcl::ContinuousCollisionResultd result;

            // Check 1: e(from->to) vs f(from->to) (parallel motion).
            fcl::continuousCollide(
                &e_obj, e_tf_end,
                &f_obj, f_tf_end,
                request, result
            );

            // Check 2: e(from->to) vs f(to->from) (opposite-direction motion).
            if (!result.is_collide) {
                f_obj.setTransform(f_tf_end); // Set f's start position to 'to'.
                fcl::continuousCollide(&e_obj, e_tf_end, &f_obj, f_tf_start, request, result);
            }

            if (result.is_collide) { // If there is a collision in either direction.
                conEE[e.id].insert(f.id);
                conEE[f.id].insert(e.id);
            }
        }
    }
    
    return conEE;
}

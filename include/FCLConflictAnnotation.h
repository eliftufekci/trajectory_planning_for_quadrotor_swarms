#pragma once
#include <map>
#include <set>
#include "Graph.h"
#include "RobotModel.h"
#include <fcl/narrowphase/continuous_collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/geometry/shape/ellipsoid.h>

class FCLConflictAnnotation{
public:
    const Graph& graph;
    const RobotModel& robotModel;

    std::map<int, std::set<int>> conVV;
    std::map<int, std::set<int>> conEV;
    std::map<int, std::set<int>> conEE;

    FCLConflictAnnotation(const Graph& graph, const RobotModel& robotModel) 
            : graph(graph), robotModel(robotModel) {}

    FCLConflictAnnotation annotate(){
        FCLConflictAnnotation fclConflictAnnotation(graph, robotModel);
        fclConflictAnnotation.conVV = findConVV(graph, robotModel);
        fclConflictAnnotation.conEV = findConEV(graph, robotModel);
        fclConflictAnnotation.conEE = findConEE(graph, robotModel);

        return fclConflictAnnotation;
    }


private:
    // map<vertex_id, set<vertex_id>>
    std::map<int, std::set<int>> findConVV(Graph& graph, const RobotModel& robotModel){
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

                // Edge 'e' için hareket tanımı (from -> to)
                fcl::Transform3d e_tf_start = fcl::Transform3d::Identity();
                e_tf_start.translation() = graph.vertices[e.from].pos;
                fcl::Transform3d e_tf_end = fcl::Transform3d::Identity();
                e_tf_end.translation() = graph.vertices[e.to].pos;
                fcl::CollisionObjectd e_obj(ellipsoid_geom, e_tf_start);
                
                // Edge 'f' için hareket tanımları
                fcl::Transform3d f_tf_start = fcl::Transform3d::Identity();
                f_tf_start.translation() = graph.vertices[f.from].pos;
                fcl::Transform3d f_tf_end = fcl::Transform3d::Identity();
                f_tf_end.translation() = graph.vertices[f.to].pos;
                fcl::CollisionObjectd f_obj(ellipsoid_geom, f_tf_start);

                fcl::ContinuousCollisionRequestd request;
                fcl::ContinuousCollisionResultd result;

                // Kontrol 1: e(from->to) vs f(from->to) (paralel hareket)
                fcl::continuousCollide(
                    &e_obj, e_tf_end,
                    &f_obj, f_tf_end,
                    request, result
                );

                // Kontrol 2: e(from->to) vs f(to->from) (zıt yönlü hareket)
                if (!result.is_collide) {
                    f_obj.setTransform(f_tf_end); // f'nin başlangıç pozisyonunu 'to' yap
                    fcl::continuousCollide(&e_obj, e_tf_end, &f_obj, f_tf_start, request, result);
                }

                if (result.is_collide) { // Herhangi bir yönde çarpışma varsa
                    conEE[e.id].insert(f.id);
                    conEE[f.id].insert(e.id);
                }
            }
        }
        
        return conEE;
    }
};

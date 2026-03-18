#pragma once
#include <map>
#include <vector>
#include <tuple>
#include <set>
#include <Eigen/Dense>
#include <algorithm>
#include "Graph.h"
#include "RobotModel.h"

class SweptConflictAnnotation{
public:
    const Graph& graph;
    const RobotModel& robotModel;

    std::map<int, std::set<int>> conVV;
    std::map<int, std::set<int>> conEV;
    std::map<int, std::set<int>> conEE;
    
    SweptConflictAnnotation(const Graph& graph, const RobotModel& robotModel) 
            : graph(graph), robotModel(robotModel) {}

    SweptConflictAnnotation annotate(){
        SweptConflictAnnotation sweptConflictAnnotation(graph, robotModel);
        sweptConflictAnnotation.conVV = findConVV(graph, robotModel);
        sweptConflictAnnotation.conEV = findConEV(graph, robotModel);
        sweptConflictAnnotation.conEE = findConEE(graph, robotModel);

        return sweptConflictAnnotation;
    }


private:
    Eigen::Vector3d segment_to_point(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& p){
        Eigen::Vector3d ab = b - a;
        double len_sq = ab.squaredNorm();
        if (len_sq < 1e-8) {
            return a;
        }
        double t = (p - a).dot(ab) / len_sq;
        t = std::clamp(t, 0.0, 1.0);
        return a + t * ab;
    }

    std::tuple<Eigen::Vector3d, Eigen::Vector3d> segment_to_segment(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3, const Eigen::Vector3d& p4){
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

    // Elipsoid uzayını birim küre uzayına dönüştürür.
    Eigen::Vector3d scale_pos(const Eigen::Vector3d& pos, const RobotModel& model) {
        return Eigen::Vector3d(pos.x() / model.rx, pos.y() / model.ry, pos.z() / model.rz);
    }

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

        for (const auto& e : graph.getEdges()) {
            for (const auto& v : graph.getVertices()) {
                // Kenarın kendi başlangıç/bitiş noktalarıyla çakışmasını kontrol etme
                if (e.from == v.id || e.to == v.id) continue;

                // Elipsoid çarpışma kontrolü için uzayı ölçekle
                Eigen::Vector3d e_from_scaled = scale_pos(graph.vertices[e.from].pos, robotModel);
                Eigen::Vector3d e_to_scaled   = scale_pos(graph.vertices[e.to].pos, robotModel);
                Eigen::Vector3d v_pos_scaled  = scale_pos(v.pos, robotModel);

                // Ölçeklenmiş uzayda en yakın noktayı bul
                auto point_on_segment_scaled = segment_to_point(e_from_scaled, e_to_scaled, v_pos_scaled);

                // İki elipsoid çarpışması için, ölçeklenmiş uzayda merkezler arası
                // uzaklığın karesi 4'ten küçük olmalıdır. Bu, Minkowski toplamının
                // yarıçapı 2 katına çıkarmasından kaynaklanır (uzaklık < 2 -> uzaklık^2 < 4).
                if ((v_pos_scaled - point_on_segment_scaled).squaredNorm() < 4.0) {
                    conEV[e.id].insert(v.id);
                }
            }
        }
        return conEV;
    }


    // map<edge_id, set<edge_id>> 
    std::map<int, std::set<int>> findConEE(Graph& graph, const RobotModel& robotModel){
        std::map<int, std::set<int>> conEE;

        for (size_t i = 0; i < graph.edges.size(); ++i) {
            for (size_t j = i + 1; j < graph.edges.size(); ++j) {
                const auto& e = graph.edges[i];
                const auto& f = graph.edges[j];

                // Elipsoid çarpışma kontrolü için uzayı ölçekle
                Eigen::Vector3d e_from_scaled = scale_pos(graph.vertices[e.from].pos, robotModel);
                Eigen::Vector3d e_to_scaled   = scale_pos(graph.vertices[e.to].pos, robotModel);
                Eigen::Vector3d f_from_scaled = scale_pos(graph.vertices[f.from].pos, robotModel);
                Eigen::Vector3d f_to_scaled   = scale_pos(graph.vertices[f.to].pos, robotModel);

                // Ölçeklenmiş uzayda en yakın noktaları bul
                auto [p1_scaled, p2_scaled] = segment_to_segment(e_from_scaled, e_to_scaled, f_from_scaled, f_to_scaled);
                // İki elipsoid çarpışması için, ölçeklenmiş uzayda en yakın noktalar arası
                // uzaklığın karesi 4'ten küçük olmalıdır. Bu, Minkowski toplamının
                // yarıçapı 2 katına çıkarmasından kaynaklanır (uzaklık < 2 -> uzaklık^2 < 4).
                if ((p1_scaled - p2_scaled).squaredNorm() < 4.0) {
                    conEE[e.id].insert(f.id);
                    conEE[f.id].insert(e.id);
                }
                
            }
        }
        return conEE;
    }

};

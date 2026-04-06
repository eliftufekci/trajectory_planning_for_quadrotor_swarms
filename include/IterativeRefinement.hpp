#pragma once
#include "DiscreteSchedule.hpp"
#include "Environment.hpp"
#include "Graph.hpp"
#include "HyperPlane.hpp"
#include "BezierCurve.hpp"
#include "HyperPlaneSeparator.hpp"
#include "SafePolyhedron.hpp"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <cstddef>
#include <unistd.h>
#include <vector>

class IterativeRefinement{
public:
    const Graph& graph;
    const DiscreteSchedule& discreteSchedule;
    const RobotModel& robotModel;
    const Environment& environment;

    int D = 7; // Bezier curve degree
    int C = 4; // Continuity level
    std::vector<double> user_parameter = {1.0, 1.0, 1.0, 1.0}; // gamma_c weights

    IterativeRefinement(const Graph& graph, const DiscreteSchedule& discreteSchedule, const RobotModel& robotModel, const Environment& environment)
        : graph(graph), discreteSchedule(discreteSchedule), robotModel(robotModel), environment(environment) {}

    std::vector<std::vector<Eigen::Vector3d>> refine(double T_total, int num_iterations = 6) {
        int N = discreteSchedule.waypoint.size();
        std::vector<int> goal_pieces(N);
        for(int i=0; i < N; i++){
            // path.size()-1 = robotun goal'a ulaştığı timestep
            // O timestep'ten itibaren tüm piece'ler goal'da sabit kalmalı
            goal_pieces[i] = discreteSchedule.waypoint[i].size() - 1;
        }
        
        // iterasyon 0: HyperPlane kaynağı = discrete plan segmentleri
        HyperPlaneSeparator separator(graph, robotModel, discreteSchedule, environment);
        SafePolyhedron safe_poly = separator.compute();

        BezierCurve bezier(D, C, &environment);
        auto current_trajectories = bezier.compute(safe_poly, user_parameter,
                                                    discreteSchedule.K, T_total, goal_pieces);
        
        // Save initial control points (iteration 0)
        saveControlPointsToCSV(current_trajectories, "control_points_iter_0.csv");

        for (int iter = 1; iter < num_iterations; ++iter) {
            auto sampled_points = sampleTrajectories(current_trajectories);

            SafePolyhedron new_poly = separator.compute(sampled_points);

            SafePolyhedron combined_poly = intersectPolyhedra(new_poly, safe_poly);

            auto new_trajectories = bezier.compute(combined_poly, user_parameter,
                                                discreteSchedule.K, T_total, goal_pieces);

            current_trajectories = new_trajectories;
            
            // Save control points for current iteration
            saveControlPointsToCSV(current_trajectories, "control_points_iter_" + std::to_string(iter) + ".csv");
        }

        return current_trajectories;
    }

    SafePolyhedron intersectPolyhedra(const SafePolyhedron& a, const SafePolyhedron& b) {
        // a ve b'nin planes yapısı: planes[agent][timestep_k] → vector<HyperPlane>
        size_t num_agents = a.planes.size();
        size_t K = a.planes[0].size();

        std::vector<std::vector<std::vector<HyperPlane>>> combined(num_agents);
        for (size_t i = 0; i < num_agents; ++i) {
            combined[i].resize(K);
            for (size_t k = 0; k < K; ++k) {
                // a'nın tüm plane'leri
                combined[i][k] = a.planes[i][k];
                // b'nin plane'lerini ekle
                combined[i][k].insert(combined[i][k].end(),
                                      b.planes[i][k].begin(),
                                      b.planes[i][k].end());
            }
        }
        return SafePolyhedron(combined);
    }


    // Moved implementation from main.cpp
    void saveControlPointsToCSV(const std::vector<std::vector<Eigen::Vector3d>>& all_control_points,
                                const std::string& path) const {
        std::ofstream f(path);
        if (!f) {
            std::cerr << "HATA: Kontrol noktası dosyası açılamadı: " << path << std::endl;
            return;
        }
        f << "agent_id,piece_id,point_index_in_piece,x,y,z\n";

        for (size_t agent_id = 0; agent_id < all_control_points.size(); ++agent_id) {
            const auto& agent_control_points = all_control_points[agent_id];
            int points_per_piece = this->D + 1;
            for (size_t i = 0; i < agent_control_points.size(); ++i) {
                int piece_id = i / points_per_piece;
                int point_index_in_piece = i % points_per_piece;
                const auto& p = agent_control_points[i];
                f << agent_id << "," << piece_id << "," << point_index_in_piece << ","
                  << p.x() << "," << p.y() << "," << p.z() << "\n";
            }
        }
        std::cout << "Kaydedildi: " << path << "\n";
    }

private:
    // agent başına, timestep başına S nokta
    // current_trajectories[agent] → K*(D+1) control point listesi
    // k. piece için control pointler: indeks k*(D+1) ... k*(D+1)+D
    std::vector<std::vector<std::vector<Eigen::Vector3d>>> sampleTrajectories(const std::vector<std::vector<Eigen::Vector3d>>& trajectories) {  // control points
        int S = 32;
        int N = trajectories.size();
        // result[agent][k] → S nokta
        std::vector<std::vector<std::vector<Eigen::Vector3d>>> result(N); // result[agent][timestep_k][sample_point]
        
        for (int agent = 0; agent < N; ++agent) {
            result[agent].resize(discreteSchedule.K);
            for (int k = 0; k < discreteSchedule.K; ++k) {
                std::vector<Eigen::Vector3d> control_points(trajectories[agent].begin() + k*(D+1),
                                                trajectories[agent].begin() + k*(D+1) + D+1);
                for (int s = 0; s < S; ++s) {
                    double t = (double)s / S;
                    result[agent][k].push_back(sampleBezier(t, control_points));
                }
            }
        }
        return result;
    }

     // t_local: 0.0 ile 1.0 arasında (zaman yüzdesi)
    // control_points: O segmente ait 8 nokta (D=7 için)
    Eigen::Vector3d sampleBezier(double t_local, const std::vector<Eigen::Vector3d>& control_points) {
        std::vector<Eigen::Vector3d> temp = control_points;
        int n = temp.size();

        // De Casteljau döngüsü
        for (int j = 1; j < n; ++j) {
            for (int i = 0; i < n - j; ++i) {
                temp[i] = (1.0 - t_local) * temp[i] + t_local * temp[i + 1];
            }
        }
        return temp[0]; // Eğri üzerindeki nokta
    }
};
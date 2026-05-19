#include "IterativeRefinement.hpp"
#include "Environment.hpp"
#include "Graph.hpp"
#include "HyperPlane.hpp"
#include "BezierCurve.hpp"
#include "HyperPlaneSeparator.hpp"
#include "SafePolyhedron.hpp"
#include "SubdividedSchedule.hpp"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <fstream>
#include <iostream>
#include <cstddef>
#include <unistd.h>
#include <vector>


IterativeRefinement::IterativeRefinement(const Graph& graph, const SubdividedSchedule& subdividedSchedule, const RobotModel& robotModel, const Environment& environment)
    : graph(graph), subdividedSchedule(subdividedSchedule), robotModel(robotModel), environment(environment) {}

std::vector<std::vector<Eigen::Vector3d>> IterativeRefinement::refine(double T_total, int num_iterations) {
    int N = subdividedSchedule.positions.size();
    std::vector<int> goal_pieces(N);
    for(int i=0; i < N; i++){
        // path.size()-1 = robotun goal'a ulaştığı timestep
        // O timestep'ten itibaren tüm piece'ler goal'da sabit kalmalı
        goal_pieces[i] = subdividedSchedule.K;
        const auto& pos = subdividedSchedule.positions[i];
        Eigen::Vector3d goal_pos = pos.back();
        for(int k=0; k < subdividedSchedule.K; k++){
            if((pos[k] - goal_pos).norm() < 1e-3){
                goal_pieces[i] = k;
                break;
            }
        }
    }
    
    // iterasyon 0: HyperPlane kaynağı = discrete plan segmentleri
    HyperPlaneSeparator separator(graph, robotModel, subdividedSchedule, environment);
    
    // Iterasyon 0 hiperdüzlemleri
    SafePolyhedron current_poly = separator.compute();
    saveSafePolyhedronCSV(current_poly, "hyperplanes_iter_0.csv");

    BezierCurve bezier(D, C, &environment, subdividedSchedule);
    auto current_trajectories = bezier.compute(current_poly, user_parameter,
                                                subdividedSchedule.K, T_total, goal_pieces);
    
    // Save initial control points (iteration 0)
    saveControlPointsToCSV(current_trajectories, "control_points_iter_0.csv");

    for (int iter = 1; iter < num_iterations; ++iter) {
        auto sampled_points = sampleTrajectories(current_trajectories);

        current_poly = separator.compute(sampled_points); 

        saveSafePolyhedronCSV(current_poly, "hyperplanes_iter_" + std::to_string(iter) + ".csv");

        current_trajectories = bezier.compute(current_poly, user_parameter,
                                        subdividedSchedule.K, T_total, goal_pieces);

        // Save control points for current iteration
        saveControlPointsToCSV(current_trajectories, "control_points_iter_" + std::to_string(iter) + ".csv");
    }

    return current_trajectories;
}

void IterativeRefinement::saveSafePolyhedronCSV(const SafePolyhedron& poly, const std::string& path) const {
    std::ofstream f(path);
    if (!f) {
        std::cerr << "HATA: Güvenli Polyhedron dosyası açılamadı: " << path << std::endl;
        return;
    }
    f << "robot_id,timestep,nx,ny,nz,d,ellipsoid_offset,separated_from_type,separated_from_id\n";
    for (size_t i = 0; i < poly.planes.size(); ++i) {
        for (size_t k = 0; k < poly.planes[i].size(); ++k) {
            for (const auto& plane : poly.planes[i][k]) {
                f << i << "," << k << ","
                    << plane.normal_vector.x() << ","
                    << plane.normal_vector.y() << ","
                    << plane.normal_vector.z() << ","
                    << plane.d << ","
                    << plane.ellipsoid_offset << ","
                    << plane.separated_from_type << ","
                    << plane.separated_from_id << "\n";
            }
        }
    }
    std::cout << "Kaydedildi: " << path << "\n";
}

// Moved implementation from main.cpp
void IterativeRefinement::saveControlPointsToCSV(const std::vector<std::vector<Eigen::Vector3d>>& all_control_points,
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

double IterativeRefinement::computeScaledTime(
    const std::vector<std::vector<Eigen::Vector3d>>& control_points,
    double T_initial,
    int K,
    double max_acceleration,   // m/s^2
    double max_velocity){

    // OPTİMİZASYON: Başlangıç süresi (T_initial) zaten limitleri sağlıyorsa, 
    // boşuna binary search (ikili arama) yapmadan direkt mevcut süreyi döndür.
    if(checkLimits(control_points, T_initial, K, max_acceleration, max_velocity)) {
        return T_initial;
    }

    double T_low = T_initial;
    double T_high = T_initial * 20.0; // üst sınır

    while(!checkLimits(control_points, T_high, K, max_acceleration, max_velocity))
        T_high *= 2.0;

    for(int iter = 0; iter < 30; ++iter){
        double T_mid = (T_low + T_high) * 0.5;

        if(checkLimits(control_points, T_mid, K, max_acceleration, max_velocity))
            T_high = T_mid;
        else
            T_low = T_mid;
    }

    return T_high;
    
}

// agent başına, timestep başına S nokta
// current_trajectories[agent] → K*(D+1) control point listesi
// k. piece için control pointler: indeks k*(D+1) ... k*(D+1)+D
std::vector<std::vector<std::vector<Eigen::Vector3d>>> IterativeRefinement::sampleTrajectories(const std::vector<std::vector<Eigen::Vector3d>>& trajectories) {  // control points
    int S = 32;
    int N = trajectories.size();
    // result[agent][k] → S nokta
    std::vector<std::vector<std::vector<Eigen::Vector3d>>> result(N); // result[agent][timestep_k][sample_point]
    
    for (int agent = 0; agent < N; ++agent) {
        result[agent].resize(subdividedSchedule.K);
        for (int k = 0; k < subdividedSchedule.K; ++k) {
            std::vector<Eigen::Vector3d> control_points(trajectories[agent].begin() + k*(D+1),
                                            trajectories[agent].begin() + k*(D+1) + D+1);
            for (int s = 0; s < S; ++s) {
                double t = (double)s / (S - 1);  
                result[agent][k].push_back(sampleBezier(t, control_points));
            }
        }
    }
    return result;
}

    // t_local: 0.0 ile 1.0 arasında (zaman yüzdesi)
// control_points: O segmente ait 8 nokta (D=7 için)
Eigen::Vector3d IterativeRefinement::sampleBezier(double t_local, const std::vector<Eigen::Vector3d>& control_points) {
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

// belirli bir türev derecesi için sadece kontrol noktalarını döndürür
std::vector<Eigen::Vector3d> IterativeRefinement::getDerivativeControlPoints(const std::vector<Eigen::Vector3d>& cps, double T_piece, int c) {
    std::vector<Eigen::Vector3d> derivate_cps = cps;
    int deg = derivate_cps.size() - 1;

    for (int i = 0; i < c; i++) {
        std::vector<Eigen::Vector3d> next;
        for (int j = 0; j < (int)derivate_cps.size() - 1; j++) {
            next.push_back(deg * (derivate_cps[j+1] - derivate_cps[j]));
        }
        derivate_cps = next;
        deg--;
    }

    // Zaman ölçeklemesini kontrol noktalarına uygula
    double scale = std::pow(T_piece, c);
    for (auto& p : derivate_cps) {
        p /= scale;
    }
    return derivate_cps;
}

bool IterativeRefinement::checkLimits(
    const std::vector<std::vector<Eigen::Vector3d>>& control_points,
    double T_total,
    int K,
    double max_acc,
    double max_vel){

    double T_piece = T_total / K;

    for (const auto& agent_cps : control_points) {
        for (int k = 0; k < K; ++k) {
            std::vector<Eigen::Vector3d> cps(agent_cps.begin() + k * (D + 1), agent_cps.begin() + k * (D + 1) + D + 1);

            // Hız kontrol noktalarını al ve maksimum sınırı aşıp aşmadığını test et
            auto vel_cps = getDerivativeControlPoints(cps, T_piece, 1);
            for (const auto& v : vel_cps) {
                if (v.norm() > max_vel) return false;
            }

            // İvme kontrol noktalarını al ve maksimum sınırı aşıp aşmadığını test et
            auto acc_cps = getDerivativeControlPoints(cps, T_piece, 2);
            for (const auto& a : acc_cps) {
                if (a.norm() > max_acc) return false;
            }
        }
    }
    return true;
}

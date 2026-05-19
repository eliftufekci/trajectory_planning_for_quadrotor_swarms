#pragma once
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

class IterativeRefinement{
public:
    const Graph& graph;
    const SubdividedSchedule& subdividedSchedule;
    const RobotModel& robotModel;
    const Environment& environment;

    int D = 7; // Bezier curve degree
    int C = 4; // Continuity level
    std::vector<double> user_parameter = {1.0, 1.0, 1.0, 1.0}; // gamma_c weights

    IterativeRefinement(const Graph& graph, const SubdividedSchedule& subdividedSchedule, const RobotModel& robotModel, const Environment& environment);
    
    std::vector<std::vector<Eigen::Vector3d>> refine(double T_total, int num_iterations = 6);

    void saveSafePolyhedronCSV(const SafePolyhedron& poly, const std::string& path) const;

    // Moved implementation from main.cpp
    void saveControlPointsToCSV(const std::vector<std::vector<Eigen::Vector3d>>& all_control_points,
                                const std::string& path) const;

    double computeScaledTime(
        const std::vector<std::vector<Eigen::Vector3d>>& control_points,
        double T_initial,
        int K,
        double max_acceleration,   // m/s^2
        double max_velocity);

private:
    // agent başına, timestep başına S nokta
    // current_trajectories[agent] → K*(D+1) control point listesi
    // k. piece için control pointler: indeks k*(D+1) ... k*(D+1)+D
    std::vector<std::vector<std::vector<Eigen::Vector3d>>> sampleTrajectories(const std::vector<std::vector<Eigen::Vector3d>>& trajectories);

    // t_local: 0.0 ile 1.0 arasında (zaman yüzdesi)
    // control_points: O segmente ait 8 nokta (D=7 için)
    Eigen::Vector3d sampleBezier(double t_local, const std::vector<Eigen::Vector3d>& control_points);

    // belirli bir türev derecesi için sadece kontrol noktalarını döndürür
    std::vector<Eigen::Vector3d> getDerivativeControlPoints(const std::vector<Eigen::Vector3d>& cps, double T_piece, int c);

    bool checkLimits(
        const std::vector<std::vector<Eigen::Vector3d>>& control_points,
        double T_total,
        int K,
        double max_acc,
        double max_vel);
};
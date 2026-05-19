#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unistd.h>
#include <vector>
#include <algorithm>
#include <OsqpEigen/Solver.hpp>
#include "MAPFCTypes.hpp"

class Environment;
class Graph;
struct HyperPlane;
struct SafePolyhedron;
struct SubdividedSchedule;

struct BezierCurve{
    const int D; // Bezier curve degree
    const int C; // Continuity level (e.g., C0, C1, C2)

    const Environment* environment;
    const SubdividedSchedule& subdividedSchedule;

    std::map<int, long long> factorial_lookup; // Changed to long long to prevent overflow

    BezierCurve(int degree, int continuity_level, const Environment* env, const SubdividedSchedule& schedule);

    void fill_lookup_table();

    Eigen::MatrixXd get_sub_Q_mono_matrix(int c, double T_piece);

    Eigen::MatrixXd get_total_Q_mono_matrix(const std::vector<double>& user_parameter, int K, double T_piece);

    Eigen::MatrixXd get_B_matrix(double T_piece);

    Eigen::MatrixXd get_total_B_matrix(int K, double T_piece);

    double combinations(int n, int i);

    std::vector<Eigen::Vector3d> compute_control_points(const SafePolyhedron& safe_polyhedron,
                                                        int agent_id, // Use agent_id to get start/goal from environment
                                                        const std::vector<double>& user_parameter, // gamma_c weights
                                                        int K, // number of pieces
                                                        double T_total,
                                                        int goal_piece);

    std::vector<std::vector<Eigen::Vector3d>> compute(const SafePolyhedron& safe_polyhedron,
                                                      const std::vector<double>& user_parameter, // gamma_c weights
                                                      int K, // number of pieces
                                                      double T_total,
                                                      const std::vector<int>& goal_pieces);

    std::vector<Eigen::Vector3d> linear_fallback(const Eigen::Vector3d& start_pos, 
                                                 const Eigen::Vector3d& goal_pos, 
                                                 int agent_id,
                                                 int K) const;
};
#pragma once
#include "DiscreteSchedule.hpp"
#include "Environment.hpp"
#include "Graph.hpp"
#include "HyperPlane.hpp"
#include "MAPFCTypes.hpp"
#include "SafePolyhedron.hpp"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <vector>
#include <algorithm>
#include <OsqpEigen/Solver.hpp>

struct BezierCurve{
    const int D; // Bezier curve degree
    const int C; // Continuity level (e.g., C0, C1, C2)

    const Environment* env_ptr; // Pointer to the environment for agent start/goal

    std::map<int, long long> factorial_lookup; // Changed to long long to prevent overflow

    BezierCurve(int degree, int continuity_level, const Environment* env)
        : D(degree), C(continuity_level), env_ptr(env) {
        fill_lookup_table();
    }

    void fill_lookup_table(){
        factorial_lookup[0] = 1;
        for(int i=1; i <= D+1; i++){
            long long value = 1; // Use long long to avoid overflow for larger D
            for(int j=1; j <= i; j++){
                value *= j;
            }
            factorial_lookup[i] = value;
        }
    }

    // Calculates the Q matrix for a single piece for a single derivative 'c'.
    // This is for the monomial basis p(t) = a0 + a1*t + ... on [0, T_piece].
    Eigen::MatrixXd get_sub_Q_mono_matrix(int c, double T_piece){
        Eigen::MatrixXd Q(D+1, D+1);
        Q.setZero();
        
        for(int n=0; n < D+1; n++){
            for(int m=0; m < D+1; m++){
                if(n >= c && m >= c){
                    double first_initial = (double)(factorial_lookup[n] * factorial_lookup[m]) / (factorial_lookup[n-c] * factorial_lookup[m-c]);
                    double second_initial = std::pow(T_piece, n+m-2*c+1) / (n+m-2*c+1);
                    double value = first_initial * second_initial;
                    Q(n,m) = value;
                }
            }
        }
        return Q;
    }

    // Builds the block-diagonal Q matrix for all K pieces in the monomial basis.
    Eigen::MatrixXd get_total_Q_mono_matrix(const std::vector<double>& user_parameter, int K, double T_piece){
        Eigen::MatrixXd Q_total(K*(D+1), K*(D+1));
        Q_total.setZero();

        Eigen::MatrixXd Q_piece(D+1, D+1);
        Q_piece.setZero();
        for(size_t c = 1; c <= user_parameter.size(); c++){
            if (c <= C && user_parameter.at(c-1) > 1e-9) {
                 Q_piece += user_parameter.at(c-1) * get_sub_Q_mono_matrix(c, T_piece);
            }
        }

        for(int i = 0; i < K; i++){
            Q_total.block(i*(D+1), i*(D+1), D+1, D+1) = Q_piece;
        }

        return Q_total;
    }

    // Matrix to convert Bezier control points to monomial coefficients for one piece on [0, T_piece].
    // p = B * c, where p are monomial coeffs, c are bezier control points.
    Eigen::MatrixXd get_B_matrix(double T_piece) {
        Eigen::MatrixXd M(D + 1, D + 1);
        M.setZero();
        for (int j = 0; j <= D; ++j) { // row, monomial exponent
            for (int i = 0; i <= D; ++i) { // column, control point index
                if(j >= i){
                    M(j, i) = std::pow(-1, j - i) * combinations(D, i) * combinations(D - i, j - i);
                    M(j, i) /= std::pow(T_piece, j);
                }
            }
        }
        return M;
    }

    // Builds the full block-diagonal conversion matrix for K pieces.
    Eigen::MatrixXd get_total_B_matrix(int K, double T_piece){
        Eigen::MatrixXd B_total(K*(D+1), K*(D+1));
        B_total.setZero();
        Eigen::MatrixXd B_piece = get_B_matrix(T_piece);
        
        for(int i = 0; i < K; i++){
            B_total.block(i*(D+1), i*(D+1), D+1, D+1) = B_piece;
        }

        return B_total;
    }

    double combinations(int n, int i){
        if (n < i || i < 0) return 0;
        return (double)factorial_lookup[n] / (factorial_lookup[i] * factorial_lookup[n-i]);
    }

    std::vector<Eigen::Vector3d> QPSolver(
        const SafePolyhedron& safe_polyhedron,
        int agent_id, // Use agent_id to get start/goal from env_ptr
        const std::vector<double>& user_parameter, // gamma_c weights
        int K, // number of pieces
        double T_total // total time
    ){
        double T_piece = T_total / K;

        const int n_vars_per_dim = K * (D + 1);
        const int n_vars = 3 * n_vars_per_dim;

        if (!env_ptr) {
            throw std::runtime_error("Environment pointer is null in BezierCurve::QPSolver.");
        }

        const Eigen::Vector3d& start_pos = env_ptr->agents[agent_id].start;
        const Eigen::Vector3d& goal_pos = env_ptr->agents[agent_id].goal;

        // --- 1. Hessian Matrisi (P) ---
        // Cost = y^T * P * y
        // P = B^T * Q_mono * B, where B is Bezier-to-Monomial conversion
        Eigen::MatrixXd Q_mono_1d = get_total_Q_mono_matrix(user_parameter, K, T_piece);
        Eigen::MatrixXd B_1d = get_total_B_matrix(K, T_piece);
        Eigen::MatrixXd P_1d = B_1d.transpose() * Q_mono_1d * B_1d;

        Eigen::SparseMatrix<double> P(n_vars, n_vars);
        std::vector<Eigen::Triplet<double>> P_triplets;
        for(int i=0; i<n_vars_per_dim; ++i) {
            for(int j=0; j<n_vars_per_dim; ++j) {
                if (std::abs(P_1d(i,j)) > 1e-9) {
                    P_triplets.emplace_back(i, j, P_1d(i,j)); // X
                    P_triplets.emplace_back(i + n_vars_per_dim, j + n_vars_per_dim, P_1d(i,j)); // Y
                    P_triplets.emplace_back(i + 2*n_vars_per_dim, j + 2*n_vars_per_dim, P_1d(i,j)); // Z
                }
            }
        }
        P.setFromTriplets(P_triplets.begin(), P_triplets.end());

        // --- 2. Gradyan Vektörü (q) ---
        Eigen::VectorXd q = Eigen::VectorXd::Zero(n_vars);

        // --- 3. Kısıtlar (A, l, u) ---
        std::vector<Eigen::Triplet<double>> A_triplets;
        std::vector<double> l_vec, u_vec;
        int constraint_idx = 0;

        // a. Corridor Constraints: y_k,d in P_k
        const double inf = std::numeric_limits<double>::infinity();
        for (int k = 0; k < K; ++k) {
            for (int d = 0; d <= D; ++d) {
                int cp_idx = k * (D + 1) + d;
                // Use the correct agent_id for the safe polyhedron
                for (const auto& hyperplane : safe_polyhedron.planes[agent_id][k]) {
                    A_triplets.emplace_back(constraint_idx, cp_idx, hyperplane.normal_vector.x());
                    A_triplets.emplace_back(constraint_idx, cp_idx + n_vars_per_dim, hyperplane.normal_vector.y());
                    A_triplets.emplace_back(constraint_idx, cp_idx + 2*n_vars_per_dim, hyperplane.normal_vector.z());
                    l_vec.push_back(hyperplane.d + hyperplane.ellipsoid_offset);
                    u_vec.push_back(inf);
                    constraint_idx++;
                }
            }
        }

        // b. Start/Goal Position Constraints
        // Start pos: y_0,0 = start_pos
        A_triplets.emplace_back(constraint_idx, 0, 1.0); l_vec.push_back(start_pos.x()); u_vec.push_back(start_pos.x()); constraint_idx++;
        A_triplets.emplace_back(constraint_idx, n_vars_per_dim, 1.0); l_vec.push_back(start_pos.y()); u_vec.push_back(start_pos.y()); constraint_idx++;
        A_triplets.emplace_back(constraint_idx, 2*n_vars_per_dim, 1.0); l_vec.push_back(start_pos.z()); u_vec.push_back(start_pos.z()); constraint_idx++;

        // Goal pos: y_K-1,D = goal_pos
        int last_cp_idx = n_vars_per_dim - 1;
        A_triplets.emplace_back(constraint_idx, last_cp_idx, 1.0); l_vec.push_back(goal_pos.x());  u_vec.push_back(goal_pos.x()); constraint_idx++;
        A_triplets.emplace_back(constraint_idx, last_cp_idx + n_vars_per_dim, 1.0);  l_vec.push_back(goal_pos.y());  u_vec.push_back(goal_pos.y());  constraint_idx++;
        A_triplets.emplace_back(constraint_idx, last_cp_idx + 2*n_vars_per_dim, 1.0);  l_vec.push_back(goal_pos.z());  u_vec.push_back(goal_pos.z());  constraint_idx++;

        // c. Continuity Constraints up to derivative C
        for (int k = 1; k < K; ++k) { // for each junction
            // C0: y_{k-1,D} = y_{k,0}
            int cp_prev_idx = (k - 1) * (D + 1) + D;
            int cp_curr_idx = k * (D + 1);
            A_triplets.emplace_back(constraint_idx, cp_prev_idx, 1.0); A_triplets.emplace_back(constraint_idx, cp_curr_idx, -1.0); l_vec.push_back(0); u_vec.push_back(0); constraint_idx++;
            A_triplets.emplace_back(constraint_idx, cp_prev_idx + n_vars_per_dim, 1.0); A_triplets.emplace_back(constraint_idx, cp_curr_idx + n_vars_per_dim, -1.0); l_vec.push_back(0); u_vec.push_back(0); constraint_idx++;
            A_triplets.emplace_back(constraint_idx, cp_prev_idx + 2*n_vars_per_dim, 1.0); A_triplets.emplace_back(constraint_idx, cp_curr_idx + 2*n_vars_per_dim, -1.0); l_vec.push_back(0); u_vec.push_back(0); constraint_idx++;

            // C1: y_{k,1} - y_{k,0} = y_{k-1,D} - y_{k-1,D-1}
            if (C >= 1) {
                int ykD_idx = (k-1)*(D+1) + D;
                int ykD_1_idx = (k-1)*(D+1) + D - 1;
                int yk1_idx = k*(D+1) + 1;
                int yk0_idx = k*(D+1);
                // yk1 - yk0 - (ykD - ykD_1) = 0
                A_triplets.emplace_back(constraint_idx, yk1_idx, 1.0); A_triplets.emplace_back(constraint_idx, yk0_idx, -1.0); A_triplets.emplace_back(constraint_idx, ykD_idx, -1.0); A_triplets.emplace_back(constraint_idx, ykD_1_idx, 1.0); l_vec.push_back(0); u_vec.push_back(0); constraint_idx++;
                A_triplets.emplace_back(constraint_idx, yk1_idx+n_vars_per_dim, 1.0); A_triplets.emplace_back(constraint_idx, yk0_idx+n_vars_per_dim, -1.0); A_triplets.emplace_back(constraint_idx, ykD_idx+n_vars_per_dim, -1.0); A_triplets.emplace_back(constraint_idx, ykD_1_idx+n_vars_per_dim, 1.0); l_vec.push_back(0); u_vec.push_back(0); constraint_idx++;
                A_triplets.emplace_back(constraint_idx, yk1_idx+2*n_vars_per_dim, 1.0); A_triplets.emplace_back(constraint_idx, yk0_idx+2*n_vars_per_dim, -1.0); A_triplets.emplace_back(constraint_idx, ykD_idx+2*n_vars_per_dim, -1.0); A_triplets.emplace_back(constraint_idx, ykD_1_idx+2*n_vars_per_dim, 1.0); l_vec.push_back(0); u_vec.push_back(0); constraint_idx++;
            }
            // Higher order continuity can be added here...
        }

        // d. Boundary Derivative Constraints (vel, acc, ... = 0 at start and end)
        for (int c = 1; c <= C; ++c) {
            // At t=0, c-th derivative is 0. This means Delta^c y_0 = 0.
            // For c=1: y_1 - y_0 = 0
            if (c==1) {
                A_triplets.emplace_back(constraint_idx, 1, 1.0); A_triplets.emplace_back(constraint_idx, 0, -1.0); l_vec.push_back(0); u_vec.push_back(0); constraint_idx++;
                A_triplets.emplace_back(constraint_idx, 1+n_vars_per_dim, 1.0); A_triplets.emplace_back(constraint_idx, 0+n_vars_per_dim, -1.0); l_vec.push_back(0); u_vec.push_back(0); constraint_idx++;
                A_triplets.emplace_back(constraint_idx, 1+2*n_vars_per_dim, 1.0); A_triplets.emplace_back(constraint_idx, 0+2*n_vars_per_dim, -1.0); l_vec.push_back(0); u_vec.push_back(0); constraint_idx++;
            }
            // At t=T, c-th derivative is 0. This means Delta^c y_{D-c} = 0.
            // For c=1: y_D - y_{D-1} = 0
            if (c==1) {
                int cpD_idx = n_vars_per_dim - 1;
                int cpD_1_idx = n_vars_per_dim - 2;
                A_triplets.emplace_back(constraint_idx, cpD_idx, 1.0); A_triplets.emplace_back(constraint_idx, cpD_1_idx, -1.0); l_vec.push_back(0); u_vec.push_back(0); constraint_idx++;
                A_triplets.emplace_back(constraint_idx, cpD_idx+n_vars_per_dim, 1.0); A_triplets.emplace_back(constraint_idx, cpD_1_idx+n_vars_per_dim, -1.0); l_vec.push_back(0); u_vec.push_back(0); constraint_idx++;
                A_triplets.emplace_back(constraint_idx, cpD_idx+2*n_vars_per_dim, 1.0); A_triplets.emplace_back(constraint_idx, cpD_1_idx+2*n_vars_per_dim, -1.0); l_vec.push_back(0); u_vec.push_back(0); constraint_idx++;
            }
        }

        const int n_constraints = constraint_idx;
        Eigen::SparseMatrix<double> A(n_constraints, n_vars);
        A.setFromTriplets(A_triplets.begin(), A_triplets.end());

        Eigen::VectorXd l = Eigen::Map<Eigen::VectorXd>(l_vec.data(), l_vec.size());
        Eigen::VectorXd u = Eigen::Map<Eigen::VectorXd>(u_vec.data(), u_vec.size());

        // --- 5. Solver'ı kur ve çöz ---
        OsqpEigen::Solver solver;
        solver.settings()->setVerbosity(false);
        solver.data()->setNumberOfVariables(n_vars);
        solver.data()->setNumberOfConstraints(n_constraints);

        if (!solver.data()->setHessianMatrix(P) || !solver.data()->setGradient(q) ||
            !solver.data()->setLinearConstraintsMatrix(A) ||
            !solver.data()->setLowerBound(l) || !solver.data()->setUpperBound(u)) {
            throw std::runtime_error("QP setup failed");
        }
                                
        if (!solver.initSolver()) throw std::runtime_error("QP init failed");
        
        auto solver_status = solver.solveProblem();

        std::vector<Eigen::Vector3d> control_points;
        if (solver_status == OsqpEigen::ErrorExitFlag::NoError) {
            Eigen::VectorXd solution = solver.getSolution();
            control_points.resize(K * (D + 1));
            for (int i = 0; i < n_vars_per_dim; ++i) {
                control_points[i] = Eigen::Vector3d(
                    solution(i),
                    solution(i + n_vars_per_dim),
                    solution(i + 2 * n_vars_per_dim)
                );
            }
        } else {
            // Fallback or error handling
            throw std::runtime_error("QP could not be solved.");
        }

        return control_points;
    }
    }
};
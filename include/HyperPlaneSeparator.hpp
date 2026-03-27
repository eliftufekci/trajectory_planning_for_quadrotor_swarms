#pragma once
#include "DiscreteSchedule.hpp"
#include "Graph.hpp"
#include "HyperPlane.hpp"
#include "MAPFCTypes.hpp"
#include "TrajectoryOptimizationTypes.hpp"
#include <Eigen/Dense>
#include <eigen3/Eigen/src/Core/DiagonalMatrix.h>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <eigen3/Eigen/src/Core/util/Constants.h>
#include <vector>

class HyperPlaneSeparator{
public:
    
    HyperPlaneSeparator(const Graph& graph, const RobotModel& robotModel, const DiscreteSchedule& discreteSchedule)
        : graph(graph), discreteSchedule(discreteSchedule), robotModel(robotModel) {}

    // Tüm schedule için tüm (i,j,k) hyperplane'lerini hesapla
    SafePolyhedron compute(){
        const auto& waypoint = discreteSchedule.waypoint;
        std::vector<std::vector<std::vector<HyperPlane>>> planes(waypoint.size());
        for(size_t i = 0; i < waypoint.size(); ++i) {
            planes[i].resize(discreteSchedule.K);
        }

        // Her `(i, j, k)` üçlüsü için bir `(n, d)` çifti: robot `i` timestep `k`'da `n^T x >= d` tarafında, robot `j` `n^T x <= d` tarafında kalacak.
        for(int k=0; k < discreteSchedule.K-1; k++){
            for(int i=0; i < waypoint.size(); i++){ // robot i
                for(int j=i+1; j< waypoint.size(); j++){ // robot j                    
                    Eigen::MatrixXd segment_i(3, 2);
                    segment_i << graph.getVertex(waypoint[i][k]).pos, graph.getVertex(waypoint[i][k+1]).pos;
                    
                    Eigen::MatrixXd segment_j(3, 2);
                    segment_j << graph.getVertex(waypoint[j][k]).pos, graph.getVertex(waypoint[j][k+1]).pos;

                    HyperPlane plane = computeSVM(segment_i, segment_j);
                    planes[i][k].push_back(plane);
                    // Karşıt düzlemi de diğer ajan için ekle
                    planes[j][k].push_back(HyperPlane(-plane.normal_vector, -plane.d, plane.ellipsoid_offset));
                    
                }
            }
        }

        for(int k=0; k < discreteSchedule.K-1; k++){
            for(int i=0; i < waypoint.size(); i++){ // robot i
                for(auto& obstacle : environment.obstacles){
                    Eigen::MatrixXd segment_i(3, 2);
                        segment_i << graph.getVertex(waypoint[i][k]).pos, graph.getVertex(waypoint[i][k+1]).pos;
                    
                    Eigen::MatrixXd obstacle_matrix = getObstacleCorners(obstacle.min, obstacle.max);
                    
                    HyperPlane plane = computeSVM(segment_i, obstacle_matrix);
                    planes[i][k].push_back(plane);
                }
            }
        }

        return SafePolyhedron(planes);
    }


private:
    const Graph& graph;
    const DiscreteSchedule& discreteSchedule;
    const RobotModel& robotModel;
    const Environment& environment;

    std::tuple<Eigen::Vector3d, Eigen::Vector3d> find_closest_points(const Eigen::MatrixXd& s1, const Eigen::MatrixXd& s2) {
        Eigen::Vector3d p1 = s1.col(0);
        Eigen::Vector3d p2 = s1.col(1);
        Eigen::Vector3d p3 = s2.col(0);
        Eigen::Vector3d p4 = s2.col(1);
        
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
    
    bool sweptVolumesCollide(const Eigen::MatrixXd& segment_i, const Eigen::MatrixXd& segment_j) {
        auto [p_i, p_j] = find_closest_points(segment_i, segment_j);
        return robotModel.collides(p_i, p_j);
    }

    Eigen::MatrixXd getObstacleCorners(Eigen::Vector3d min, Eigen::Vector3d max){
        Eigen::MatrixXd corners(3, 8);
        
        corners.col(0) << min.x(), min.y(), min.z();
        corners.col(1) << max.x(), min.y(), min.z();
        corners.col(2) << min.x(), max.y(), min.z();
        corners.col(3) << max.x(), max.y(), min.z();
        
        corners.col(4) << min.x(), min.y(), max.z();
        corners.col(5) << max.x(), min.y(), max.z();
        corners.col(6) << min.x(), max.y(), max.z();
        corners.col(7) << max.x(), max.y(), max.z();
        
        return corners;
    }
    
    // İki nokta kümesini (burada iki segmentin uç noktaları) ayıran
    // maksimum marjlı hiperdüzlemi bulan SVM (QP) çözücü.    
    HyperPlane computeSVM( Eigen::MatrixXd segment_i, Eigen::MatrixXd segment_j){

        // QP Problemi:
        // minimize   0.5 * n'.n
        // subject to n'.p_i - d >= 1   (for p_i in segment_i endpoints)
        //            n'.p_j - d <= -1  (for p_j in segment_j endpoints)
        //
        // Değişkenler (x): [n_x, n_y, n_z, d] (n=4)
        // Kısıtlar (m): 4 tane (her segmentin iki ucu için birer tane)

        int n_vars = 4;
        int n_constraints = 4;

        // 1. Hessian Matrisi (P): minimize 0.5 * x'Px
        // Sadece normal vektörün (ilk 3 değişken) karesini minimize ediyoruz.
        Eigen::SparseMatrix<double> P(n_vars, n_vars);
        P.insert(0, 0) = robotModel.rx * robotModel.rx;
        P.insert(1, 1) = robotModel.ry * robotModel.ry;
        P.insert(2, 2) = robotModel.rz * robotModel.rz;
        P.insert(3, 3) = 0; // Beta için 0 bıraktık

        // 2. Gradyan Vektörü (q): q'x terimi yok.
        Eigen::VectorXd q = Eigen::VectorXd::Zero(n_vars);

        // 3. Kısıt Matrisi (A): l <= Ax <= u
        // Her satır bir kısıtı temsil eder: p'.n - d
        Eigen::MatrixXd A_dense(n_constraints, n_vars);
        A_dense.row(0) << segment_i.col(0).transpose(), -1;
        A_dense.row(1) << segment_i.col(1).transpose(), -1;
        A_dense.row(2) << segment_j.col(0).transpose(), -1;
        A_dense.row(3) << segment_j.col(1).transpose(), -1; 

        Eigen::SparseMatrix<double> A = A_dense.sparseView();


        // 4. Kısıt Sınırları (l ve u)
        // Ajan i'nin noktaları için: Alt sınır (l) = 1, Üst sınır (u) = sonsuz
        // Ajan j'nin noktaları için: Alt sınır (l) = -sonsuz, Üst sınır (u) = -1
        Eigen::VectorXd l(n_constraints), u(n_constraints);
        const double inf = std::numeric_limits<double>::infinity();
        l << 1.0, 1.0, -inf, -inf;
        u << inf, inf, -1.0, -1.0;

        
        // Solver ayarlama
        OsqpEigen::Solver solver;
        solver.settings()->setVerbosity(false);
        solver.data()->setNumberOfVariables(n_vars);
        solver.data()->setNumberOfConstraints(n_constraints);

        if (!solver.data()->setHessianMatrix(P)) throw std::runtime_error("SVM QP setup failed: P");
        if (!solver.data()->setGradient(q)) throw std::runtime_error("SVM QP setup failed: q");
        if (!solver.data()->setLinearConstraintsMatrix(A)) throw std::runtime_error("SVM QP setup failed: A");
        if (!solver.data()->setLowerBound(l)) throw std::runtime_error("SVM QP setup failed: l");
        if (!solver.data()->setUpperBound(u)) throw std::runtime_error("SVM QP setup failed: u");
                                
        if (!solver.initSolver()) throw std::runtime_error("SVM QP init failed");
        solver.solveProblem();

        Eigen::VectorXd solution = solver.getSolution();

        Eigen::Vector3d normal = solution.head<3>();
        double d_val = solution(3);

        // Hiperdüzlem denklemi: normal' * x = d_val
        // HyperPlane struct'ı normalize edilmiş normal bekliyor.
        // Eğer normal'i normalize edersek, d'yi de aynı oranda ölçeklemeliyiz.
        double norm_val = normal.norm();
        double ellipsoid_offset = calculate_offset(normal / norm_val);
        
        return HyperPlane(normal / norm_val, d_val / norm_val, ellipsoid_offset);
    }

    double calculate_offset(Eigen::Vector3d normal_vector){
        Eigen::Vector3d E_n(robotModel.rx * normalized_n.x(),
                        robotModel.ry * normalized_n.y(),
                        robotModel.rz * normalized_n.z());
        return E_n.norm();

    }
};
#pragma once
#include "DiscreteSchedule.hpp"
#include "Environment.hpp"
#include "Graph.hpp"
#include "HyperPlane.hpp"
#include "MAPFCTypes.hpp"
#include "SafePolyhedron.hpp"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <OsqpEigen/Solver.hpp>

class HyperPlaneSeparator{
public:
    
    HyperPlaneSeparator(const Graph& graph, const RobotModel& robotModel, const DiscreteSchedule& discreteSchedule, const Environment& environment)
        : graph(graph), discreteSchedule(discreteSchedule), robotModel(robotModel), environment(environment) {}

    // Tüm schedule için tüm (i,j,k) hyperplane'lerini hesapla
    SafePolyhedron compute(){
        const auto& waypoint = discreteSchedule.waypoint;
        std::vector<std::vector<std::vector<HyperPlane>>> planes(waypoint.size());
        for(size_t i = 0; i < waypoint.size(); ++i) {
            planes[i].resize(discreteSchedule.K);
        }

        // Her `(i, j, k)` üçlüsü için bir `(n, d)` çifti: robot `i` timestep `k`'da `n^T x >= d` tarafında, robot `j` `n^T x <= d` tarafında kalacak.
        for(int k=0; k < discreteSchedule.K; k++){
            for(int i=0; i < waypoint.size(); i++){ // robot i
                for(int j=i+1; j< waypoint.size(); j++){ // robot j                    
                    Eigen::MatrixXd segment_i(3, 2);
                    segment_i << graph.getVertex(getWaypoint(i, k)).pos, graph.getVertex(getWaypoint(i, k+1)).pos;
                    
                    Eigen::MatrixXd segment_j(3, 2);
                    segment_j << graph.getVertex(getWaypoint(j, k)).pos, graph.getVertex(getWaypoint(j, k+1)).pos;

                    HyperPlane plane = computeSVM(segment_i, segment_j);
                    planes[i][k].push_back(plane);
                    // Karşıt düzlemi de diğer ajan için ekle
                    planes[j][k].push_back(HyperPlane(-plane.normal_vector, -plane.d, plane.ellipsoid_offset));
                }
            }
        }

        for(int k=0; k < discreteSchedule.K; k++){
            for(int i=0; i < waypoint.size(); i++){ // robot i
                for(auto& obstacle : environment.obstacles){
                    Eigen::MatrixXd segment_i(3, 2);
                    segment_i << graph.getVertex(getWaypoint(i, k)).pos, graph.getVertex(getWaypoint(i, k+1)).pos;
                    
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

    int getWaypoint(size_t agent_idx, int time) const {
        const auto& path = discreteSchedule.waypoint[agent_idx];
        if (time < path.size()) {
            return path[time];
        }
        // Ajan hedefine ulaştıysa, son konumunda bekler.
        return path.back();
    }

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
    HyperPlane computeSVM( const Eigen::MatrixXd& points_i, const Eigen::MatrixXd& points_j){
        const int m_i = points_i.cols();
        const int m_j = points_j.cols();
        const int n_vars = 4; // [nx, ny, nz, d]
        const int n_constraints = m_i + m_j;

        bool is_obstacle = (points_j.cols() > 2);

        // --- 1. Hessian Matrisi (P): minimize 0.5 * x'Px ---
        Eigen::SparseMatrix<double> P(n_vars, n_vars);
        if(is_obstacle){
            // Engel durumunda, robotun yarıçapını kullanarak marjı maksimize et
            P.insert(0, 0) = robotModel.radius * robotModel.radius;
            P.insert(1, 1) = robotModel.radius * robotModel.radius;
            P.insert(2, 2) = robotModel.radius * robotModel.radius;
        } else {
            // Robot-robot durumunda, elipsoid yarıçaplarını kullan
            P.insert(0, 0) = robotModel.rx * robotModel.rx;
            P.insert(1, 1) = robotModel.ry * robotModel.ry;
            P.insert(2, 2) = robotModel.rz * robotModel.rz;
        }
        // P.insert(3, 3) = 0; // d'nin karesi minimize edilmiyor

        // --- 2. Gradyan Vektörü (q): q'x terimi yok ---
        Eigen::VectorXd q = Eigen::VectorXd::Zero(n_vars);

        // --- 3. Kısıt Matrisi (A): l <= Ax <= u ---
        Eigen::MatrixXd A_dense(n_constraints, n_vars);
        // points_i için kısıtlar: n'.p_i - d >= 1
        for (int i = 0; i < m_i; ++i) {
            A_dense.row(i) << points_i.col(i).transpose(), -1;
        }
        // points_j için kısıtlar: n'.p_j - d <= -1
        for (int j = 0; j < m_j; ++j) {
            A_dense.row(m_i + j) << points_j.col(j).transpose(), -1;
        }
        Eigen::SparseMatrix<double> A = A_dense.sparseView();

        // --- 4. Kısıt Sınırları (l ve u) ---
        Eigen::VectorXd l(n_constraints), u(n_constraints);
        const double inf = std::numeric_limits<double>::infinity();
        
        // points_i için: l=1, u=inf
        l.head(m_i).fill(1.0);
        u.head(m_i).fill(inf);

        // points_j için: l=-inf, u=-1
        l.tail(m_j).fill(-inf);
        u.tail(m_j).fill(-1.0);

        // --- 5. Solver'ı kur ve çöz ---
        OsqpEigen::Solver solver;
        solver.settings()->setVerbosity(false);
        solver.data()->setNumberOfVariables(n_vars);
        solver.data()->setNumberOfConstraints(n_constraints);

        if (!solver.data()->setHessianMatrix(P) || !solver.data()->setGradient(q) ||
            !solver.data()->setLinearConstraintsMatrix(A) ||
            !solver.data()->setLowerBound(l) || !solver.data()->setUpperBound(u)) {
            throw std::runtime_error("SVM QP setup failed");
        }
                                
        if (!solver.initSolver()) throw std::runtime_error("SVM QP init failed");
        solver.solveProblem();

        // --- 6. Sonucu işle ---
        Eigen::VectorXd solution = solver.getSolution();
        Eigen::Vector3d normal = solution.head<3>();
        double d_val = solution(3);

        double norm_val = normal.norm();
        if (norm_val < 1e-6) {
            // Çözüm başarısız veya normal vektör sıfır. Bu durum ayrılabilir olmadıklarında olabilir.
            // Geçici bir çözüm olarak, merkezleri birleştiren vektörü kullanabiliriz.
            normal = (points_i.rowwise().mean() - points_j.rowwise().mean()).normalized();
            d_val = normal.dot((points_i.rowwise().mean() + points_j.rowwise().mean()) * 0.5);
            norm_val = 1.0;
        }

        double ellipsoid_offset = calculate_offset(normal / norm_val);
        return HyperPlane(normal / norm_val, d_val / norm_val, ellipsoid_offset);
    }

    double calculate_offset(Eigen::Vector3d normal_vector){
        Eigen::Vector3d E_n(robotModel.rx * normal_vector.x(),
                        robotModel.ry * normal_vector.y(),
                        robotModel.rz * normal_vector.z());
        return E_n.norm();

    }
};
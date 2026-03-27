#pragma once
#include "DiscreteSchedule.hpp"
#include "Graph.hpp"
#include "HyperPlane.hpp"
#include "MAPFCTypes.hpp"
#include "TrajectoryOptimizationTypes.hpp"
#include <Eigen/Dense>
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

                    if(isCollide(segment_i, segment_j)){
                        HyperPlane plane = computeSVM(segment_i, segment_j);
                        planes[i][k].push_back(plane);
                        // Karşıt düzlemi de diğer ajan için ekle
                        planes[j][k].push_back(HyperPlane(-plane.normal_vector, -plane.d));
                    }
                }
            }
        }

        return SafePolyhedron(planes);
    }


private:
    const Graph& graph;
    const DiscreteSchedule& discreteSchedule;
    const RobotModel& robotModel;
    
    // TODO
    bool isCollide(Eigen::Vector3d p, Eigen::Vector3d q){
        Eigen::Vector3d diff = E_diag * (p - q);
        return diff.norm() < 2.0;
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
        Eigen::MatrixXd A(n_constraints, n_vars);
        A.row(0) << segment_i.col(0).transpose(), -1;
        A.row(1) << segment_i.col(1).transpose(), -1;
        A.row(2) << segment_j.col(0).transpose(), -1;
        A.row(3) << segment_j.col(1).transpose(), -1; 

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
        solver.solveProblem()

        Eigen::VectorXd solution = solver.getSolution();

        Eigen::Vector3d normal = solution.head<3>();
        double d_val = solution(3);

        // Hiperdüzlem denklemi: normal' * x = d_val
        // HyperPlane struct'ı normalize edilmiş normal bekliyor.
        // Eğer normal'i normalize edersek, d'yi de aynı oranda ölçeklemeliyiz.
        double norm_val = normal.norm();
        
        return HyperPlane(normal / norm_val, d_val / norm_val);
    }
};
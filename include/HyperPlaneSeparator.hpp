#pragma once
#include "DiscreteSchedule.hpp"
#include "Environment.hpp"
#include "Graph.hpp"
#include "HyperPlane.hpp"
#include "MAPFCTypes.hpp"
#include "TrajectoryOptimizationTypes.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <vector>

class HyperPlaneSeparator{
public:
    
    HyperplaneSeparator(const Graph& graph, const Eigen::DiagonalMatrix<double, 3>& E_diag, const DiscreteSchedule& discreteSchedule)
        : graph(graph), E_diag(E_diag), discreteSchedule(discreteSchedule)  {} 

    // Tüm schedule için tüm (i,j,k) hyperplane'lerini hesapla
    SafePolyhedron compute(const DiscreteSchedule&, const Graph&){
        std::vector<HyperPlane> planes;

        // Her `(i, j, k)` üçlüsü için bir `(n, d)` çifti: robot `i` timestep `k`'da `n^T x >= d` tarafında, robot `j` `n^T x <= d` tarafında kalacak.
        for(int k=0; k < discreteSchedule.K; k++){
            for(int i=0; i < discreteSchedule.waypoint.size(); i++){
                for(int j=i+1; j< discreteSchedule.waypoint.size(); j++){
                    auto& point_i = graph.getVertex(discreteSchedule.waypoint[i][k]).pos;
                    auto& point_j = graph.getVertex(discreteSchedule.waypoint[j][k]).pos;

                    if(isCollide(point_i, point_j)){
                        planes.push_back(computePair(point_i, point_j, i, j, k))
                    }
                }
            }
        }

        return SafePolyhedron(planes);
    }


private:
    const Graph& graph;
    const DiscreteSchedule& discreteSchedule;
    const Eigen::DiagonalMatrix<double, 3>& E_diag;  // diag(1/rx, 1/ry, 1/rz)
    
    // ||E^{-1}(p_i - p_j)||₂ <= threshold   (örn. 4.0 — 2x safety margin)
    bool isCollide(Eigen::Vector3d p, Eigen::Vector3d q){
        Eigen::Vector3d diff = E_diag * (p - q);
        return diff.norm() < 2.0;
    }

    // İki nokta için kapalı form hyperplane
    HyperPlane computePair(Eigen::Vector3d p, Eigen::Vector3d q, size_t i, size_t j, int k){
                               
        Eigen::Vector3d diff = p - q;
        Eigen::Vector3d normal = diff / diff.norm();
        double d = normal.dot((p + q) / 2.0);
        return HyperPlane(normal, d, i, j, k);

    }
    
    // libSVM ile (birden fazla nokta durumu için)
    HyperPlane computeSVM(...);

};
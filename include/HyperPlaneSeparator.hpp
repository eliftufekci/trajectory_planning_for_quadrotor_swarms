#pragma once
#include "Environment.hpp"
#include "Graph.hpp"
#include "HyperPlane.hpp"
#include "MAPFCTypes.hpp"
#include "SafePolyhedron.hpp"
#include "SubdividedSchedule.hpp"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <OsqpEigen/Solver.hpp>
#include "RobotModel.hpp"

class HyperPlaneSeparator{
public:
    
    HyperPlaneSeparator(const Graph& graph, 
                        const RobotModel& robotModel, 
                        const SubdividedSchedule& subdividedSchedule, 
                        const Environment& environment);

    // Compute all (i,j,k) hyperplanes for the entire schedule.
    SafePolyhedron compute();

    // For iterative refinement.
    SafePolyhedron compute(const std::vector<std::vector<std::vector<Eigen::Vector3d>>>& samples);

    // SVM (QP) solver that finds the maximum-margin hyperplane separating
    // two point sets (here, the endpoints of two segments).
    HyperPlane computeSVM( const Eigen::MatrixXd& points_i, const Eigen::MatrixXd& points_j, bool is_obstacle = false);


private:
    const Graph& graph;
    const SubdividedSchedule& subdividedSchedule;
    const RobotModel& robotModel;
    const Environment& environment;

    std::vector<size_t> findNearbyObstacles(const Eigen::Vector3d& p_start, const Eigen::Vector3d& p_end, double margin);

    std::tuple<Eigen::Vector3d, Eigen::Vector3d> find_closest_points(const Eigen::MatrixXd& s1, const Eigen::MatrixXd& s2);
    
    bool sweptVolumesCollide(const Eigen::MatrixXd& segment_i, const Eigen::MatrixXd& segment_j);

    Eigen::MatrixXd getObstacleCorners(Eigen::Vector3d min, Eigen::Vector3d max);

    double calculate_offset(Eigen::Vector3d normal_vector);
};

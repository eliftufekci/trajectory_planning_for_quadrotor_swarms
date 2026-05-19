#pragma once
#include <vector>
#include <tuple>
#include <Eigen/Dense>
#include <algorithm>
#include "ConflictAnnotation.hpp"

class SweptConflictAnnotation : public ConflictAnnotation {
public:
    SweptConflictAnnotation(const Graph& graph, const RobotModel& robotModel);

    void annotate();

private:
    Eigen::Vector3d segment_to_point(const Eigen::Vector3d& a,
                                     const Eigen::Vector3d& b, 
                                     const Eigen::Vector3d& p);

    std::tuple<Eigen::Vector3d, Eigen::Vector3d> segment_to_segment(const Eigen::Vector3d& p1, 
                                                                    const Eigen::Vector3d& p2, 
                                                                    const Eigen::Vector3d& p3, 
                                                                    const Eigen::Vector3d& p4);
    // Elipsoid uzayını birim küre uzayına dönüştürür.
    Eigen::Vector3d scale_pos(const Eigen::Vector3d& pos, 
                              const RobotModel& model);

    // map<vertex_id, set<vertex_id>>
    std::map<int, std::set<int>> findConVV();

    // map<edge_id,   set<vertex_id>>
    std::map<int, std::set<int>> findConEV();


    // map<edge_id, set<edge_id>> 
    std::map<int, std::set<int>> findConEE();

};

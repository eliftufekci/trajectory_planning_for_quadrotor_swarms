#pragma once
#include <fcl/narrowphase/continuous_collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <fcl/geometry/shape/ellipsoid.h>
#include "ConflictAnnotation.hpp"


class FCLConflictAnnotation : public ConflictAnnotation {
public:
    FCLConflictAnnotation(const Graph& graph, const RobotModel& robotModel);

    void annotate();


private:
    // map<vertex_id, set<vertex_id>>
    std::map<int, std::set<int>> findConVV(); 

    // map<edge_id,   set<vertex_id>>
    std::map<int, std::set<int>> findConEV();

    // map<edge_id, set<edge_id>> 
    std::map<int, std::set<int>> findConEE();

};

#pragma once
#include <map>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include "Environment.hpp"
#include "FCLCollisionChecker.hpp"
#include "Graph.hpp"
#include "RoadMapGenerator.hpp"
#include "RobotModel.hpp"
#include <iosfwd>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class SPARSRoadMapGenerator : public RoadMapGenerator{
public:
    double sparseDeltaFraction;

    // sparseDeltaFraction : SPARS δ parametresi (space diagonal'ına oranı).
    //                       OMPL bunu sparseDelta = fraction * space_diagonal olarak kullanır.
    //                       connectRadius: start/goal bağlama için kullanılan yarıçap.
    //                       Negatif bırakılırsa → sparseDelta değerine otomatik eşitlenir.
    SPARSRoadMapGenerator(const Environment& env, const RobotModel& robot,
                          const FCLCollisionChecker& collisionChecker,
                          double sparseDeltaFraction = 0.05,
                          double connectRadius = -1.0);

    Graph createRoadMap() override;

    bool isStateValid(const ob::State* state);

private:
    double connectRadius_;   // start/goal bağlama yarıçapı

    // SPARS'ın kendi δ'sıyla tutarlı bir search radius hesapla.
    // OMPL içinde: sparseDelta = sparseDeltaFraction * space_diagonal
    // connectRadius negatifse aynı formülü kullan → tutarlılık sağlanır.
    double computeSearchRadius() const;

    void addVertices(Graph& graph,
                     std::map<unsigned int, int>& vertexMap,
                     ob::PlannerData& data);
    
    
    void addEdges(Graph& graph,
                  std::map<unsigned int, int>& vertexMap,
                  ob::PlannerData& data);


    void connectAgents(Graph& graph);

    // Makale Section IV: "connect to up to six neighbors within a search radius
    //                     if the edge could be traversed without collision"
    // Başarı durumunda vertex ID'si, başarısızlıkta -1 döndürür.
    int connectPoint(const Eigen::Vector3d& point, Graph& graph);
};

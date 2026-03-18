#pragma once
#include <map>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include "Environment.hpp"
#include "FCLCollisionChecker.hpp"
#include "Graph.hpp"
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
    SPARSRoadMapGenerator(const Environment& env,
                          const FCLCollisionChecker& collisionChecker,
                          double sparseDeltaFraction = 0.05,
                          double connectRadius = -1.0)
        : RoadMapGenerator(env, collisionChecker)
        , sparseDeltaFraction(sparseDeltaFraction)
        , connectRadius_(connectRadius) {}

    Graph createRoadMap() override{
        Graph roadMap;
        std::map<unsigned int, int> vertexMap;

        auto space = std::make_shared<ob::RealVectorStateSpace>(3);
        ob::RealVectorBounds bounds(3);
        bounds.setLow(0,  env.world_min.x()); bounds.setHigh(0, env.world_max.x());
        bounds.setLow(1,  env.world_min.y()); bounds.setHigh(1, env.world_max.y());
        bounds.setLow(2,  env.world_min.z()); bounds.setHigh(2, env.world_max.z());
        space->setBounds(bounds);

        auto si = std::make_shared<ob::SpaceInformation>(space);
        si->setStateValidityChecker([this](const ob::State* s) {
            return isStateValid(s);
        });
        si->setup();

        // SPARS'a start/goal vermek sadece arama yolunu etkiler;
        // roadmap oluşumu bundan bağımsızdır. Ancak birden fazla start/goal
        // verilmesi, coverage'ı artırır.
        // Agent yoksa ortamın zıt iki köşesini kullan — tek nokta verilmesi
        // SPARS'ın çok seyrek roadmap üretmesine yol açabilir.
        ob::ScopedState<ob::RealVectorStateSpace> startState(space);
        ob::ScopedState<ob::RealVectorStateSpace> goalState(space);

        if (!env.agents.empty()) {
            startState[0] = env.agents[0].start.x();
            startState[1] = env.agents[0].start.y();
            startState[2] = env.agents[0].start.z();
            goalState[0]  = env.agents[0].goal.x();
            goalState[1]  = env.agents[0].goal.y();
            goalState[2]  = env.agents[0].goal.z();
        } else {
            // Zıt köşeler → SPARS tüm alanı tarama şansı bulur
            startState[0] = env.world_min.x();
            startState[1] = env.world_min.y();
            startState[2] = env.world_min.z();
            goalState[0]  = env.world_max.x();
            goalState[1]  = env.world_max.y();
            goalState[2]  = env.world_max.z();
        }

        auto pdef = std::make_shared<ob::ProblemDefinition>(si);
        pdef->addStartState(startState);
        pdef->setGoalState(goalState);

        auto planner = std::make_shared<og::SPARS>(si);
        planner->setSparseDeltaFraction(sparseDeltaFraction);
        planner->setProblemDefinition(pdef);
        planner->setup();

        // solve() hem roadmap oluşturur hem yol arar.
        // Başarısız olsa bile roadmap oluşmuş olabilir — getPlannerData() yeterli.
        ob::PlannerTerminationCondition ptc = ob::timedPlannerTerminationCondition(10.0);
        planner->solve(ptc);

        ob::PlannerData data(si);
        planner->getPlannerData(data);

        if (data.numVertices() == 0) {
            std::cerr << "SPARS: Roadmap bos olustu!\n";
            return roadMap;
        }

        std::cout << "SPARS: " << data.numVertices() << " vertex, "
                  << data.numEdges() << " edge olusturuldu.\n";

        addVertices(roadMap, vertexMap, data);
        addEdges(roadMap, vertexMap, data);

        connectAgents(roadMap);

        return roadMap;
    }

    bool isStateValid(const ob::State* state) {
        const auto* pos = state->as<ob::RealVectorStateSpace::StateType>();
        Eigen::Vector3d point(pos->values[0], pos->values[1], pos->values[2]);
        return !collisionChecker.isOccupied(point);
    }

private:
    double connectRadius_;   // start/goal bağlama yarıçapı

    // SPARS'ın kendi δ'sıyla tutarlı bir search radius hesapla.
    // OMPL içinde: sparseDelta = sparseDeltaFraction * space_diagonal
    // connectRadius negatifse aynı formülü kullan → tutarlılık sağlanır.
    double computeSearchRadius() const {
        double dx = env.world_max.x() - env.world_min.x();
        double dy = env.world_max.y() - env.world_min.y();
        double dz = env.world_max.z() - env.world_min.z();
        double sparseDelta = std::sqrt(dx*dx + dy*dy + dz*dz) * sparseDeltaFraction;

        if (connectRadius_ > 0.0)
            return connectRadius_;   // kullanıcı tanımlı
        return sparseDelta;          // SPARS δ ile aynı → dispersion'a uyumlu
    }

    void addVertices(Graph& graph,
                     std::map<unsigned int, int>& vertexMap,
                     ob::PlannerData& data)
    {
        for (unsigned int i = 0; i < data.numVertices(); i++) {
            const ob::PlannerDataVertex& v = data.getVertex(i);
            if (!v.getState()) continue;   
            const auto* s = v.getState()->as<ob::RealVectorStateSpace::StateType>();
            Eigen::Vector3d pos(s->values[0], s->values[1], s->values[2]);
            vertexMap[i] = graph.addVertex(pos);
        }
    }

    void addEdges(Graph& graph,
                  std::map<unsigned int, int>& vertexMap,
                  ob::PlannerData& data)
    {
        for (unsigned int i = 0; i < data.numVertices(); i++) {
            std::vector<unsigned int> edgeList;
            data.getEdges(i, edgeList);
            for (unsigned int nb : edgeList) {
                if (vertexMap.count(i) && vertexMap.count(nb))
                    graph.addEdge(vertexMap.at(i), vertexMap.at(nb));
            }
        }
    }

    void connectAgents(Graph& graph) {
        for (const auto& agent : env.agents) {
            auto s = connectPoint(agent.start, graph, indexMap);
            auto g = connectPoint(agent.goal,  graph, indexMap);
            start_goal_vertices.insert({s, g});
        }
    }

    // Makale Section IV: "connect to up to six neighbors within a search radius
    //                     if the edge could be traversed without collision"
    int connectPoint(const Eigen::Vector3d& point, Graph& graph) {
        int id = graph.addVertex(point);
        double search_radius = computeSearchRadius();

        std::vector<std::pair<double, int>> candidates;
        for (const auto& v : graph.getVertices()) {
            if (v.id == id) continue;
            double dist = (v.pos - point).norm();
            if (dist <= search_radius && collisionChecker.isEdgeFree(point, v.pos))
                candidates.push_back({dist, v.id});
        }

        std::sort(candidates.begin(), candidates.end());

        int connected = 0;
        for (const auto& [dist, vid] : candidates) {
            if (connected >= 6) break;
            graph.addEdge(id, vid);
            connected++;
        }

        if (connected == 0)
            std::cerr << "Uyari: [" << point.transpose()
                      << "] noktasi roadmap'e baglanamiyor! "
                      << "Search radius: " << search_radius << "\n";

        return id;
    }
};

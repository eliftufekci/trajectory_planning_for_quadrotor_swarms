#pragma once
#include <map>
#include <tuple>
#include <vector>
#include <limits>
#include <algorithm>
#include <Eigen/Dense>
#include "Environment.hpp"
#include "FCLCollisionChecker.hpp"
#include "Graph.hpp"
#include "RoadMapGenerator.hpp"


class GridRoadMapGenerator : public RoadMapGenerator{
public:
    double grid_step;

    GridRoadMapGenerator(const Environment& env,
                     const FCLCollisionChecker& collisionChecker,
                     double grid_step = 0.5)
        : RoadMapGenerator(env, collisionChecker)
        , grid_step(grid_step) {}

    Graph createRoadMap() override {
        Graph graph;
        std::map<std::tuple<int,int,int>, int> indexMap;

        addFreeVertices(graph, indexMap);
        addEdges(graph, indexMap);
        connectAgents(graph, indexMap);

        return graph;
    }

private:
    // ── Koordinatı tuple index'e çevir ───────────────────────────────
    std::tuple<int,int,int> toKey(double x, double y, double z) const {
        return {
            static_cast<int>(round((x - env.world_min.x()) / grid_step)),
            static_cast<int>(round((y - env.world_min.y()) / grid_step)),
            static_cast<int>(round((z - env.world_min.z()) / grid_step))
        };
    }

    std::tuple<int,int,int> toKey(const Eigen::Vector3d& p) const {
        return toKey(p.x(), p.y(), p.z());
    }

    // Tuple index'i gerçek koordinata çevir
    Eigen::Vector3d toPos(const std::tuple<int,int,int>& key) const {
        auto [ix, iy, iz] = key;
        return {
            env.world_min.x() + ix * grid_step,
            env.world_min.y() + iy * grid_step,
            env.world_min.z() + iz * grid_step
        };
    }

    // ── 1: engel olmayan noktaları vertex olarak ekle ─────────
    void addFreeVertices(Graph& graph, std::map<std::tuple<int,int,int>, int>& indexMap)
    {
        const auto& wmin = env.world_min;
        const auto& wmax = env.world_max;

        // Kaç adım atılacağını önceden hesapla (sınır dahil)
        int nx = static_cast<int>(std::ceil((wmax.x() - wmin.x()) / grid_step)) + 1;
        int ny = static_cast<int>(std::ceil((wmax.y() - wmin.y()) / grid_step)) + 1;
        int nz = static_cast<int>(std::ceil((wmax.z() - wmin.z()) / grid_step)) + 1;

        for (int ix = 0; ix < nx; ++ix)
        for (int iy = 0; iy < ny; ++iy)
        for (int iz = 0; iz < nz; ++iz) {
            double x = wmin.x() + ix * grid_step;
            double y = wmin.y() + iy * grid_step;
            double z = wmin.z() + iz * grid_step;

            // Sınırı aşanları sınıra sabitle
            x = std::min(x, wmax.x());
            y = std::min(y, wmax.y());
            z = std::min(z, wmax.z());
            Eigen::Vector3d point(x, y, z);
            if (collisionChecker.isOccupied(point)) continue;

            auto key = std::make_tuple(ix, iy, iz);
            if (indexMap.count(key)) continue;

            int id = graph.addVertex(point);
            indexMap[key] = id;
        }
    }

    // ── 2: her vertex için 6 komşuya edge ekle ────────────────
    void addEdges(Graph& graph, std::map<std::tuple<int,int,int>, int>& indexMap)
    {
        // 6 yönlü komşuluk (±x, ±y, ±z)
        const std::vector<Eigen::Vector3d> directions = {
            { grid_step, 0, 0}, {-grid_step, 0, 0},
            {0,  grid_step, 0}, {0, -grid_step, 0},
            {0, 0,  grid_step}, {0, 0, -grid_step},
        };

        for (const auto& [key, id1] : indexMap) {
            Eigen::Vector3d pos = toPos(key);

            for (const auto& dir : directions) {
                Eigen::Vector3d neighbor = pos + dir;
                auto neighborKey = toKey(neighbor);

                // Komşu indexMap'te yoksa engelde veya sınır dışında
                if (!indexMap.count(neighborKey)) continue;

                int id2 = indexMap.at(neighborKey);

                // Çifte kenar eklemeyi önle (id1 < id2)
                if (id1 >= id2) continue;

                if (collisionChecker.isEdgeFree(pos, neighbor)) {
                    graph.addEdge(id1, id2);
                }
            }
        }
    }

    // ── 3: Agent start/goal'larını en yakın vertex'e bağla ───────────
    void connectAgents(Graph& graph, std::map<std::tuple<int,int,int>, int>& indexMap)
    {
        for (const auto& agent : env.agents) {
            int start_id = connectPoint(agent.start, graph);
            if (start_id < 0) {
                throw std::runtime_error("Ajan " + std::to_string(agent.id) + " baslangic noktasi roadmape baglanamadi!");
            }
            int goal_id = connectPoint(agent.goal, graph);
            if (goal_id < 0) {
                throw std::runtime_error("Ajan " + std::to_string(agent.id) + " hedef noktasi roadmape baglanamadi!");
            }
            graph.start_vertices.push_back(start_id);
            graph.goal_vertices.push_back(goal_id);
        }
    }

    // Nokta indexMap'te yoksa vertex ekle + en yakın 6 vertex'e bağla
    int connectPoint(  const Eigen::Vector3d& point,
                        Graph& graph,
                        std::map<std::tuple<int,int,int>, int>& indexMap){

        // Zaten birebir grid noktasıysa ek işlem yok
        auto key = toKey(point);
        if (indexMap.count(key)) {
            if ((toPos(key) - point).norm() < 1e-6) return indexMap[key];
        }

        
        int id = graph.addVertex(point);
        indexMap[key] = id;

        std::vector<std::pair<double, int>> candidates;

        double search_radius = grid_step * 1.5;
        for (const auto& [k, vid] : indexMap) {
            if (vid == id) continue;

            double dist = (toPos(k) - point).norm();
            if(dist <= search_radius && collisionChecker.isEdgeFree(point, toPos(k)))
                candidates.push_back({dist, vid});
        }

        std::sort(candidates.begin(), candidates.end());

        int connected = 0;
        for (const auto& [dist, vid] : candidates) {
            if (connected >= 6) break;  // Makale: "up to six neighbors"
            graph.addEdge(id, vid);
            connected++;
        }

        if (connected == 0) {
            // Fallback: Yarıçap içinde düğüm bulunamazsa, en yakın çarpışmasız
            // komşuyu bul ve ona bağlan. Bu, SPARS grafiğinin yerel olarak çok
            // seyrek olduğu durumlarda bağlantı kopmasını önleyen pragmatik bir çözümdür.
            std::cerr << "Uyari: [" << point.transpose()
                      << "] noktasi icin search_radius (" << search_radius
                      << ") icinde komsu bulunamadi. Fallback: en yakin komsu araniyor.\n";

            std::pair<double, int> best_candidate = {std::numeric_limits<double>::max(), -1};
            for (const auto& v : graph.getVertices()) {
                if (v.id == id) continue;
                double dist = (v.pos - point).norm();
                if (dist < best_candidate.first && collisionChecker.isEdgeFree(point, v.pos)) {
                    best_candidate = {dist, v.id};
                }
            }

            if (best_candidate.second != -1) {
                graph.addEdge(id, best_candidate.second);
                connected++;
            }
        }

        if (connected == 0) { // Fallback'ten sonra tekrar kontrol et
            std::cerr << "HATA: [" << point.transpose()
                      << "] noktasi roadmap'e hicbir sekilde baglanamiyor!\n";
            // Bu noktayı izole bırakmak yerine, başarısızlığı bildir.
            return -1;
        }
        return id; // Başarılı bağlantı
    }
};
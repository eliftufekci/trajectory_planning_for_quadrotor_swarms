#include <iostream>
#include <fstream>
#include "Environment.h"
#include "Graph.h"
#include "RobotModel.h"

void saveEnvironmentCSV(const Environment& env, const std::string& path) {
    std::ofstream f(path);
    f << "type,x,y,z,extra\n";
    f << "world_min," << env.world_min.x() << "," << env.world_min.y() << "," << env.world_min.z() << ",0\n";
    f << "world_max," << env.world_max.x() << "," << env.world_max.y() << "," << env.world_max.z() << ",0\n";
    for (size_t i = 0; i < env.obstacles.size(); ++i) {
        const auto& obs = env.obstacles[i];
        f << "obs_min," << obs.min.x() << "," << obs.min.y() << "," << obs.min.z() << "," << i << "\n";
        f << "obs_max," << obs.max.x() << "," << obs.max.y() << "," << obs.max.z() << "," << i << "\n";
    }
    for (const auto& ag : env.agents) {
        f << "start," << ag.start.x() << "," << ag.start.y() << "," << ag.start.z() << "," << ag.id << "\n";
        f << "goal,"  << ag.goal.x()  << "," << ag.goal.y()  << "," << ag.goal.z()  << "," << ag.id << "\n";
    }
    std::cout << "Kaydedildi: " << path << "\n";
}

void saveOctreeCSV(const Environment& env, const std::string& path) {
    std::ofstream f(path);
    f << "x,y,z\n";
    int count = 0;
    for (auto it = env.tree->begin_leafs(); it != env.tree->end_leafs(); ++it) {
        if (env.tree->isNodeOccupied(*it)) {
            f << it.getX() << "," << it.getY() << "," << it.getZ() << "\n";
            ++count;
        }
    }
    std::cout << "Kaydedildi: " << path << " (" << count << " voxel)\n";
}

int main(int argc, char* argv[]) {
    // YAML path: argümandan al, yoksa varsayılan
    std::string yaml_path = (argc > 1) ? argv[1] : "../config/environment.yaml";

    std::cout << "=== Environment Test ===\n";
    std::cout << "YAML: " << yaml_path << "\n\n";

    Environment env = Environment::loadFromYAML(yaml_path);

    std::cout << "Dünya min : " << env.world_min.transpose() << "\n";
    std::cout << "Dünya max : " << env.world_max.transpose() << "\n";
    std::cout << "Engel     : " << env.obstacles.size() << "\n";
    std::cout << "Agent     : " << env.agents.size()    << "\n\n";

    // isOccupied testleri
    struct Test { Eigen::Vector3d pos; bool expected; std::string label; };
    std::vector<Test> tests = {
        { {2.0, 2.0, 1.2},  true,  "Engel-1 içi      " },
        { {3.0, 3.5, 1.0},  true,  "Engel-2 içi      " },
        { {8.0, 8.0, 2.0},  false, "Açık alan        " },
        { {0.5, 0.5, 1.0},  false, "Agent-0 start    " },
        { {9.5, 9.5, 1.0},  false, "Agent-0 goal     " },
    };

    std::cout << "isOccupied testleri:\n";
    bool all_pass = true;
    for (const auto& t : tests) {
        bool result = env.isOccupied(t.pos);
        bool ok = (result == t.expected);
        if (!ok) all_pass = false;
        std::cout << "  " << t.label
                  << (result ? "DOLU " : "BOŞ  ")
                  << (ok ? "✓" : "✗ HATA") << "\n";
    }
    std::cout << "\nSonuç: " << (all_pass ? "TÜM TESTLER GEÇTİ ✓" : "HATA VAR ✗") << "\n\n";

    // CSV çıktıları
    saveEnvironmentCSV(env, "environment_data.csv");
    saveOctreeCSV(env, "octree_voxels.csv");

    // Graph testi
    Graph g;
    int v0 = g.addVertex(env.agents[0].start);
    int v1 = g.addVertex(env.agents[0].goal);
    int v2 = g.addVertex(Eigen::Vector3d(5.0, 5.0, 1.0));
    g.addEdge(v0, v2);
    g.addEdge(v2, v1);
    g.printStats();
    g.saveToCSV("vertices.csv", "edges.csv");

    return all_pass ? 0 : 1;
}
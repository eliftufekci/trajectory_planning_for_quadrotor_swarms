#include <iostream>
#include <fstream>
#include "BezierCurve.hpp"
#include "Environment.hpp"
#include "FCLCollisionChecker.hpp"
#include "Graph.hpp"
#include "HyperPlaneSeparator.hpp"
#include "IterativeRefinement.hpp"
#include "RobotModel.hpp"
#include "GridRoadMapGenerator.hpp"
#include "SPARSRoadMapGenerator.hpp"
#include "FCLConflictAnnotation.hpp"
#include "SafePolyhedron.hpp"
#include "SubdividedSchedule.hpp"
#include "SweptConflictAnnotation.hpp"
#include "MAPFCSolver.hpp"
#include <GoalAssigner.hpp>

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
    }
    for (size_t i = 0; i < env.goal_positions.size(); ++i) {
        const auto& g = env.goal_positions[i];
        f << "goal," << g.x() << "," << g.y() << "," << g.z() << "," << i << "\n";
    }
    std::cout << "Saved: " << path << "\n";
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
    std::cout << "Saved: " << path << " (" << count << " voxels)\n";
}

int main(int argc, char* argv[]) {
    std::string yaml_path = (argc > 1) ? argv[1] : "../config/environment.yaml";
    std::cout << "=== Multi-Agent Trajectory Planning ===\n";
    std::cout << "YAML: " << yaml_path << "\n\n";

    // 1. Environment
    Environment env = Environment::loadFromYAML(yaml_path);

    std::cout << "World min : " << env.world_min.transpose() << "\n";
    std::cout << "World max : " << env.world_max.transpose() << "\n";
    std::cout << "Obstacles : " << env.obstacles.size() << "\n";
    std::cout << "Agents    : " << env.agents.size() << "\n";
    std::cout << "Labeled   : " << (env.labeled ? "yes" : "no") << "\n\n";

    if (env.agents.size() != env.goal_positions.size()) {
        std::cerr << "ERROR: agent count (" << env.agents.size()
                  << ") does not match goal count (" << env.goal_positions.size()
                  << ")!\n";
        return 1;
    }

    saveEnvironmentCSV(env, "environment_data.csv");

    // 2. Roadmap
    RobotModel robot;
    FCLCollisionChecker fclCollisionChecker(env, robot);
    SPARSRoadMapGenerator roadMapGenerator(env, robot, fclCollisionChecker);

    Graph environment_graph;
    try {
        environment_graph = roadMapGenerator.createRoadMap();
    } catch (const std::runtime_error& e) {
        std::cerr << "Roadmap creation failed: " << e.what() << "\n";
        return 1;
    }
    environment_graph.saveToCSV("vertices.csv", "edges.csv");
    saveOctreeCSV(env, "octree_voxels.csv");

    // 3. Store start vertices in the graph.
    for (const auto& agent : env.agents)
        environment_graph.start_vertices.push_back(
            environment_graph.findNearestVertex(agent.start));

    // 4. Goal assignment
    std::vector<int> goal_verts;
    for (const auto& gp : env.goal_positions)
        goal_verts.push_back(environment_graph.findNearestVertex(gp));

    if (env.labeled) {
        // Preserve order: goal[i] -> robot[i].
        std::cout << "=== Labeled: Ordered goal assignment ===\n";
        environment_graph.goal_vertices = goal_verts;
    } else {
        // Assignment with the threshold algorithm.
        std::cout << "=== Unlabeled: Threshold goal assignment ===\n";
        GoalAssigner assigner;
        auto assignment = assigner.assign( environment_graph,
                                           environment_graph.start_vertices,
                                           goal_verts);

        if ((int)assignment.size() != (int)env.agents.size()) {
            std::cerr << "ERROR: Goal assignment failed!\n";
            return 1;
        }

        environment_graph.goal_vertices.resize(assignment.size());
        for (int i = 0; i < (int)assignment.size(); i++) {
            environment_graph.goal_vertices[i] = goal_verts[assignment[i]];
            std::cout << "Robot " << i << " -> Goal " << assignment[i] << "\n";
        }
    }

    // 5. Conflict annotation
    std::cout << "\n=== Conflict Annotation ===\n";
    FCLConflictAnnotation fcl_conflict_annotation(environment_graph, robot);
    fcl_conflict_annotation.annotate();

    SweptConflictAnnotation swept_conflict_annotation(environment_graph, robot);
    swept_conflict_annotation.annotate();

    std::cout << "FCL   - conVV: " << fcl_conflict_annotation.conVV.size()
              << "  conEV: " << fcl_conflict_annotation.conEV.size()
              << "  conEE: " << fcl_conflict_annotation.conEE.size() << "\n";
    std::cout << "Swept - conVV: " << swept_conflict_annotation.conVV.size()
              << "  conEV: " << swept_conflict_annotation.conEV.size()
              << "  conEE: " << swept_conflict_annotation.conEE.size() << "\n";

    // 6. Discrete planning (ECBS)
    std::cout << "\n=== Discrete Planning (ECBS) ===\n";
    MAPFCSolver solver(environment_graph, swept_conflict_annotation);
    DiscreteSchedule schedule = solver.solve();

    std::cout << "Makespan (K): " << schedule.K << "\n";
    for (size_t i = 0; i < schedule.waypoint.size(); ++i)
        std::cout << "  Robot " << i
                  << " path length: " << schedule.waypoint[i].size() << "\n";

    SubdividedSchedule subdividedSchedule = schedule.subdivide(environment_graph);

    // 7. Continuous trajectory optimization
    std::cout << "\n=== Iterative Refinement ===\n";
    IterativeRefinement iterativeRefinement(
        environment_graph, subdividedSchedule, robot, env);

    double T_initial = static_cast<double>(subdividedSchedule.K);
    int K = subdividedSchedule.K;

    auto control_points = iterativeRefinement.refine(T_initial, 6);
    iterativeRefinement.saveControlPointsToCSV(control_points, "control_points.csv");

    // 8. Dynamic scaling
    double max_velocity     = 2.0;
    double max_acceleration = 2.0;
    double T_scaled = iterativeRefinement.computeScaledTime(
        control_points, T_initial, K, max_acceleration, max_velocity);

    std::cout << "Flight time (T_scaled): " << T_scaled << " s\n";

    return 0;
}

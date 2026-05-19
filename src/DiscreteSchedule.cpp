#include "DiscreteSchedule.hpp"
#include "SubdividedSchedule.hpp"
#include "Graph.hpp"
#include <Eigen/Dense>

DiscreteSchedule::DiscreteSchedule( const std::vector<std::vector<int>>& waypoint, int K)
    : waypoint(waypoint), K(K) {}


SubdividedSchedule DiscreteSchedule::subdivide(const Graph& graph){
    SubdividedSchedule subdividedSchedule(waypoint.size(), this->K);
    
    for(size_t i = 0; i < waypoint.size(); ++i){
        const auto& path = waypoint[i];
        if (path.empty()) {
            continue; // Skip empty paths.
        }
        
        // Iterate over the original K time intervals.
        for (int k = 0; k < this->K; ++k){
            // Find the vertex at time k and k+1.
            // If the agent has reached the goal, it waits at the last position.
            int curr_vertex_id = (k < path.size()) ? path[k] : path.back();
            int next_vertex_id = (k + 1 < path.size()) ? path[k + 1] : path.back();

            Eigen::Vector3d curr = graph.getVertex(curr_vertex_id).pos;
            Eigen::Vector3d next = graph.getVertex(next_vertex_id).pos;

            Eigen::Vector3d mid = (curr + next) * 0.5;

            subdividedSchedule.positions[i].push_back(curr);
            subdividedSchedule.positions[i].push_back(mid);
        }

        // Final point (position at time K).
        int final_vertex_id = (this->K < path.size()) ? path[this->K] : path.back();
        subdividedSchedule.positions[i].push_back(graph.getVertex(final_vertex_id).pos);
    }

    return subdividedSchedule;
}
    

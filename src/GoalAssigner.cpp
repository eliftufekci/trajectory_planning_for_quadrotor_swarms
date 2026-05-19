#include "GoalAssigner.hpp"
#include "Graph.hpp"
#include <vector>
#include <queue>
#include <algorithm>
#include <numeric>


// Returns the mapping start_vertices[i] -> goal_vertices[assignment[i]].
std::vector<int> GoalAssigner::assign(const Graph& graph,
                        const std::vector<int>& start_vertices,
                        const std::vector<int>& goal_vertices) {
    
    int N = start_vertices.size();

    // Her start'tan tim goal'lara maliyet matrisi
    std::vector<std::vector<double>> cost(N, std::vector<double>(N, 1e9));

    for (int i=0; i < N; i++) {
        auto dist = graph.dijkstra(start_vertices[i]);
        for (int j=0; j < N; j++) {
            cost[i][j] = dist[goal_vertices[j]];
        }
    }

    return GoalAssigner::thresholdAssign(cost, N);
}


// Bottleneck assignment: minimize the longest individual path.
std::vector<int> GoalAssigner::thresholdAssign(const std::vector<std::vector<double>>& cost, int N){

    // Collect and sort all cost values.
    std::vector<double> sorted_costs;
    for (auto& row : cost){
        for (double v : row){
            if (v < 1e8){
                sorted_costs.push_back(v);
            }
        }
    }
    std::sort(sorted_costs.begin(), sorted_costs.end());
    sorted_costs.erase(std::unique(sorted_costs.begin(), sorted_costs.end()), sorted_costs.end());

    // Start from the smallest threshold and try until a perfect matching is found.
    for (double threshold : sorted_costs) {
        auto assignment = tryBipartiteMatch(cost, N, threshold);
        if ((int)assignment.size() == N)
            return assignment;
    }

    // Fallback: ordered assignment.
    std::vector<int> fallback(N);
    std::iota(fallback.begin(), fallback.end(), 0);
    return fallback;
}

// Bipartite matching with the given threshold (augmenting path).
std::vector<int> GoalAssigner::tryBipartiteMatch( const std::vector<std::vector<double>>& cost, 
                                    int N, 
                                    double threshold){

    // matchL[i] = robot i'nin eşleştiği goal (-1 = yok)
    // matchR[j] = goal j'nin eşleştiği robot (-1 = yok)
    std::vector<int> matchL(N, -1), matchR(N, -1);

    for (int i = 0; i < N; i++) {
        std::vector<bool> visited(N, false);
        augment(i, cost, threshold, matchL, matchR, visited, N);
    }

    // Return if all robots are matched.
    for (int i = 0; i < N; i++){
        if (matchL[i] == -1) 
            return {};
    }
        
    return matchL;
}

bool GoalAssigner::augment(int u,
                const std::vector<std::vector<double>>& cost,
                double threshold,
                std::vector<int>& matchL,
                std::vector<int>& matchR,
                std::vector<bool>& visited,
                int N) {

    for (int v = 0; v < N; v++) {
        if (cost[u][v] > threshold || visited[v]) 
            continue;
        visited[v] = true;

        if (matchR[v] == -1 || augment(matchR[v], cost, threshold, matchL, matchR, visited, N)) {
            matchL[u] = v;
            matchR[v] = u;
            return true;
        }
    }
    return false;
}

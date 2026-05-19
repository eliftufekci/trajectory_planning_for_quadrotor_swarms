#pragma once
#include <vector>
#include <queue>
#include <algorithm>
#include <numeric>

class Graph;

class GoalAssigner {
public:
    // start_vertices[i] -> goal_vertices[assignment[i]] eşleşmesini döndürür
    std::vector<int> assign(const Graph& graph,
                            const std::vector<int>& start_vertices,
                            const std::vector<int>& goal_vertices);
private:
    // Bottleneck assignment: en uzun tek yolu minimize et
    std::vector<int> thresholdAssign(const std::vector<std::vector<double>>& cost, int N);

    // Verilen threshold ile bipartite matching (augmenting path)
    std::vector<int> tryBipartiteMatch( const std::vector<std::vector<double>>& cost, 
                                        int N, 
                                        double threshold);
    bool augment(int u,
                 const std::vector<std::vector<double>>& cost,
                 double threshold,
                 std::vector<int>& matchL,
                 std::vector<int>& matchR,
                 std::vector<bool>& visited,
                 int N);
};
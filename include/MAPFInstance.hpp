#pragma once
#include <vector>
#include <map>
#include <set>
#include <tuple>
#include "Graph.hpp"

struct MAPFInstance{
    const Graph& graph;
    std::map<int, std::set<int>> conVV;
    std::map<int, std::set<int>> conEV;
    std::map<int, std::set<int>> conEE;
    std::tuple<std::vector<int>, std::vector<int>> start_goal_vertices;

    MAPFInstance( const Graph& graph
                , std::map<int, std::set<int>> conVV
                , std::map<int, std::set<int>> conEV
                , std::map<int, std::set<int>> conEE
                , std::tuple<std::vector<int>, std::vector<int>> start_goal_vertices)
        : graph(graph)
        , conVV(conVV)
        , conEV(conEV)
        , conEE(conEE)
        , start_goal_vertices(start_goal_vertices) {}
};

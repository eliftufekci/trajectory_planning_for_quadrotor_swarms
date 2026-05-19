#pragma once
#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <stdexcept>
#include <queue>
#include <iostream>


struct Vertex{
    int id;
    Eigen::Vector3d pos;

    Vertex(int id, const Eigen::Vector3d& pos);
};


struct Edge{
    int id;       // unique edge index (undirected: each edge appears once)
    int from;
    int to;
    double cost;

    Edge(int id, int from, int to, double cost);
};


class Graph{
public:
    std::vector<Vertex> vertices;

    // edges: each undirected edge is stored ONLY ONCE (from < to is guaranteed).
    std::vector<Edge> edges;

    // adjacency: vertex id -> neighboring vertex ids
    std::unordered_map<int, std::vector<int>> adjacency;

    // edge_index: (from, to) pair -> index in the edges vector
    // Both directions point to the same edge id: (u,v) and (v,u) -> same Edge.
    std::unordered_map<int, std::unordered_map<int, int>> edge_index;

    std::vector<int> start_vertices;
    std::vector<int> goal_vertices;

    // Find the nearest vertex.
    int findNearestVertex(const Eigen::Vector3d& pos) const;

    // Dijkstra - distances from start to all vertices.
    std::vector<double> dijkstra(int start) const;

    // Add a vertex and return its id.
    int addVertex(const Eigen::Vector3d& pos);

    // Add an edge (undirected); returns the edge id, or the existing id if present.
    int addEdge(int from, int to);

    // Access an Edge object by edge id.
    const Edge& getEdge(int edge_id) const;
    const Vertex& getVertex(int vertex_id) const;

    // Access an edge id from a (from, to) pair (-1: not found).
    int getEdgeId(int from, int to) const;

    // Return neighbors.
    const std::vector<int>& neighbors(int id) const;

    // Statistics.
    void printStats() const;

    // Save to CSV.
    void saveToCSV(const std::string& vertex_file, const std::string& edge_file) const;

    const std::vector<Vertex>& getVertices() const;
    const std::vector<Edge>&   getEdges()    const;
};

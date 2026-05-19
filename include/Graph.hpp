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
    int id;       // benzersiz kenar indeksi (undirected: her kenar bir kez)
    int from;
    int to;
    double cost;

    Edge(int id, int from, int to, double cost);
};


class Graph{
public:
    std::vector<Vertex> vertices;

    // edges: her undirected kenar SADECE BİR KEZ tutulur (from < to garantili)
    std::vector<Edge> edges;

    // adjacency: vertex id → komşu vertex id'leri
    std::unordered_map<int, std::vector<int>> adjacency;

    // edge_index: (from, to) çifti → edges vektöründeki indeks
    // Her iki yön de aynı edge id'ye işaret eder: (u,v) ve (v,u) → aynı Edge
    std::unordered_map<int, std::unordered_map<int, int>> edge_index;

    std::vector<int> start_vertices;
    std::vector<int> goal_vertices;

    // ── En Yakın Vertex'i bul ─────────────────────────────
    int findNearestVertex(const Eigen::Vector3d& pos) const;

    // ── Dijkstra - strat'tan -> tüm vertex'lere mesafe ─────────────────────────────
    std::vector<double> dijkstra(int start) const;

    // ── Vertex ekle, id döndür ────────────────────────────────────────
    int addVertex(const Eigen::Vector3d& pos);

    // ── Kenar ekle (undirected) — edge id döndürür, zaten varsa mevcut id döner
    int addEdge(int from, int to);

    // ── Edge id'den Edge nesnesine eriş ──────────────────────────────
    const Edge& getEdge(int edge_id) const;
    const Vertex& getVertex(int vertex_id) const;

    // ── (from, to) çiftinden edge id'ye eriş (-1: yok) ───────────────
    int getEdgeId(int from, int to) const;

    // ── Komşuları döndür ─────────────────────────────────────────────
    const std::vector<int>& neighbors(int id) const;

    // ── İstatistik ───────────────────────────────────────────────────
    void printStats() const;

    // ── CSV'ye kaydet ─────────────────────────────────────────────────
    void saveToCSV(const std::string& vertex_file, const std::string& edge_file) const;

    const std::vector<Vertex>& getVertices() const;
    const std::vector<Edge>&   getEdges()    const;
};
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

    Vertex(int id, const Eigen::Vector3d& pos) : id(id), pos(pos) {}
};


struct Edge{
    int id;       // benzersiz kenar indeksi (undirected: her kenar bir kez)
    int from;
    int to;
    double cost;

    Edge(int id, int from, int to, double cost)
        : id(id), from(from), to(to), cost(cost) {}
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
    int findNearestVertex(const Eigen::Vector3d& pos) const{
        int best = -1;
        double best_dist = 1e9;
        
        for (const auto& v : vertices) {
            double dist = (v.pos - pos).norm();
            if (dist < best_dist) {
                best_dist = dist;
                best = v.id;
            }
        }
        return best;
    }

    // ── Dijkstra - strat'tan -> tüm vertex'lere mesafe ─────────────────────────────
    std::vector<double> dijkstra(int start) const {
        std::vector<double> dist(vertices.size(), 1e9);

        std::priority_queue<std::pair<double,int>,
                            std::vector<std::pair<double,int>>,
                            std::greater<>> pq;

        dist[start] = 0.0;
        pq.push({0.0, start});

        while (!pq.empty()) {
            auto [d, u] = pq.top();
            pq.pop();

            if (d > dist[u]) 
                continue;
            
            for (int v : neighbors(u)) {
                double nd = d + getEdge(getEdgeId(u,v)).cost;
                if (nd < dist[v]) {
                    dist[v] = nd;
                    pq.push({nd, v});
                }
            }
        }
        return dist;
    }


    // ── Vertex ekle, id döndür ────────────────────────────────────────
    int addVertex(const Eigen::Vector3d& pos) {
        int id = static_cast<int>(vertices.size());
        vertices.emplace_back(id, pos);
        adjacency[id] = {};
        edge_index[id] = {};
        return id;
    }

    // ── Kenar ekle (undirected) — edge id döndürür, zaten varsa mevcut id döner
    int addEdge(int from, int to) {
        if (from < 0 || from >= static_cast<int>(vertices.size()) ||
            to   < 0 || to   >= static_cast<int>(vertices.size())){
            throw std::out_of_range("addEdge: geçersiz vertex id");
        }

        // Zaten varsa mevcut edge id'yi döndür
        if (edge_index.count(from) && edge_index[from].count(to))
            return edge_index[from][to];

        // Canonical yön: her zaman from < to olarak sakla
        int u = std::min(from, to);
        int v = std::max(from, to);

        int eid = static_cast<int>(edges.size());
        double cost = (vertices[u].pos - vertices[v].pos).norm();
        edges.emplace_back(eid, u, v, cost);

        // Her iki yön de aynı edge id'ye işaret etsin
        edge_index[u][v] = eid;
        edge_index[v][u] = eid;

        adjacency[u].push_back(v);
        adjacency[v].push_back(u);

        return eid;
    }

    // ── Edge id'den Edge nesnesine eriş ──────────────────────────────
    const Edge& getEdge(int edge_id) const {
        return edges.at(edge_id);
    }

    const Vertex& getVertex(int vertex_id) const{
        return vertices.at(vertex_id);
    }

    // ── (from, to) çiftinden edge id'ye eriş (-1: yok) ───────────────
    int getEdgeId(int from, int to) const {
        auto it = edge_index.find(from);
        if (it == edge_index.end()) return -1;
        auto jt = it->second.find(to);
        if (jt == it->second.end()) return -1;
        return jt->second;
    }

    // ── Komşuları döndür ─────────────────────────────────────────────
    const std::vector<int>& neighbors(int id) const {
        return adjacency.at(id);
    }

    // ── İstatistik ───────────────────────────────────────────────────
    void printStats() const {
        std::cout << "Graph: "
                  << vertices.size() << " vertices, "
                  << edges.size()    << " edges\n";
    }

    // ── CSV'ye kaydet ─────────────────────────────────────────────────
    void saveToCSV(const std::string& vertex_file,
                   const std::string& edge_file) const {

        std::ofstream vf(vertex_file);
        if (!vf) throw std::runtime_error("Vertex dosyası açılamadı: " + vertex_file);
        vf << "id,x,y,z\n";
        for (const auto& v : vertices)
            vf << v.id << ","
               << v.pos.x() << "," << v.pos.y() << "," << v.pos.z() << "\n";

        // edges zaten tekil (from < to), doğrudan yaz
        std::ofstream ef(edge_file);
        if (!ef) throw std::runtime_error("Edge dosyası açılamadı: " + edge_file);
        ef << "edge_id,from,to,cost,from_x,from_y,from_z,to_x,to_y,to_z\n";
        for (const auto& e : edges) {
            const auto& a = vertices[e.from].pos;
            const auto& b = vertices[e.to  ].pos;
            ef << e.id   << ","
               << e.from << "," << e.to << "," << e.cost << ","
               << a.x()  << "," << a.y() << "," << a.z() << ","
               << b.x()  << "," << b.y() << "," << b.z() << "\n";
        }
        std::cout << "Kaydedildi: " << vertex_file << ", " << edge_file << "\n";
    }

    const std::vector<Vertex>& getVertices() const { return vertices; }
    const std::vector<Edge>&   getEdges()    const { return edges;    }
};
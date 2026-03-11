#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <stdexcept>


struct Vertex{
    int id;
    Eigen::Vector3d pos;

    Vertex(int id, const Eigen::Vector3d& pos) : id(id), pos(pos) {}
};


struct Edge{
    int from;
    int to;
    double cost;

    Edge(int from, int to, double cost) : from(from), to(to), cost(cost) {}
};


class Graph{
public:
    std::vector<Vertex> vertices;
    std::vector<Edge> edges;
    std::unordered_map<int, std::vector<int>> adjacency;

    // ── Vertex ekle, id döndür ────────────────────────────────────────
    int addVertex(const Eigen::Vector3d& pos) {
        int id = static_cast<int>(vertices.size());
        vertices.emplace_back(id, pos);
        adjacency[id] = {};
        return id;
    }

    // ── Kenar ekle (undirected), kenar indexini döndür ────────────────
    void addEdge(int from, int to) {
        auto& adj = adjacency[from];
        if (std::find(adj.begin(), adj.end(), to) != adj.end()) 
            return;

        if (from < 0 || from >= static_cast<int>(vertices.size()) ||
            to   < 0 || to   >= static_cast<int>(vertices.size())){
            throw std::out_of_range("addEdge: geçersiz vertex id");
        }

        double cost = (vertices[from].pos - vertices[to].pos).norm();

        // Çift yönlü kenar (undirected)
        edges.emplace_back(from, to,   cost);
        edges.emplace_back(to,   from, cost);

        adjacency[from].push_back(to);
        adjacency[to].push_back(from);
    }

    // ── Komşuları döndür ─────────────────────────────────────────────
    const std::vector<int>& neighbors(int id) const {
        return adjacency.at(id);
    }

    // ── İstatistik ───────────────────────────────────────────────────
    void printStats() const {
        std::cout << "Graph: "
                  << vertices.size()     << " vertices, "
                  << edges.size() / 2    << " edges\n";
    }

    // ── CSV'ye kaydet ─────────────────────────────────────────────────
    void saveToCSV(const std::string& vertex_file,
                   const std::string& edge_file) const{
                    
        // Vertex'ler
        std::ofstream vf(vertex_file);
        if (!vf) throw std::runtime_error("Vertex dosyası açılamadı: " + vertex_file);
        vf << "id,x,y,z\n";
        for (const auto& v : vertices)
            vf << v.id << ","
               << v.pos.x() << "," << v.pos.y() << "," << v.pos.z() << "\n";

        // Edge'ler (her çifti bir kere yaz: from < to)
        std::ofstream ef(edge_file);
        if (!ef) throw std::runtime_error("Edge dosyası açılamadı: " + edge_file);
        ef << "from,to,cost,from_x,from_y,from_z,to_x,to_y,to_z\n";
        for (const auto& e : edges) {
            if (e.from < e.to) {
                const auto& a = vertices[e.from].pos;
                const auto& b = vertices[e.to  ].pos;
                ef << e.from << "," << e.to << "," << e.cost << ","
                   << a.x() << "," << a.y() << "," << a.z() << ","
                   << b.x() << "," << b.y() << "," << b.z() << "\n";
            }
        }
        std::cout << "Kaydedildi: " << vertex_file << ", " << edge_file << "\n";
    }
};
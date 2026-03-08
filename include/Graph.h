#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <fstream>


struct Vertex{
    int id;
    Eigen::Vector3d pos;
    
    Vertex(int id, Eigen::Vector3d pos) : id(id), pos(pos) {}
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

    int addVertex(Eigen::Vector3d pos){
        int id = vertices.size();
        vertices.emplace_back(id, pos);
        adjacency[id] = {};
        return id;
    }

    //push_back: önce bir nesne oluşturur, sonra onu vector'ün içine kopyalar
    //emplace_back:	nesneyi doğrudan vector'ün içinde oluşturur (constructor'ı orada çağırır)

    int addEdge(int from, int to){
        double cost = (vertices[from].pos - vertices[to].pos).norm();
        edges.emplace_back(from, to, cost);
        edges.emplace_back(to, from, cost);  // Çift yönlü, undirected
        adjacency[from].push_back(to);
        adjacency[to].push_back(from);
    }

    //const: bu metodun değişkenleri değiştirmeyeceğinin sözünü verir
    void printStats() const{
        std::cout << "Graph: " << vertices.size() << " vertices, "
                  << edges.size() / 2 << " edges\n";
    }

    void saveToCSV(const std::string& vertex_file,
                   const std::string& edge_file) const {
        // Vertex'leri kaydet
        std::ofstream vf(vertex_file);
        vf << "id,x,y,z\n";
        for (const auto& v : vertices) {
            vf << v.id << "," << v.pos.x() << ","
               << v.pos.y() << "," << v.pos.z() << "\n";
        }

        // Edge'leri kaydet
        std::ofstream ef(edge_file);
        ef << "from_x,from_y,from_z,to_x,to_y,to_z\n";
        for (const auto& edge : edges) {
            if (edge.from < edge.to) {  // Duplicate'leri atla
                const auto& a = vertices[edge.from].pos;
                const auto& b = vertices[edge.to].pos;
                ef << a.x() << "," << a.y() << "," << a.z() << ","
                   << b.x() << "," << b.y() << "," << b.z() << "\n";
            }
        }
        std::cout << "Saved: " << vertex_file << ", " << edge_file << "\n";
    }

};
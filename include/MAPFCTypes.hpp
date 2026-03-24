#pragma once
#include <unordered_set>
#include "Graph.hpp"

struct State{
    int time;
    int vertex_id;

    State(int time, int vertex_id) : time(time), vertex_id(vertex_id) {}

    bool operator==(const State& s) const {
        return time == s.time && vertex_id == s.vertex_id;
    }

    bool equalExceptTime(const State& s) const {
        return vertex_id == s.vertex_id; 
    }

};

namespace std{
    template <>
    struct hash<State> {
        size_t operator()(const State& s) const {
            std::hash<int> hasher;
    
            // 2. İlk değeri al
            size_t seed = hasher(s.time);
    
            // 3. Diğer değerleri "karıştırarak" ekle (Manuel hash_combine)
            // Bu sihirli sayı (0x9e3779b9) altın oran tabanlıdır ve çakışmayı azaltır.
            seed ^= hasher(s.vertex_id) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            // ^= xor
    
            return seed;
        }
    };
}

struct Action{ // bir timestmapde yapılan bir hareket
    int edge_id;

    Action (int edge_id) : edge_id(edge_id) {}

    // wait -1
};


struct Conflict{
    enum Type {
        vertex,
        edge,
        conVV,
        conEE,
        conEV
    };

    size_t agent1;
    size_t agent2;
    int time;
    Type type;

    int vertex_id; // conVV için
    int edge_id1;  // conEE ve conEV için
    int edge_id2;

};


struct VertexConstraint{
    int time;
    int vertex_id;

    VertexConstraint(int time, int vertex_id) : time(time), vertex_id(vertex_id) {}

    bool operator==(const VertexConstraint& vc) const {
        return time == vc.time && vertex_id == vc.vertex_id;
    }

    bool operator<(const VertexConstraint& other) const {
        // return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
        return std::tie(time, vertex_id) < std::tie(other.time, other.vertex_id);
    }
};


namespace std{
    template <>
    struct hash<VertexConstraint> {
        size_t operator()(const VertexConstraint& vc) const {
            std::hash<int> hasher;
    
            size_t seed = hasher(vc.time);
            seed ^= hasher(vc.vertex_id) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    
            return seed;
        }
    };
}


struct EdgeConstraint{
    int time;
    int edge_id;

    EdgeConstraint(int time, int edge_id) : time(time), edge_id(edge_id){}

    bool operator==(const EdgeConstraint& ec) const {
        return time == ec.time && edge_id == ec.edge_id;
    }

    bool operator<(const EdgeConstraint& other) const {
        return std::tie(time, edge_id) < std::tie(other.time, other.edge_id);
    }
};

namespace std{
    template <>
    struct hash<EdgeConstraint> {
        size_t operator()(const EdgeConstraint& ec) const {
            std::hash<int> hasher;
    
            size_t seed = hasher(ec.time);
            seed ^= hasher(ec.edge_id) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    
            return seed;
        }
    };
}


struct Constraints {
    std::unordered_set<VertexConstraint> vertexConstraints;
    std::unordered_set<EdgeConstraint> edgeConstraints;

    void add(const Constraints& other) {
        vertexConstraints.insert( other.vertexConstraints.begin(),
                                  other.vertexConstraints.end());

        edgeConstraints.insert( other.edgeConstraints.begin(),
                                other.edgeConstraints.end());
    }

    bool overlap(const Constraints& other) const {
        for (const auto& vc : vertexConstraints) {
            if (other.vertexConstraints.count(vc) > 0) {
                return true;
            }
        }
        for (const auto& ec : edgeConstraints) {
            if (other.edgeConstraints.count(ec) > 0) {
                return true;
            }
        }
        return false;
    }
};

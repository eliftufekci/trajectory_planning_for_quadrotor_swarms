struct ConflictAnnotation{
    ConflictAnnotation(Graph graph);

    std::map<int, std::set<int>> conVV; // map<vertex_id, set<vertex_id>>
    std::map<int, std::set<int>> conEE; // map<edge_id,   set<edge_id>> 
    std::map<int, std::set<int>> conEV; // map<edge_id,   set<vertex_id>>
};

void findCovVV(){

}

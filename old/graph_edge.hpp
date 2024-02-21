#ifndef GRAPH_EDGE_HPP
#define GRAPH_EDGE_HPP

#include "graph_node.hpp" 

#include <cmath> // For sqrt and pow functions
#include <string>

class Edge {
public:
    Node* from;
    Node* to;
    float length;
    std::string type;

    Edge(Node* from, Node* to, std::string type);
    void calculateEdgeLength(); // Declaration
};

#endif // GRAPH_EDGE_HPP
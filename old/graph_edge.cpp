#include "graph_edge.hpp"

Edge::Edge(Node* from, Node* to, std::string type) 
: from(from), to(to), type(type) {
    calculateEdgeLength(); // This will set the length based on from and to nodes
}

void Edge::calculateEdgeLength() {
    this->length = sqrt(pow(from->x - to->x, 2) + pow(from->y - to->y, 2) + pow(from->z - to->z, 2));
}
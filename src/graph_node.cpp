#include "graph_node.hpp"

Node::Node(int id, float x, float y, float z)
    : id(id), mappedId(-1), x(x), y(y), z(z), parent(nullptr) {}

void Node::addChild(Node* child) {
    children.push_back(child);
}
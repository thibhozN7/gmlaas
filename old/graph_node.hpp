#ifndef GRAPH_NODE_HPP
#define GRAPH_NODE_HPP

#include <vector>
#include <string>

class Node {
public:
    int id;
    int mappedId;
    float x, y, z;
    Node* parent;
    std::vector<Node*> children;

    Node(int id, float x, float y, float z);
    void addChild(Node* child);
};

#endif // GRAPH_NODE_HPP
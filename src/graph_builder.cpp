#include "graph_builder.hpp"

#include <iostream>


GraphBuilder::~GraphBuilder() {
    for (auto node : nodes) delete node;
    for (auto edge : edges) delete edge;
}

void GraphBuilder::addNode(Node* node) {
    nodes.push_back(node);
}

void GraphBuilder::addEdge(Edge* edge) {
    edges.push_back(edge);
}


void GraphBuilder::calculateAdjacencyMatrix(std::vector<std::vector<int>>& adjacencyMatrix, bool sym) {
    // Create a mapping of node IDs to indices
    std::unordered_map<int, int> nodeMapping = createNodeMapping();

    // Clear the matrix before populating it again
    adjacencyMatrix.clear();
    adjacencyMatrix.resize(nodes.size(), std::vector<int>(nodes.size(), 0));

    // Populate the adjacency matrix using edge lengths
    for (const auto& edge : edges) {
        // Check if the from and to nodes exist in the node mapping
        if (nodeMapping.find(edge->from->id) == nodeMapping.end() ||
            nodeMapping.find(edge->to->id) == nodeMapping.end()) {
            // Handle error: Nodes not found in the node mapping
            std::cerr << "Error: Nodes not found in the node mapping." << std::endl;
            return;
        }

        // Get the indices of the from and to nodes
        int fromId = nodeMapping[edge->from->id];
        int toId = nodeMapping[edge->to->id];

        // Assign the edge length to the corresponding matrix entry
        adjacencyMatrix[fromId][toId] = edge->length;

        // If symmetric, assign the same length to the corresponding entry
        if (sym) {
            adjacencyMatrix[toId][fromId] = edge->length;
        }
    }
}

void GraphBuilder::printAdjacencyMatrix(const std::vector<std::vector<int>>& adjacencyMatrix) {
    std::cout << "Adjacency Matrix:" << std::endl;
    for (const auto& row : adjacencyMatrix) {
        for (int val : row) {
            std::cout << val << " ";
        }
        std::cout << std::endl;
    }
}

std::unordered_map<int, int> GraphBuilder::createNodeMapping() {
    std::unordered_map<int, int> mapping;
    int index = 0;
    for (const auto& node : nodes) {
        mapping[node->id] = index;
        node->mappedId = index;
        ++index;
    }
    return mapping;
}

void GraphBuilder::printNodeMapping(const std::unordered_map<int, int>& nodeMapping) {
    std::cout << "Node Mapping:" << std::endl;
    for (const auto& pair : nodeMapping) {
        std::cout << "Original ID: " << pair.first << " -> Sequential Index: " << pair.second << std::endl;
    }
}
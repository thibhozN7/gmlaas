#include <vector>
#include <unordered_map>

#include "graph_node.hpp"
#include "graph_edge.hpp"

class GraphBuilder {
public:
    std::vector<Node*> nodes;
    std::vector<Edge*> edges;

    ~GraphBuilder();
    void addNode(Node* node);
    void addEdge(Edge* edge);
    void printNodeMapping(const std::unordered_map<int, int>& nodeMapping);
    void calculateAdjacencyMatrix(std::vector<std::vector<int>>& adjacencyMatrix,  bool sym=true);
    std::unordered_map<int, int> createNodeMapping();
    void printAdjacencyMatrix(const std::vector<std::vector<int>>& adjacencyMatrix);};

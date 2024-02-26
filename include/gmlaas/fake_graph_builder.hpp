#ifndef FAKE_GRAPH_BUILDER_HPP
#define FAKE_GRAPH_BUILDER_HPP

#include <unordered_map>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

class Node {
public:
    int id;
    int mappedId;
    float x;
    float y;
    float z;
    std::vector<Node*> children;
    Node* parent;

    Node(int id, float x, float y, float z);
    void addChild(Node* child);
};

class Edge {
public:
    Node* from;
    Node* to;
    float length;
    std::string type;

    Edge(Node* from, Node* to, std::string type);
    void calculateEdgeLength();
};

class FakeGraphBuilder {
private:
    ros::NodeHandle m_node;
    ros::Publisher m_pub;

    std::vector<Node*> nodes;
    std::vector<Edge*> edges;

    void init();
    std::unordered_map<int, int> createNodeMapping();
    void printNodeMapping(const std::unordered_map<int, int>& nodeMapping);
    
public:
    FakeGraphBuilder(ros::NodeHandle& node);
    // ~FakeGraphBuilder();
    void addNode(Node* node);
    void addEdge(Edge* edge);
    void calculateAdjacencyMatrix(std::vector<std::vector<float>>& adjacencyMatrix, bool sym = true, bool oriented = true);
    void publishAdjacencyMatrix(const std::vector<std::vector<float>>& adjacencyMatrix);
    void initFakeGraph();
    void processFakeGraph();

};

#endif // FAKE_GRAPH_BUILDER_HPP

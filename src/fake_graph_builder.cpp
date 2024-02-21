#include "fake_graph_builder.hpp"
#include <ros/console.h>
#include <iomanip>
#include <iostream>
#include <cmath> // Include cmath for sqrt
#include <utility> // Include utility for std::pair

Edge::Edge(Node* from, Node* to, std::string type)
    : from(from), to(to), type(type) {
    calculateEdgeLength(); // This will set the length based on from and to nodes
}

void Edge::calculateEdgeLength() {
    this->length = sqrt(pow(from->x - to->x, 2) + pow(from->y - to->y, 2) + pow(from->z - to->z, 2));
}

Node::Node(int id, float x, float y, float z)
    : id(id), mappedId(-1), x(x), y(y), z(z), parent(nullptr) {}

void Node::addChild(Node* child) {
    children.push_back(child);
}

FakeGraphBuilder::FakeGraphBuilder(ros::NodeHandle& node) : m_node(node)
{
    // Initialize the graph matcher
    init();
}

void FakeGraphBuilder::init()
{
    ROS_INFO("[INFO] FakeGraphBuilder initialization");    

    // Initialize the publisher(s)
    m_pub = m_node.advertise<std_msgs::Float32MultiArray>("/graph_building/fake/adjency_matrix",1000);
}

FakeGraphBuilder::~FakeGraphBuilder() {
    for (auto node : nodes) delete node;
    for (auto edge : edges) delete edge;
}

void FakeGraphBuilder::addNode(Node* node) {
    nodes.push_back(node);
}

void FakeGraphBuilder::addEdge(Edge* edge) {
    edges.push_back(edge);
}

void FakeGraphBuilder::calculateAdjacencyMatrix(std::vector<std::vector<int>>& adjacencyMatrix, bool sym) {
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
            ROS_ERROR("Error: Nodes not found in the node mapping.");
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

void FakeGraphBuilder::publishAdjacencyMatrix(const std::vector<std::vector<int>>& adjacencyMatrix) {
    ROS_INFO("Publishing adjacency matrix...");

    int size = adjacencyMatrix.size();
    std_msgs::Float32MultiArray matrix;
    matrix.layout.dim.push_back(std_msgs::MultiArrayDimension());
    matrix.layout.dim[0].size = size;
    matrix.layout.dim[0].stride = size * size;
    matrix.layout.dim[0].label = "rows";

    matrix.layout.dim.push_back(std_msgs::MultiArrayDimension());
    matrix.layout.dim[1].size = size;
    matrix.layout.dim[1].stride = size;
    matrix.layout.dim[1].label = "cols";

    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            matrix.data.push_back(static_cast<float>(adjacencyMatrix[i][j]));
        }
    }

    m_pub.publish(matrix);
    
    std::cout << "Adjacency Matrix: " << std::endl;
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            std::cout << std::setw(5) << adjacencyMatrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
    ROS_INFO("...Adjacency matrix published.");
}

std::unordered_map<int, int> FakeGraphBuilder::createNodeMapping() {
    std::unordered_map<int, int> mapping;
    int index = 0;
    for (const auto& node : nodes) {
        mapping[node->id] = index;
        node->mappedId = index;
        ++index;
    }
    return mapping;
}

void FakeGraphBuilder::printNodeMapping(const std::unordered_map<int, int>& nodeMapping) {
    ROS_INFO("Node Mapping:");
    for (const auto& pair : nodeMapping) {
        ROS_INFO("Original ID: %d -> Sequential Index: %d", pair.first, pair.second);
    }
}

void FakeGraphBuilder::createFakeGraph() {
    // Create a fake graph

    // Nodes
    Node* node0 = new Node(0, 0, 0, 0);
    Node* node1 = new Node(1, 0, 1, 0);
    Node* node11 = new Node(11, -1, 1, 0);
    Node* node111 = new Node(111, -2, 2, 0);
    Node* node1111 = new Node(1111, 0, 0, 0);
    Node* node1112 = new Node(1112, 0, 0, 0);
    Node* node12 = new Node(12, 1, 0, 0);
    Node* node121 = new Node(121, 1, 0, 0);
    Node* node13 = new Node(13, 1, 0, 0);

    // Add nodes to FakeGraphBuilder
    this->addNode(node0);
    this->addNode(node1);
    this->addNode(node11);
    this->addNode(node111);
    this->addNode(node1111);
    this->addNode(node1112);
    this->addNode(node12);
    this->addNode(node121);
    this->addNode(node13);

    // Add edges
    this->addEdge(new Edge(node0, node1, "trunk"));
    this->addEdge(new Edge(node1, node11, "branch11"));
    this->addEdge(new Edge(node11, node111, "branch111"));
    this->addEdge(new Edge(node111, node1111, "branch1111"));
    this->addEdge(new Edge(node111, node1112, "branch1112"));
    this->addEdge(new Edge(node1, node12, "branch12"));
    this->addEdge(new Edge(node12, node121, "branch121"));
    this->addEdge(new Edge(node1, node13, "branch13"));

    // Create node mapping
    std::unordered_map<int, int> nodeMapping = this->createNodeMapping();

    // Print node mapping
    this->printNodeMapping(nodeMapping);

    // Calculate and print adjacency matrix
    std::vector<std::vector<int>> adjacencyMatrix;
    this->calculateAdjacencyMatrix(adjacencyMatrix);
    this->publishAdjacencyMatrix(adjacencyMatrix);
}

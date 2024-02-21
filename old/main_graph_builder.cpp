#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"

#include <iostream>

#include "graph_builder.hpp"
#include "graph_edge.hpp"
#include "graph_node.hpp"


int main(int argc, char **argv) {

    // Initialize the ROS node
    ros::init(argc, argv, "graph_builder_node");

    // Create a node handle
    ros::NodeHandle n;
    /**
     * @brief Publisher for the "chatter" topic.
     */
    ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("chatter", 1000);
    ros::Rate loop_rate(1); // 1 Hz

    ///////CODE SNIPPET///////

    // Create a new graph
    GraphBuilder g;

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

    // Add nodes to GraphBuilder
    g.addNode(node0);
    g.addNode(node1);
    g.addNode(node11);
    g.addNode(node111);
    g.addNode(node1111);
    g.addNode(node1112);
    g.addNode(node12);
    g.addNode(node121);
    g.addNode(node13);

    // Add edges
    g.addEdge(new Edge(node0, node1, "trunk"));
    g.addEdge(new Edge(node1, node11, "branch11"));
    g.addEdge(new Edge(node11, node111, "branch111"));
    g.addEdge(new Edge(node111, node1111, "branch1111"));
    g.addEdge(new Edge(node111, node1112, "branch1112"));
    g.addEdge(new Edge(node1, node12, "branch12"));
    g.addEdge(new Edge(node12, node121, "branch121"));
    g.addEdge(new Edge(node1, node13, "branch13"));

    // Create node mapping
    std::unordered_map<int, int> nodeMapping = g.createNodeMapping();

    // Print node mapping
    g.printNodeMapping(nodeMapping);

    // Calculate and print adjacency matrix
    std::vector<std::vector<int>> adjacencyMatrix;
    g.calculateAdjacencyMatrix(adjacencyMatrix);
    g.printAdjacencyMatrix(adjacencyMatrix);

    /////////////////////////

    while (ros::ok())
    {
        std::cout<<"Publishing adjacency matrix..."<<std::endl;

        int size = adjacencyMatrix.size(); // Taille de la matrice d'adjacence
        std_msgs::Float32MultiArray matrix;
        matrix.layout.dim.push_back(std_msgs::MultiArrayDimension());
        matrix.layout.dim[0].size = size; // Nombre de lignes
        matrix.layout.dim[0].stride = size * size; // Total des éléments
        matrix.layout.dim[0].label = "rows";

        matrix.layout.dim.push_back(std_msgs::MultiArrayDimension());
        matrix.layout.dim[1].size = size; // Nombre de colonnes
        matrix.layout.dim[1].stride = size; // Elements par ligne
        matrix.layout.dim[1].label = "cols";

        // Convertir la matrice d'adjacence en un vector linéaire
        for (int i = 0; i < size; ++i) {
            for (int j = 0; j < size; ++j) {
                matrix.data.push_back(static_cast<float>(adjacencyMatrix[i][j]));
            }
        }

        // Publish the adjacency matrix
        chatter_pub.publish(matrix);
        std::cout<<"Published adjacency matrix."<<std::endl;

        // Sleep to maintain the desired loop rate
        loop_rate.sleep();
    }

    return 0;
}

#include "graph_matcher.hpp"
#include <ros/ros.h>

GraphMatcher::GraphMatcher(ros::NodeHandle& node, const bool fake) : m_node(node), m_fake(fake)
{
    // Initialize the graph matcher
    init();
}

void GraphMatcher::init()
{
    ROS_INFO("[INFO] GraphMatcher initialization");    

    // Initialize the publisher(s)
    //m_pub = m_node.advertise<your_graph_msgs::Graph>("/", 1);

    // Initialize the subscriber(s)
    if (m_fake) {
        m_sub = m_node.subscribe("graph_building/fake/adjency_matrix", 1, &GraphMatcher::callback, this);
    } else {
        m_sub = m_node.subscribe("graph_building/adjency_matrix", 1, &GraphMatcher::callback, this);
    }
}

void GraphMatcher::callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    // Logging
    ROS_INFO("[INFO] GraphMatcher callback");
    int rows = msg->layout.dim[0].size;
    int cols = msg->layout.dim[1].size;
    ROS_INFO_STREAM(rows << "x" << cols << " Adjacency Matrix received.");

    const std::vector<float>& data = msg->data;

    // Rebuild the shared matrix
    std::vector<std::vector<float>> adjMatrix = rebuildMatrix(rows, cols, data);
    
    // Compute the graph matcher
    computeGraphMatcher();

    // Visualize the graph matcher
    visualizeGraphMatcher();
}

std::vector<std::vector<float>> GraphMatcher::rebuildMatrix(int rows, int cols, const std::vector<float>& data)
{
    // Rebuild the matrix from the Float32MultiArray message

    if (rows * cols != data.size()) {
        ROS_ERROR("[ERROR] Data size does not match matrix dimensions!");
        return std::vector<std::vector<float>>();
    }

    // Initialize the matrix with the specified dimensions
    std::vector<std::vector<float>> adjMatrix(rows, std::vector<float>(cols, 0.0));

    // Copy data to the matrix
    int k = 0;
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            adjMatrix[i][j] = data[k++];
        }
    }

    return adjMatrix;
}

void GraphMatcher::computeGraphMatcher()
{
    // Compute the graph matcher
    // ...
}

void GraphMatcher::visualizeGraphMatcher()
{
    // Visualize the graph matcher
    // ...
}

#include "graph_matcher.hpp"

#include "std_msgs/Float32MultiArray.h"

GraphMatcher::GraphMatcher(ros::NodeHandle& node) : m_node(node)
{
    // Initialize the graph matcher
    init();
}

void GraphMatcher::init()
{
    ROS_INFO("[INFO] GraphMatcher initialization");    

    // Initialize the publisher(s)
    m_pub = m_node.advertise<your_graph_msgs::Graph>("/", 1);

    // Initialize the subscriber(s)
    m_sub = m_node.subscribe("graph_building/adjency_matrix", std_msgs::Float32MultiArray::adjMatrixMsg, &GraphMatcher::callback, this);
}

void GraphMatcher::callback(const std_msgs::Float32MultiArray::adjMatrixMsg& msg)
{
    // Logging
    ROS_INFO("[INFO] GraphMatcher callback");
    rows = msg.layout.dim[0].size;
    cols = msg.layout.dim[1].size;
    rospy.loginfo("Matrice Adj " + std::to_string(rows) + "x" + std::to_string(cols) + " reçue.");

    // Rebuild the shared matrix
    rebuildMatrix(rows, cols, msg.data);
    
    // Compute the graph matcher
    computeGraphMatcher();

    // Visualize the graph matcher
    visualizeGraphMatcher();
}

void GraphMatcher::rebuildMatrix(rows, cols, msg.data)
{
    // Rebuild the matrix from the Float32MultiArray message
    // ...
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

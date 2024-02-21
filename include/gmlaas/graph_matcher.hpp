#ifndef GRAPH_MATCHER_HPP
#define GRAPH_MATCHER_HPP

#include <ros/ros.h>
#//include <your_graph_msgs/Graph.h> 


class GraphMatcher
{
public:
    GraphMatcher(ros::NodeHandle& m_node);
    void init();
    void callback();
    void resizeAdjacencyMatrix();
    void computeGraphMatcher();
    void visualizeGraphMatcher();

private:
    ros::NodeHandle& m_node;
    ros::Publisher m_pub;
    ros::Subscriber m_sub;
    message_filters::Subscriber<sensor_msgs::Image> m_color_image_sub;
    message_filters::Subscriber<sensor_msgs::Image> m_depth_image_sub;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> m_image_sync;

};

#endif // GRAPH_MATCHER_HPP

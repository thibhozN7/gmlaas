#include "std_msgs/Float32MultiArray.h"
#include <ros/ros.h>
#include <vector>


class GraphMatcher {
public:
    GraphMatcher(ros::NodeHandle& node);
    void init();
    void callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    std::vector<std::vector<float>> rebuildMatrix(int rows, int cols, const std::vector<float>& data);
    void computeGraphMatcher();
    void visualizeGraphMatcher();

private:
    ros::NodeHandle m_node;
    ros::Publisher m_pub;
    ros::Subscriber m_sub;
};

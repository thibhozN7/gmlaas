#pragma once

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>
#include <rosgraph_msgs/Clock.h> // Include the necessary header file

#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/core/vpCameraParameters.h>
// #include <visp_ros/vpROSRobot.h> 
#include <visp3/vs/vpAdaptiveGain.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>

#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

struct Difference {
    double distance;
    double orientation_difference;
};

class VisualServoingCommand /*: public vpROSRobot*/ 
{

public:

    VisualServoingCommand(ros::NodeHandle& node);
    void computeCommandCallbackPbvs(const std_msgs::Float32MultiArray& msg);
    void init();
    void publishVelocity(const vpColVector& vel);    
    void stop_visual_servoing();
    Difference difference_between_matrices(vpHomogeneousMatrix& matrix1, vpHomogeneousMatrix& matrix2);
    bool stop_condition_satisfied (vpHomogeneousMatrix& current_homo_matrix, vpHomogeneousMatrix& desire_homo_matrix);
    
    ros::Publisher m_vel_pub;
    geometry_msgs::TwistStamped m_vel_twist_stamped;
    bool m_interrupt_flag = true;

private:
    ros::NodeHandle& m_node;
    vpColVector     m_vel;
    ros::Subscriber m_sub;
    vpPoint m_point[4];
    vpServo m_task;
    vpFeaturePoint m_s[4];
    vpFeaturePoint m_s_desire[4];
    //vpHomogeneousMatrix previous_homo_matrix;
    std::string m_vs_type;
    bool m_flag;
    bool m_visp_publisher;
    bool m_adaptative_gain;
    bool m_save_velocity = false;
    double m_lambda_gain;
    double m_x_gain;
    double m_y_gain; 
    double m_z_gain;
    double m_rx_gain;
    double m_ry_gain;
    double m_rz_gain;
    
    float threshold_distance;
    float threshold_orientation;
    Difference desired_diff;
    Difference variation_diff;
};

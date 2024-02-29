#pragma once

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>

//#include "visual_servoing_realsense_visp/point_3d.h"

#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/core/vpCameraParameters.h>
// #include <visp_ros/vpROSRobot.h> 
#include <visp3/vs/vpAdaptiveGain.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>

#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

const std::vector<double> K = {605.132568359375, 0.0, 322.9114074707031,
			              0.0, 604.875244140625, 239.2445526123047,
			              0.0, 0.0, 1.0};

class VisualServoingCommand /*: public vpROSRobot*/ 
{

public:
    // VisualServoingCommand(int argc, char **argv);
    VisualServoingCommand(ros::NodeHandle& node);
    //void computeCommandCallbackIbvs(const visual_servoing_realsense_visp::point_3dConstPtr& msg);
    void computeCommandCallbackPbvs(const std_msgs::Float32MultiArray& msg);
    // void init(int argc, char **argv);
    void init();
    //bool point3dMsgIsNan(const visual_servoing_realsense_visp::point_3dConstPtr& msg);
    void publishVelocity(const vpColVector& vel);
    void velCommandCallback(const geometry_msgs::TwistStampedConstPtr& vel_command_msg);

    ros::Publisher m_vel_pub;
    geometry_msgs::TwistStamped m_vel_twist_stamped;
    bool m_interrupt_flag = true;

private:
    ros::NodeHandle& m_node;
    vpColVector     m_vel;
    ros::Subscriber m_points_sub;
    ros::Subscriber m_vel_command;
    vpPoint m_point[4];
    vpServo m_task;
    vpFeaturePoint m_s[4];
    vpFeaturePoint m_s_desire[4];
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
    std::fstream m_file;
    std::fstream m_vel_file;
};

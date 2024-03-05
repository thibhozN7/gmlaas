#include "visual_servoing_command.hpp"
#include "params.hpp"

#include <geometry_msgs/TwistStamped.h>
#include <cv_bridge/cv_bridge.h>

#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>
#include <visp/vpImageConvert.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/core/vpMouseButton.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/vs/vpAdaptiveGain.h>
#include <visp/vpQuaternionVector.h>
#include <visp/vpMath.h>

#include <iostream>


VisualServoingCommand::VisualServoingCommand(ros::NodeHandle& node):m_node{node}{
    init();
}
 
void VisualServoingCommand::init(){
    
    m_vel_pub = m_node.advertise<geometry_msgs::TwistStamped>("/visual_servoing_command/velocity_ref", 1, false);
    
    //Eye in hand visual servoing task initialization (camera in the hand of the robot)
    //effectors are the camera and the robot is the object
    m_task.setServo(vpServo::EYEINHAND_CAMERA);
    //type matrix interaction configuration
    //CURRENT : the interaction matrix is computed using the current visual features
    m_task.setInteractionMatrixType(vpServo::CURRENT);

    m_vs_type="pbvs";
    std::cout << "visual servoing mode: PBVS" << std::endl;

    m_sub = m_node.subscribe("/h_computation/h_matrix", 1,
                                    &VisualServoingCommand::computeCommandCallbackPbvs, this);
    
    // Fixed or dynamic gain used to adjust the control law in minimizing the error.
    ros::param::get("adaptative_gain_value",  m_adaptative_gain);
    ros::param::get("lamda_gain_value",  m_lambda_gain);


    if(m_adaptative_gain){
        m_task.setLambda(vpAdaptiveGain());
        std::cout << "Adaptive Gain initialization" << std::endl;
    }else{
        m_task.setLambda(0.2);
        std::cout << "Lambda gain initialization :" << m_lambda_gain << std::endl;
    }

    threshold_distance = 0.1; //10 centimeters
    threshold_orientation = 5; //5 degres 
    
    this-> desired_diff = {threshold_distance, threshold_orientation};

    //vpHomogeneousMatrix previous_homo_matrix;

    //Initializing the velocity command to zero in all directions.
    m_vel_twist_stamped.twist.linear.x = 0;
    m_vel_twist_stamped.twist.linear.y = 0;
    m_vel_twist_stamped.twist.linear.z = 0;
    m_vel_twist_stamped.twist.angular.x = 0;
    m_vel_twist_stamped.twist.angular.y = 0;
    m_vel_twist_stamped.twist.angular.z = 0;

    ros::param::get("x_gain_value",  m_x_gain);
    ros::param::get("y_gain_value",  m_y_gain);
    ros::param::get("z_gain_value",  m_z_gain);
    ros::param::get("rx_gain_value",  m_rx_gain);
    ros::param::get("ry_gain_value",  m_ry_gain);
    ros::param::get("rz_gain_value",  m_rz_gain);


    std::cout << "VisualServoingCommand Initialization Done" << std::endl;
}

void VisualServoingCommand::computeCommandCallbackPbvs(const std_msgs::Float32MultiArray& msg){
    std::cout << "PBVS callback" << std::endl;
    vpHomogeneousMatrix current_homo_matrix{msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                                            msg.data[4], msg.data[5], msg.data[6], msg.data[7],
                                            msg.data[8], msg.data[9], msg.data[10], msg.data[11],
                                            msg.data[12], msg.data[13], msg.data[14], msg.data[15]};

    vpHomogeneousMatrix desire_homo_matrix{1, 0, 0, 0,
                                           0, 1, 0, 0,
                                           0, 0, 1, 0,
                                           0, 0, 0, 1};

    // check stop conditions
    if (stop_condition_satisfied(current_homo_matrix, desire_homo_matrix)) {
        ROS_INFO("Stop criterion reached...");
        stop_visual_servoing(); // nul vel command
    } 
    else {
        //updating that stop cond has been passed 
        //previous_homo_matrix = current_homo_matrix;

        // caracteristics extraction : translation (t) and rotation (tu)
        vpFeatureTranslation t(vpFeatureTranslation::cdMc);
        vpFeatureThetaU tu(vpFeatureThetaU::cdRc);
        t.buildFrom(current_homo_matrix);
        tu.buildFrom(current_homo_matrix);
    
        // definition of desired caracteristics 
        vpFeatureTranslation td(vpFeatureTranslation::cdMc);
        vpFeatureThetaU tud(vpFeatureThetaU::cdRc);
        td.buildFrom(desire_homo_matrix);
        tud.buildFrom(desire_homo_matrix);

        m_task.addFeature(t, td);
        m_task.addFeature(tu, tud);
        m_task.setServo(vpServo::EYEINHAND_CAMERA); //same as in init()
        m_task.setInteractionMatrixType(vpServo::CURRENT); //same as in init()
        
        // Velocity command is computed there
        m_vel = m_task.computeControlLaw(); 

        publishVelocity();
    }
}

void VisualServoingCommand::publishVelocity(){

    //Setting linear and angular velocity components to the end effector
    // Gains adjust the effect enforced by the velocity command
    m_vel_twist_stamped.twist.linear.x = m_x_gain*m_vel[0];
    m_vel_twist_stamped.twist.linear.y = m_y_gain*m_vel[1];
    m_vel_twist_stamped.twist.linear.z = m_z_gain*m_vel[2];
    m_vel_twist_stamped.twist.angular.x = m_rx_gain*m_vel[3];
    m_vel_twist_stamped.twist.angular.y = m_ry_gain*m_vel[4];
    m_vel_twist_stamped.twist.angular.z = m_rz_gain*m_vel[5];
    
    //Publishing to /visual_servoing_command/velocity_ref topic
    m_vel_pub.publish(m_vel_twist_stamped);
}

Difference VisualServoingCommand::difference_between_matrices(vpHomogeneousMatrix& matrix1, vpHomogeneousMatrix& matrix2) {
    // // Convert the homogeneous matrices to quaternion vectors
    // vpQuaternionVector orientation1, orientation2;
    // orientation1.buildFrom(matrix1);
    // orientation2.buildFrom(matrix2);

    // // Calculate the orientation difference in degrees
    // double orientation_difference = vpMath::deg(orientation1, orientation2);
    double orientation_difference = 0;

    // Calculate the distance between translations
    vpTranslationVector translation1, translation2;
    translation1.buildFrom(matrix1);
    translation2.buildFrom(matrix2);
    double distance = std::sqrt((translation1 - translation2).sumSquare());

    Difference difference;
    difference.orientation_difference = orientation_difference;
    difference.distance = distance;

    return difference;
}


bool VisualServoingCommand::stop_condition_satisfied (vpHomogeneousMatrix& current_homo_matrix, vpHomogeneousMatrix& desire_homo_matrix) {
    bool stop;
    //Difference c_diff = difference_between_matrices(current_homo_matrix, previous_homo_matrix);; // checking previous with current frame diff for blocked camera case
    Difference d_diff = difference_between_matrices(current_homo_matrix, desire_homo_matrix);; // checking current with desired frame diff for convergence case

    if (d_diff.distance <= this-> desired_diff.distance){
        //if (std::abs(d_diff.orientation_difference) <= std::abs(this->desired_diff.orientation_difference)){stop = true;}
        stop = true;
    }
    // if (c_diff.distance <= this-> variation_diff.distance){stop = true};
    else {stop = false;}
    
    return stop;
}

void VisualServoingCommand::stop_visual_servoing() {

    m_vel_twist_stamped.twist.linear.x = 0;
    m_vel_twist_stamped.twist.linear.y = 0;
    m_vel_twist_stamped.twist.linear.z = 0;
    m_vel_twist_stamped.twist.angular.x = 0;
    m_vel_twist_stamped.twist.angular.y = 0;
    m_vel_twist_stamped.twist.angular.z = 0;
    ROS_INFO("...servoing ended");

    m_vel_pub.publish(m_vel_twist_stamped);
}

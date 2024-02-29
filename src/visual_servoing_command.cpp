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


VisualServoingCommand::VisualServoingCommand(ros::NodeHandle& node):m_node{node}{
    init();
}
 
void VisualServoingCommand::init(){
    
    m_vel_pub = m_node.advertise<geometry_msgs::TwistStamped>("/visual_servoing_command/velocity_ref", 1, true);
    m_vel_command = m_node.subscribe("/vel_command", 1,
                                         &VisualServoingCommand::velCommandCallback, this);
    
    //Eye in hand visual servoing task initialization (camera in the hand of the robot)
    //effectors are the camera and the robot is the object
    m_task.setServo(vpServo::EYEINHAND_CAMERA);
    //type matrix interaction configuration
    //CURRENT : the interaction matrix is computed using the current visual features
    m_task.setInteractionMatrixType(vpServo::CURRENT);



    if(m_vs_type == "pbvs"){
        m_points_sub = m_node.subscribe("/h_computation/h_matrix", 1,
                                         &VisualServoingCommand::computeCommandCallbackPbvs, this);
        // m_points_sub = vpROSRobot::n->subscribe("/camera_frame/point_cloud_detection/icp_homo_matrix", 1000, &VisualServoingCommand::computeCommandCallbackPbvs, this);
        ROS_INFO("visual sevoing mode: PBVS");
    }
    else{
        // m_points_sub = vpROSRobot::n->subscribe("/camera_frame/detection/3d_point", 1000, &VisualServoingCommand::computeCommandCallbackIbvs, this);
        ROS_INFO("visual sevoing mode: IBVS");
    }

    // Fixed or dynamic gain used to adjust the control law in minimizing the error.
    if(m_adaptative_gain){
        m_task.setLambda(vpAdaptiveGain());
        std::cout << "Adaptive Gain initialization" << std::endl;
    }else{
        m_task.setLambda(m_lambda_gain);
        std::cout << "Lambda gain initialization" << m_lambda_gain << std::endl;
    }

    m_flag = true; // for IBVS command computation

    // Initializing the velocity command to zero in all directions.
    m_vel_twist_stamped.twist.linear.x = 0;
    m_vel_twist_stamped.twist.linear.y = 0;
    m_vel_twist_stamped.twist.linear.z = 0;
    m_vel_twist_stamped.twist.angular.x = 0;
    m_vel_twist_stamped.twist.angular.y = 0;
    m_vel_twist_stamped.twist.angular.z = 0;

    std::cout << "VisualServoingCommand Initialization Done" << std::endl;

}

void VisualServoingCommand::computeCommandCallbackPbvs(const std_msgs::Float32MultiArray& msg){

    vpHomogeneousMatrix current_homo_matrix{msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                                            msg.data[4], msg.data[5], msg.data[6], msg.data[7],
                                            msg.data[8], msg.data[9], msg.data[10], msg.data[11],
                                            msg.data[12], msg.data[13], msg.data[14], msg.data[15]};

    vpHomogeneousMatrix desire_homo_matrix{1, 0, 0, 0,
                                           0, 1, 0, 0,
                                           0, 0, 1, 0,
                                           0, 0, 0, 1};
    
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
    
    //Setting linear and angular velocity components to the end effector
    // Gains adjust the effect enforced by the velocity command
    m_vel_twist_stamped.twist.linear.x = m_x_gain*m_vel[0]; 
    m_vel_twist_stamped.twist.linear.y = m_y_gain*m_vel[1];
    m_vel_twist_stamped.twist.linear.z = m_z_gain*m_vel[2];
    m_vel_twist_stamped.twist.angular.x = m_rx_gain*m_vel[3]; //roll
    m_vel_twist_stamped.twist.angular.y = m_ry_gain*m_vel[4]; //pitch
    m_vel_twist_stamped.twist.angular.z = m_rz_gain*m_vel[5]; //yaw

    //Publishing to /visual_servoing_command/velocity_ref topic
    m_vel_pub.publish(m_vel_twist_stamped);
}

void VisualServoingCommand::publishVelocity(const vpColVector& vel){
    if(m_visp_publisher){
        // vpROSRobot::setVelocity(vpRobot::REFERENCE_FRAME, vel);
        ROS_INFO("Not Implemented For The Moment");
    }else{
        geometry_msgs::TwistStamped vel_twist_stamped;
        vel_twist_stamped.twist.linear.x = m_x_gain*vel[0];
        vel_twist_stamped.twist.linear.y = m_y_gain*vel[1];
        vel_twist_stamped.twist.linear.z = m_z_gain*vel[2];
        vel_twist_stamped.twist.angular.x = m_rx_gain*vel[3];
        vel_twist_stamped.twist.angular.y = m_ry_gain*vel[4];
        vel_twist_stamped.twist.angular.z = m_rz_gain*vel[5];

        m_vel_pub.publish(vel_twist_stamped);
        
        // ros::param::get("save_velocity_value", m_save_velocity);
        // if(m_save_velocity){
        //     // ROS_INFO("save_velocity_value");
        //     m_file.open(global::data_vel_file_path, std::ios_base::app | std::ios::ate);
        //     if(!m_file.is_open())
        //     {
        //         ROS_ERROR("Unable to open velocity data file \n");
        //     }
        //     const std::string position_string =
        //                                 std::to_string(vel_twist_stamped.twist.linear.x) +"\t"+
        //                                 std::to_string(vel_twist_stamped.twist.linear.y) +"\t"+
        //                                 std::to_string(vel_twist_stamped.twist.linear.z) +"\t"+
        //                                 std::to_string(vel_twist_stamped.twist.angular.x) +"\t"+
        //                                 std::to_string(vel_twist_stamped.twist.angular.y) +"\t"+
        //                                 std::to_string(vel_twist_stamped.twist.angular.z) +"\t"+
        //                                 std::to_string(ros::Time::now().toSec()) +"\n";
        //     m_file << position_string;
        //     m_file.close();
        // }
    }
}

// bool VisualServoingCommand::point3dMsgIsNan(const visual_servoing_realsense_visp::point_3dConstPtr& msg){

//     if(msg){
//         for(const auto& p : msg->p_3d){
//             if(!(isnan(p.x) && isnan(p.y) && isnan(p.z))){
//                 return false;
//             }
//         }
//         return true;
//     }else{
//         return true;
//     }
// }    

void VisualServoingCommand::velCommandCallback(const geometry_msgs::TwistStampedConstPtr& vel_command_msg){
    
    m_vel_file.open(global::data_vel_command_file_path, std::ios_base::app | std::ios::ate);
                
    if(!m_vel_file.is_open()){
        std::cout<<"Unable to open the file.\n";
    }
        
    std::string myStrvel = std::to_string(vel_command_msg->twist.linear.x) +"\t"+
                           std::to_string(vel_command_msg->twist.linear.y) +"\t"+
                           std::to_string(vel_command_msg->twist.linear.z) +"\t"+
                           std::to_string(vel_command_msg->twist.angular.x) +"\t"+
                           std::to_string(vel_command_msg->twist.angular.y) +"\t"+
                           std::to_string(vel_command_msg->twist.angular.z) +"\t"+
                           std::to_string(ros::Time::now().toSec()) +"\n";
    m_vel_file<<myStrvel;
    m_vel_file.close();
}

// void VisualServoingCommand::computeCommandCallbackIbvs(const visual_servoing_realsense_visp::point_3dConstPtr& msg){

//     if(point3dMsgIsNan(msg)){
//         vpROSRobot::setVelocity(vpRobot::REFERENCE_FRAME, vpColVector(6, 0.0));
// 	}else{
//         try {
//             for(size_t i =0; i<4; i++){
//                 m_point[i].set_X(msg->p_3d[i].x);
//                 m_point[i].set_Y(msg->p_3d[i].y);
//                 m_point[i].set_Z(msg->p_3d[i].z);
//                 m_point[i].set_W(1);
//                 m_point[i].projection();
//             }

//             if(m_flag){
//                 for(size_t i = 0; i<4; i++){
//                     vpFeatureBuilder::create(m_s_desire[i], m_point[i]);
//                 }
//                 m_flag=false;
//             }
//             else{
//                 for(size_t i =0; i<4; i++){
//                     vpFeatureBuilder::create(m_s[i], m_point[i]);
//                 }
//             }

//             for(size_t i = 0; i<4; i++){
//                 m_task.addFeature(m_s[i], m_s_desire[i]);
//             }

//             m_vel = m_task.computeControlLaw();
//             publishVelocity(m_vel);

//         }catch(const vpException &e){
//             std::cout << "Catch an exception: " << e << std::endl;
//         }
//     }
// }

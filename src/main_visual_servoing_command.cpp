#include <visual_servoing_command.hpp>
#include <ros/master.h>
#include <unistd.h>


int main(int argc, char **argv){
  ros::init(argc, argv, "visual_servoing_command");
  
  ros::NodeHandle visual_servoing_command_node;
  
  VisualServoingCommand visual_servoing_command(visual_servoing_command_node);
  // VisualServoingCommand visual_servoing_command(argc, argv);

  ros::spin();
  return 0;
}
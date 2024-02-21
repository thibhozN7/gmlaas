#include <ros/master.h>
#include <unistd.h>

#include "graph_matcher.hpp"


int main(int argc, char **argv){
  
  ros::init(argc, argv, "main_graph_matcher");
  ros::NodeHandle graph_matcher_node;
  
  GraphMatcher graph_matcher(graph_matcher_node);

  ros::spin();
  return 0;
}
cmake_minimum_required(VERSION 2.8.3)
project(gmlaas)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  apriltag_ros
)

add_message_files(
  FILES
  GraphBuilderMsg.msg
  GraphMatcherMsg.msg
)
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )
 
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

set(gmlaas_INCLUDE_DIR "include/gmlaas")
set(gmlaas_SOURCE_DIR "src")
#set(gmlaas_BINARY_DIR "output")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${gmlaas_INCLUDE_DIR}
)

# graph_matcher_node
add_executable(graph_matcher_node
  ${gmlaas_SOURCE_DIR}/main_graph_matcher.cpp
  ${gmlaas_SOURCE_DIR}/graph_matcher.cpp
)
target_link_libraries(graph_matcher_node ${catkin_LIBRARIES})

# fake_graph_builder_node
add_executable(fake_graph_builder_node
  ${gmlaas_SOURCE_DIR}/main_fake_graph_builder.cpp
  ${gmlaas_SOURCE_DIR}/fake_graph_builder.cpp
)
target_link_libraries(fake_graph_builder_node ${catkin_LIBRARIES})

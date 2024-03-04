cmake_minimum_required(VERSION 2.8.3)
project(gmlaas)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  apriltag_ros
  geometry_msgs
  cv_bridge
)
find_package(VISP REQUIRED)

add_message_files(
  FILES
  GraphBuilderMsg.msg
  PreHMsg.msg
  GraphMatcherMsg.msg
)
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )
 
catkin_package(CATKIN_DEPENDS roscpp geometry_msgs cv_bridge message_runtime std_msgs)

set(gmlaas_INCLUDE_DIR "include/gmlaas")
set(gmlaas_SOURCE_DIR "src")
#set(gmlaas_BINARY_DIR "output")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${gmlaas_INCLUDE_DIR}
  ${VISP_INCLUDE_DIRS}
)

add_executable(visual_servoing_command_node
  # ${gmlaas_SOURCE_DIR}/main_fake_graph_builder.cpp
  # ${gmlaas_SOURCE_DIR}/fake_graph_builder.cpp
  ${gmlaas_SOURCE_DIR}/main_visual_servoing_command.cpp
  ${gmlaas_SOURCE_DIR}/visual_servoing_command.cpp

)
target_link_libraries(visual_servoing_command_node ${catkin_LIBRARIES})
target_link_libraries(visual_servoing_command_node ${VISP_LIBRARIES})


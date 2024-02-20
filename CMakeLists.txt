cmake_minimum_required(VERSION 2.8.3)
project(gmlaas)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

catkin_package()

set(gmlaas_INCLUDE_DIR "include/gmlaas")
set(gmlaas_SOURCE_DIR "src")
#set(gmlaas_BINARY_DIR "output")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${gmlaas_INCLUDE_DIR}
)

# graph_builder_node
add_executable(graph_builder_node
  ${gmlaas_SOURCE_DIR}/graph_builder.cpp
  ${gmlaas_SOURCE_DIR}/graph_node.cpp
  ${gmlaas_SOURCE_DIR}/graph_edge.cpp
  ${gmlaas_SOURCE_DIR}/main_graph_builder.cpp
)

# graph_matcher_node
add_executable(graph_matcher_node
  ${gmlaas_SOURCE_DIR}/main_graph_matcher.cpp
  ${gmlaas_SOURCE_DIR}/graph_matcher.cpp
)

# reading_tags_node
add_executable(reading_tags_node
  ${gmlaas_SOURCE_DIR}/main_tag_reader.cpp
  ${gmlaas_SOURCE_DIR}/tag_reader.cpp
)


target_link_libraries(graph_builder_node ${catkin_LIBRARIES})
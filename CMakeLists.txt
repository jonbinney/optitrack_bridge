cmake_minimum_required(VERSION 2.8.3)
project(optitrack_bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)

find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -lpthread -pthread")

catkin_package(
)

include_directories(include)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(optitrack_bridge_node src/optitrack_bridge_node.cpp src/optitrack_bridge.cpp)

#add_dependencies(optitrack_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(optitrack_bridge_node
  ${catkin_LIBRARIES}
)



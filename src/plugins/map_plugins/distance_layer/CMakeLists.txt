cmake_minimum_required(VERSION 3.0.2)
project(distance_layer)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  nav_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES distance_layer
  CATKIN_DEPENDS costmap_2d dynamic_reconfigure nav_msgs roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/distance_layer.cpp src/euclidean_distance_field.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

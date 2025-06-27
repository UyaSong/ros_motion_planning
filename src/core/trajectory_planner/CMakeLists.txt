cmake_minimum_required(VERSION 3.0.2)
project(trajectory_planner)

get_filename_component(PROJECT_ROOT_DIR ${CMAKE_SOURCE_DIR} DIRECTORY)
include(${PROJECT_ROOT_DIR}/3rd/conanbuildinfo.cmake)
conan_basic_setup()

find_package(catkin REQUIRED COMPONENTS
  angles
  roscpp
  costmap_2d
  distance_layer
  common
  path_processor
)

find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS distance_layer common path_processor
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# optimization
file(GLOB trajectory_optimization_src_files src/trajectory_optimization/*.cpp)
file(GLOB trajectory_conjugate_optimizer_src_files src/trajectory_optimization/conjugate_optimizer/*.cpp)
file(GLOB trajectory_lbfgs_optimizer_src_files src/trajectory_optimization/lbfgs_optimizer/*.cpp)
file(GLOB trajectory_constrained_optimizer_src_files src/trajectory_optimization/constrained_optimizer/*.cpp)
file(GLOB trajectory_minimumsnap_optimizer_src_files src/trajectory_optimization/minimumsnap_optimizer/*.cpp)

file(GLOB trajectory_generation_src_files src/trajectory_generation/*.cpp)

# trajectory optimization
add_library(trajectory_optimization
  ${trajectory_optimization_src_files}
  ${trajectory_conjugate_optimizer_src_files}
  ${trajectory_constrained_optimizer_src_files}
  ${trajectory_minimumsnap_optimizer_src_files}
  ${trajectory_lbfgs_optimizer_src_files}
)

target_include_directories(trajectory_optimization PUBLIC
  ${CONAN_INCLUDE_DIRS}
)

target_link_libraries(trajectory_optimization
  ${catkin_LIBRARIES}
  ${CONAN_LIBS}
)

# trajectory generation
add_library(trajectory_generation
  ${trajectory_generation_src_files}
)

target_include_directories(trajectory_generation PUBLIC
  ${CONAN_INCLUDE_DIRS}
)

target_link_libraries(trajectory_generation
  ${catkin_LIBRARIES}
  ${CONAN_LIBS}
)

set(BUILDING_WITH_TEST Off)
if (${BUILDING_WITH_TEST})
  add_subdirectory(test)
endif()
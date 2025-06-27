cmake_minimum_required(VERSION 3.0.2)
project(polygon_simulation)

# find dependencies
find_package(catkin REQUIRED COMPONENTS
  rviz
  geometry_msgs
  std_msgs
  pluginlib
)

set(CMAKE_AUTOMOC ON)
SET(CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake")
find_package(Qt5 COMPONENTS Core Gui Widgets UiTools REQUIRED)
QT5_WRAP_CPP(QT_HEADERS ${MOC_HEADERS})
set(QT_LIBRARIES Qt5::Widgets)


add_message_files(
  FILES
  Polygon.msg
  PolygonStamped.msg
)

add_service_files(
  FILES
  GetSelection.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

set(SOURCE_FILES
  src/polygon_simulation/polygon_simulation.cpp
  src/polygon_simulation/rviz_polygon.cpp
  src/polygon_simulation/rviz_polygon_node.cpp
  src/polygon_simulation/rviz_rectangle.cpp
  src/polygon_simulation/rviz_circle.cpp
  src/polygon_simulation/rviz_line_segment.cpp
  src/polygon_simulation/rviz_discs_model.cpp
  ${MOC_FILES}
)

set(HEADER_FILES
   include/polygon_simulation_utils/polygon_simulation_utils.h
   include/polygon_simulation/polygon_simulation.h
   include/polygon_simulation/rviz_polygon.h
   include/polygon_simulation/rviz_polygon_node.h
   include/polygon_simulation/rviz_rectangle.h
   include/polygon_simulation/rviz_circle.h
   include/polygon_simulation/rviz_line_segment.h
   include/polygon_simulation/rviz_discs_model.h
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${SOURCE_FILES}
  ${HEADER_FILES}
)

target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons
)

install(FILES 
  polygon_selection_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

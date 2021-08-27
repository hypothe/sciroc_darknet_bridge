cmake_minimum_required(VERSION 3.0.2)
project(sciroc_darknet_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  darknet_ros
  darknet_ros_msgs
  roscpp
  sciroc_objdet
  geometry_msgs
  cv_bridge
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS actionlib actionlib_msgs darknet_ros darknet_ros_msgs roscpp sciroc_objdet geometry_msgs cv_bridge
  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib
  src/SciRocDarknetBridge.cpp
  src/bridge_utils.cpp
)
file(GLOB BRIDGES_CPP
  src/EnumBridge.cpp
  src/ClasBridge.cpp
  src/CompBridge.cpp
)

target_link_libraries(${PROJECT_NAME}_lib 
  ${OpenCV_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/sciroc_darknet_bridge_node.cpp ${BRIDGES_CPP})

 
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
)

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## define flag that sets whether images have to be displayed
## (and open_cv libs included)
# add_definitions(-DDISPLAY_IMAGE)
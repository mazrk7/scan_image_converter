cmake_minimum_required(VERSION 2.8.3)
project(scan_image_converter)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(OpenCV)

catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs
  DEPENDS OpenCV
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(scan_image_converter src/scan_image_converter.cpp)
target_link_libraries(scan_image_converter
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
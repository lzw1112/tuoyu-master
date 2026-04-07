cmake_minimum_required(VERSION 3.0.2)
project(camera_implementation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  autoware_config_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES deploy
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(camera_node src/camera_node.cpp)
target_link_libraries(camera_node ${catkin_LIBRARIES})

add_executable(canny_node src/canny_node.cpp)
target_link_libraries(canny_node ${catkin_LIBRARIES})

add_executable(hough_node src/hough_node.cpp)
target_link_libraries(hough_node ${catkin_LIBRARIES})

add_executable(roi_node src/roi_node.cpp)
target_link_libraries(roi_node ${catkin_LIBRARIES})

find_package(OpenCV REQUIRED)  
target_link_libraries(camera_node ${OpenCV_LIBS})

find_package(OpenCV REQUIRED)  
target_link_libraries(canny_node ${OpenCV_LIBS})

find_package(OpenCV REQUIRED)  
target_link_libraries(hough_node ${OpenCV_LIBS})

find_package(OpenCV REQUIRED)  
target_link_libraries(roi_node ${OpenCV_LIBS})

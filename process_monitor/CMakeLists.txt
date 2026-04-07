   cmake_minimum_required(VERSION 2.8.3)
   project(process_monitor)
   
   ## Find catkin and any catkin packages
   find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)
   
   ## Declare a catkin package
   catkin_package(CATKIN_DEPENDS roscpp)

   include_directories(include   
	${catkin_INCLUDE_DIRS} 
	${CMAKE_CURRENT_SOURCE_DIR}/../common/global_common/include  # 添加common/alg_interface/include的头文件目录
  )

   add_executable(process_monitor src/process_monitor.cpp)
   target_link_libraries(process_monitor ${catkin_LIBRARIES})


install(TARGETS process_monitor
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

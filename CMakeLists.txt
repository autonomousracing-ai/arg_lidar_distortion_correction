cmake_minimum_required(VERSION 2.8.3)
project(arg_lidar_distortion_correction)

add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  pcl_conversions
  message_generation
)


catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(arg_lidar_distortion_correction_ouster src/distortion_correction_ouster.cpp)
add_dependencies(arg_lidar_distortion_correction_ouster ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arg_lidar_distortion_correction_ouster ${catkin_LIBRARIES})

add_executable(arg_lidar_distortion_correction src/distortion_correction.cpp)
add_dependencies(arg_lidar_distortion_correction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arg_lidar_distortion_correction ${catkin_LIBRARIES})



#############
## Install ##
#############

install(TARGETS arg_lidar_distortion_correction arg_lidar_distortion_correction_ouster
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


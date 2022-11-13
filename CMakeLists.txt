cmake_minimum_required(VERSION 2.8.3)
project(arg_distortion_correction)

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

add_executable(arg_distortion_correction src/distortion_correction.cpp)
add_dependencies(arg_distortion_correction ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arg_distortion_correction ${catkin_LIBRARIES})

add_executable(arg_distortion_correction_general src/distortion_correction_general.cpp)
add_dependencies(arg_distortion_correction_general ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arg_distortion_correction_general ${catkin_LIBRARIES})



#############
## Install ##
#############

install(TARGETS arg_distortion_correction arg_distortion_correction_general
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


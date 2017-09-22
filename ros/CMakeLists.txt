cmake_minimum_required(VERSION 2.8.3)
project(label_fusion_ros)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  jsk_topic_tools
  nodelet
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

find_package(OpenCV REQUIRED)

find_package(OpenMP)
if(OPENMP_FOUND)
  message("OPENMP FOUND")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
add_definitions("-g -O2")
#if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
#  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -z defs")
#endif()

find_package(PCL REQUIRED COMPONENTS io)

find_package(octomap REQUIRED)
#add_definitions(-DOCTOMAP_NODEBUGOUT)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  geometry_msgs
  sensor_msgs
  DEPENDS octomap PCL
)

###########
## Build ##
###########

include_directories(
  include
  ${OCTOMAP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/mask_fusion_nodelet.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############

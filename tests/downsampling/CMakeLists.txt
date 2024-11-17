cmake_minimum_required(VERSION 3.0.2)
project(downsampling)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  roscpp
  sensor_msgs
  pcl_ros
  roscpp
)

#set (CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

#message(WARNING ${CMAKE_MODULE_PATH})

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED QUIET COMPONENTS common filters segmentation)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
 CATKIN_DEPENDS pcl_conversions roscpp sensor_msgs
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
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(example1_to_rosbag src/example1_to_rosbag.cpp)
target_link_libraries(example1_to_rosbag ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(example1_to_rosbag ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(downsample_prova_algorithm src/downsample_prova_algorithm.cpp)
target_link_libraries(downsample_prova_algorithm ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(downsample_prova_algorithm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(icp_downsample_prova src/icp_downsample_prova.cpp)
target_link_libraries(icp_downsample_prova ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(icp_downsample_prova ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

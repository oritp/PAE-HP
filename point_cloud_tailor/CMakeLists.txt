cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_tailor)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES point_cloud_tailor
   CATKIN_DEPENDS geometry_msgs nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_msgs std_srvs
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
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

  add_executable(alignment_node src/alignment.cpp)
  add_executable(trajectory_node src/trajectory.cpp)
  
## Add cmake target dependencies of the executable
  
  add_dependencies(alignment_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  add_dependencies(trajectory_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

  target_link_libraries(alignment_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  target_link_libraries(trajectory_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  

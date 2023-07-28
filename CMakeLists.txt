cmake_minimum_required(VERSION 2.8.3)
project(draw_trajectory)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)
find_package(gazebo REQUIRED)


###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
  ${GAZEBO_INCLUDE_DIRS}
)

## Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

add_library(draw_traj SHARED src/draw_traj_plugin.cc)
target_link_libraries(draw_traj ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

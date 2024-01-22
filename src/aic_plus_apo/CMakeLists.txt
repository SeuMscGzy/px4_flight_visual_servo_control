cmake_minimum_required(VERSION 3.0.2)
project(aic_plus_apo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  px4ctrl
  quadrotor_msgs
  uav_utils
)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED) 

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs px4ctrl quadrotor_msgs uav_utils
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  include
)

add_executable(aic_plus_apo src/aic_plus_apo.cpp)
add_dependencies(aic_plus_apo ${catkin_EXPORTED_TARGETS})

target_link_libraries(aic_plus_apo ${catkin_LIBRARIES})

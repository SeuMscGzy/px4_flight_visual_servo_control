cmake_minimum_required(VERSION 3.0.2)
project(RAID-AgiVS_without_d_term)

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
  pluginlib
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

add_executable(RAID-AgiVS_without_d_term src/RAID-AgiVS_without_d_term.cpp)
add_dependencies(RAID-AgiVS_without_d_term ${catkin_EXPORTED_TARGETS})

target_link_libraries(RAID-AgiVS_without_d_term ${catkin_LIBRARIES})

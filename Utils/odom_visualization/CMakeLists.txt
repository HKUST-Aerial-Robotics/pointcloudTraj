cmake_minimum_required(VERSION 2.8.3)
project(odom_visualization)
set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -DEIGEN_DONT_PARALLELIZE ${CMAKE_CXX_FLAGS} -g -O3 -Wall")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  quadrotor_msgs
  tf
  pose_utils
)

catkin_package(
)

find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_visualization src/odom_visualization.cpp)
target_link_libraries(odom_visualization
   ${catkin_LIBRARIES}
   ${ARMADILLO_LIBRARIES}
   pose_utils
)
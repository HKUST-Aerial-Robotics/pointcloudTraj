cmake_minimum_required(VERSION 2.8)
project(pointcloudTraj)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -DEIGEN_DONT_PARALLELIZE ${CMAKE_CXX_FLAGS} -g -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    sensor_msgs
    quadrotor_msgs
    tf
    kdtree)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
    SYSTEM 
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS})

link_directories(
    ${PCL_LIBRARY_DIRS}
    ${PROJECT_SOURCE_DIR}/lib/mosek7)

add_executable (sim_planning_demo
    src/sim_planning_demo.cpp 
    src/traj_optimizer.cpp 
    src/corridor_finder.cpp 
    src/bezier_base.cpp)

target_link_libraries (sim_planning_demo
    mosek64
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})

add_executable (map_server
    src/map_server.cpp)

target_link_libraries( map_server
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES})

add_executable (traj_server 
    src/traj_server.cpp 
    src/bezier_base.cpp)

target_link_libraries( traj_server
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})

add_executable (random_map 
    src/random_map_generator.cpp)

target_link_libraries (random_map
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})

add_executable ( odom_generator 
    src/odom_generator.cpp )
target_link_libraries( odom_generator
    ${catkin_LIBRARIES} )
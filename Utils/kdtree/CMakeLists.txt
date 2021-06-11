cmake_minimum_required(VERSION 2.8.3)
project(kdtree)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/kdtree.c
)

target_link_libraries( kdtree
    ${catkin_LIBRARIES} 
    )  

#catkin_install()
#catkin_export()

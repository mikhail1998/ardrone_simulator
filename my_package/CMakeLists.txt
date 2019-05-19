cmake_minimum_required(VERSION 2.8.3)
project(my_package)
find_package(catkin REQUIRED COMPONENTS
  roscpp
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
 add_executable(publisher src/test_trajectory.cpp)
 add_executable(client src/stability.cpp)
 target_link_libraries(publisher
   ${catkin_LIBRARIES}
 )
 target_link_libraries(client
   ${catkin_LIBRARIES}
 )

cmake_minimum_required(VERSION 2.8.3)
project(my_package2)
find_package(catkin REQUIRED COMPONENTS
  roscpp
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
 add_executable(client src/stability.cpp)
 add_executable(subcriber src/dron_position.cpp)
  target_link_libraries(client
   ${catkin_LIBRARIES}
 )
  target_link_libraries(subscriber
   ${catkin_LIBRARIES}
 )

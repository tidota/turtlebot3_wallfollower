cmake_minimum_required(VERSION 2.8.3)
project(turtlebot3_wallfollower)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-maybe-uninitialized")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs

  geometry_msgs
  sensor_msgs
  std_srvs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(wallfollower src/main.cpp src/wallfollower.cpp)
target_link_libraries(wallfollower ${catkin_LIBRARIES})

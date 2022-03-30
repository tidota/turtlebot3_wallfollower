// main.cpp

#include <memory>
#include <ros/ros.h>
#include "turtlebot3_wallfollower/wallfollower.hpp"

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  ros::init(argc, argv, "wallfollower");
  Wallfollower wallfollower;
  ros::spin();
  return 0;
}

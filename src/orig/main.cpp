// main.cpp

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <turtlebot3_wallfollower/orig/wallfollower.hpp>

////////////////////////////////////////////////////////////////////////////////
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Wallfollower>());
  rclcpp::shutdown();
  return 0;
}

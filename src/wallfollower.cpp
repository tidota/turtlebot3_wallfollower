// controller.cpp

// the code was made by combining the two pieces of the sample code from
// https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "turtlebot3_wallfollower/wallfollower.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;

////////////////////////////////////////////////////////////////////////////////
Wallfollower::Wallfollower(): Node("wallfollower")
{
  // Subscribe the topic for proximity data
  subscription_
    = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&Wallfollower::topic_callback, this, _1));

  // Make a publisher for controll
  publisher_
    = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  timer_
    = this->create_wall_timer(
      500ms, std::bind(&Wallfollower::timer_callback, this));
}

////////////////////////////////////////////////////////////////////////////////
void Wallfollower::topic_callback(
  const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(scan_mutex);
  scan_msg_buff = *msg;
}

////////////////////////////////////////////////////////////////////////////////
void Wallfollower::timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "Publishing command");
  auto message = geometry_msgs::msg::Twist();

  sensor_msgs::msg::LaserScan msg;
  {
    std::lock_guard<std::mutex> lk(scan_mutex);
    msg = scan_msg_buff;
  }

  // TODO: make a command message based on the sensor
  message.angular.z = 1.0;

  publisher_->publish(message);
}

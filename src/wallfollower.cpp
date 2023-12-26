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
#include <std_srvs/srv/set_bool.hpp>

#include "turtlebot3_wallfollower/wallfollower.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

////////////////////////////////////////////////////////////////////////////////
Wallfollower::Wallfollower(): Node("wallfollower"), running_(false)
{
  // Subscribe the topic for proximity data
  subscription_
    = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&Wallfollower::topic_callback, this, _1));

  // Make a publisher for controll
  publisher_
    = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Make a timer to give a command at a specific interval.
  timer_
    = this->create_wall_timer(
      500ms, std::bind(&Wallfollower::timer_callback, this));

  // Make a service to receive a command.
  service_
    = this->create_service<std_srvs::srv::SetBool>(
      "set_running", std::bind(&Wallfollower::set_running, this, _1, _2));
}

////////////////////////////////////////////////////////////////////////////////
void Wallfollower::topic_callback(
  const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(scan_mutex_);
  scan_msg_buff_ = *msg;
}

////////////////////////////////////////////////////////////////////////////////
void Wallfollower::timer_callback()
{
  auto message = geometry_msgs::msg::Twist();

  if (this->running_)
  {
    sensor_msgs::msg::LaserScan msg;
    {
      std::lock_guard<std::mutex> lk(scan_mutex_);
      msg = scan_msg_buff_;
    }

    // make a command message based on the sensor
    constexpr int num_directions = 8;
    std::vector<double> ranges_min(num_directions);
    const int num_ranges = msg.ranges.size();
    const int interval = num_ranges/num_directions;

    if (num_ranges == 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Empty scan data!");
    }
    else
    {
      for (int m = 0; m < num_directions; ++m)
      {
        int start = (m*interval - interval/2 + num_ranges)%num_ranges;
        int end = (m*interval + interval/2)%num_ranges;
        double minimum = msg.ranges[start];
        for (int i = start + 1; i < end; ++i)
        {
          if (minimum == 0 || (msg.ranges[i] > 0 && minimum > msg.ranges[i]))
            minimum = msg.ranges[i];
        }
        ranges_min[m] = minimum;
      }

      if (ranges_min[0] < 0.3 ||
          ranges_min[1] < 0.3 ||
          ranges_min[0] < ranges_min[6] ||
          ranges_min[1] < ranges_min[6] ||
          ranges_min[2] < ranges_min[6])
      {
        // turn left
        message.angular.z = 0.5;
      }
      else if (ranges_min[7] > ranges_min[6] * 1.5)
      {
        // go around the right corner
        message.linear.x = 0.1;
        message.angular.z = -0.3;
      }
      else
      {
        if (ranges_min[7] < ranges_min[5] * 0.9 ||
            ranges_min[6] < 0.2)
        {
          // steer left
          message.linear.x = 0.1;
          message.angular.z = 0.3;
        }
        else if (ranges_min[5] < ranges_min[7] * 0.9 ||
                 ranges_min[6] > 0.3)
        {
          // steer right
          message.linear.x = 0.1;
          message.angular.z = -0.3;
        }
        else
        {
          // go straight
          message.linear.x = 0.2;
        }
      }
    }
  }

  publisher_->publish(message);
}

////////////////////////////////////////////////////////////////////////////////
void Wallfollower::set_running(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  this->running_ = request->data;
  response->success = true;
  if (this->running_)
  {
    RCLCPP_INFO(this->get_logger(), "Got a command to run.");
    response->message = "Run.";
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Got a command to stop.");
    response->message = "Stop.";
  }
}

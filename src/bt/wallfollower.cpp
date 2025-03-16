// controller.cpp

// the code was made by combining the two pieces of the sample code from
// https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <turtlebot3_wallfollower/bt/wallfollower.hpp>

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

Wallfollower::Wallfollower(): Node("wallfollower_bt") {
  // Subscribe the topic for proximity data
  laser_sub_
    = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&Wallfollower::LaserCb, this, _1));

  // Make a publisher for controll
  com_pub_
    = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // Make a service to receive a command.
  set_running_srv_
    = this->create_service<std_srvs::srv::SetBool>(
      "set_running", std::bind(&Wallfollower::SetRunningServiceCb, this, _1, _2));

  // Setup Behavior Tree
  BT::BehaviorTreeFactory factory;
  factory.registerSimpleAction("ProcData", std::bind(&Wallfollower::ProcData, this));
  factory.registerSimpleCondition("IsIdling", std::bind(&Wallfollower::IsIdling, this));
  factory.registerSimpleAction("TurnLeft", std::bind(&Wallfollower::TurnLeft, this));
  factory.registerSimpleAction("GoAroundRightCorner", std::bind(&Wallfollower::GoAroundRightCorner, this));
  factory.registerSimpleAction("SteerLeft", std::bind(&Wallfollower::SteerLeft, this));
  factory.registerSimpleAction("SteerRight", std::bind(&Wallfollower::SteerRight, this));
  factory.registerSimpleAction("GoStraight", std::bind(&Wallfollower::GoStraight, this));
  factory.registerSimpleAction("PublishCom", std::bind(&Wallfollower::PublishCom, this));

  this->declare_parameter("bt_config_path", rclcpp::PARAMETER_STRING);
  std::string path = this->get_parameter("bt_config_path").as_string();
  RCLCPP_INFO(this->get_logger(), "bt_config_path : %s", path.c_str());

  bt_ = factory.createTreeFromFile(path);
  th_bt_ = std::thread(
    [&](){
      rclcpp::Rate rate(10);
      while(rclcpp::ok()){
        bt_.tickRoot();
        rate.sleep();
      }
    });
}

BT::NodeStatus Wallfollower::ProcData() {
  RCLCPP_DEBUG(this->get_logger(), "Running ProcData");
  if (!running_) {
    ranges_min_.resize(0);
  } else {
    // proc data
    sensor_msgs::msg::LaserScan msg;
    {
      std::lock_guard<std::mutex> lk(scan_mutex_);
      msg = scan_msg_buff_;
    }

    const int num_ranges = msg.ranges.size();
    if (num_ranges == 0) {
      RCLCPP_ERROR(this->get_logger(), "Empty scan data!");
    } else {
      ranges_min_.resize(kNumDirections);
      const int index_0deg = static_cast<int>(num_ranges - msg.angle_min / msg.angle_increment)
                           % num_ranges;
      const int interval = num_ranges / kNumDirections;
      for (int m = 0; m < kNumDirections; ++m) {
        int start = (m*interval - interval/2 + index_0deg + num_ranges)
                  % num_ranges;
        int end = (start + interval) % num_ranges;
        double minimum = msg.ranges[start];
        for (int i = start + 1; i < end; ++i) {
          if (msg.ranges[i] > 0 && minimum > msg.ranges[i]) {
            minimum = msg.ranges[i];
          }
        }
        ranges_min_[m] = minimum;
      }
    }
  }
  return BT::NodeStatus::SUCCESS;
}

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
//#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

using std::placeholders::_1;

class WallFollower : public rclcpp::Node
{
  public:
    WallFollower(): Node("wall_follower")
    {
      // Subscribe the topic for proximity data
      subscription_
        = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "scan", 10, std::bind(&WallFollower::topic_callback, this, _1));

      // Make a publisher for controll
      publisher_
        = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

      timer_
        = this->create_wall_timer(
          500ms, std::bind(&WallFollower::timer_callback, this));
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  private:
    rclcpp::TimerBase::SharedPtr timer_;

  private: std::mutex scan_mutex;
  private: sensor_msgs::msg::LaserScan scan_msg_buff;

  private: void topic_callback(
                  const sensor_msgs::msg::LaserScan::SharedPtr msg);
  private: void timer_callback();
};

void WallFollower::topic_callback(
  const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(scan_mutex);
  scan_msg_buff = *msg;
}

void WallFollower::timer_callback()
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallFollower>());
  rclcpp::shutdown();
  return 0;
}
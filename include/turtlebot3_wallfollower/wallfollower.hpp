// wallfollower.hpp

#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class Wallfollower : public rclcpp::Node
{
  public: Wallfollower();
  private: void topic_callback(
                  const sensor_msgs::msg::LaserScan::SharedPtr msg);
  private: void timer_callback();

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  private:
    rclcpp::TimerBase::SharedPtr timer_;

  private: std::mutex scan_mutex;
  private: sensor_msgs::msg::LaserScan scan_msg_buff;
};

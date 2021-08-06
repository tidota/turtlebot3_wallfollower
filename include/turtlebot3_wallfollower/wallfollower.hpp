// wallfollower.hpp

#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/set_bool.hpp>

class Wallfollower : public rclcpp::Node
{
  public: Wallfollower();
  private: void topic_callback(
                  const sensor_msgs::msg::LaserScan::SharedPtr msg);
  private: void timer_callback();
  private: void set_running(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  private:
    rclcpp::TimerBase::SharedPtr timer_;
  private:
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

  private: bool running;

  private: std::mutex scan_mutex;
  private: sensor_msgs::msg::LaserScan scan_msg_buff;
};

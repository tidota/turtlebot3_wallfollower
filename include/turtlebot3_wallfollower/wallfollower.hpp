// wallfollower.hpp

#include <mutex>

#include <ros/ros.h>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/set_bool.hpp>

class Wallfollower
{
  public: Wallfollower();
  private: void topic_callback(
                  const sensor_msgs::msg::LaserScan::SharedPtr msg);
  private: void timer_callback();
  private: void set_running(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  private:
    ros::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  private:
    ros::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  private:
    ros::TimerBase::SharedPtr timer_;
  private:
    ros::Service<std_srvs::srv::SetBool>::SharedPtr service_;

  private: bool running_;

  private: std::mutex scan_mutex_;
  private: sensor_msgs::msg::LaserScan scan_msg_buff_;
};

// wallfollower.hpp

#include <mutex>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/SetBool.h>

class Wallfollower
{
  public: Wallfollower();
  private: void topic_callback(
                  const sensor_msgs::LaserScan::ConstPtr& msg);
  private: void timer_callback(const ros::TimerEvent& event);
  private: bool set_running(
    std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);

  private: ros::NodeHandle nh;

  private:
    ros::Publisher publisher_;
  private:
    ros::Subscriber subscription_;
  private:
    ros::Timer timer_;
  private:
    ros::ServiceServer service_;

  private: bool running_;

  private: std::mutex scan_mutex_;
  private: sensor_msgs::LaserScan scan_msg_buff_;
};

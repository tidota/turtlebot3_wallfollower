// controller.cpp

// the code was made by combining the two pieces of the sample code from
// https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

#include <memory>
#include <mutex>
#include <string>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/SetBool.h>

#include "turtlebot3_wallfollower/wallfollower.hpp"

////////////////////////////////////////////////////////////////////////////////
Wallfollower::Wallfollower(): running_(false)
{
  // Subscribe the topic for proximity data
  subscription_
    = this->nh.subscribe("scan", 1000, &Wallfollower::topic_callback, this);

  // Make a publisher for controll
  publisher_
    = this->nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  // Make a timer to give a command at a specific interval.
  timer_
    = this->nh.createTimer(
        ros::Duration(0.5), &Wallfollower::timer_callback, this);

  // Make a service to receive a command.
  service_
    = this->nh.advertiseService(
        "set_running", &Wallfollower::set_running, this);
}

////////////////////////////////////////////////////////////////////////////////
void Wallfollower::topic_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lk(scan_mutex_);
  scan_msg_buff_ = *msg;
}

////////////////////////////////////////////////////////////////////////////////
void Wallfollower::timer_callback(const ros::TimerEvent& event)
{
  auto message = geometry_msgs::Twist();

  if (this->running_)
  {
    sensor_msgs::LaserScan msg;
    {
      std::lock_guard<std::mutex> lk(scan_mutex_);
      msg = scan_msg_buff_;
    }

    // make a command message based on the sensor
    int M = 8;
    std::vector<double> ranges_min(M);
    // std::vector<double> ranges_ave(M);
    int N = msg.ranges.size();
    int interval = N/M;

    if (N == 0)
    {
      ROS_ERROR_STREAM("Empty scan data!");
    }
    else
    {
      for (int m = 0; m < M; ++m)
      {
        int start = (m*interval - interval/2 + N)%N;
        int end = (m*interval + interval/2)%N;
        double minimum = msg.ranges[start];
        // double total = 0;
        // int count = 0;
        for (int i = start + 1; i < end; ++i)
        {
          if (minimum == 0 || (msg.ranges[i] > 0 && minimum > msg.ranges[i]))
            minimum = msg.ranges[i];
          // if (!std::isinf(msg.ranges[i]) && msg.ranges[i] > 0)
          // {
          //   total += msg.ranges[i];
          //   ++count;
          // }
        }
        ranges_min[m] = minimum;
        // if (count > 0)
        //   ranges_ave[m] = total / count;
        // else
        //   ranges_ave[m] = minimum;
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

  publisher_.publish(message);
}

////////////////////////////////////////////////////////////////////////////////
bool Wallfollower::set_running(
  std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
  this->running_ = request.data;
  response.success = true;
  if (this->running_)
  {
    ROS_INFO_STREAM("Got a command to run.");
    response.message = "Run.";
  }
  else
  {
    ROS_INFO_STREAM("Got a command to stop.");
    response.message = "Stop.";
  }
  return true;
}

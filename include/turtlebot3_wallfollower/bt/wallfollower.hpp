// wallfollower.hpp

#include <mutex>
#include <thread>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/set_bool.hpp>

class Wallfollower : public rclcpp::Node {
 public:
  //! Constructor
  Wallfollower();
  //! Destructor
  ~Wallfollower() {
    th_bt_.join();
  }

 private:
  //! Callback for laser scan data
  void LaserCb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(scan_mutex_);
    scan_msg_buff_ = *msg;
  }

  //! Callback for the service to set the running flag
  void SetRunningServiceCb(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    this->running_ = request->data;
    response->success = true;
    if (this->running_) {
      RCLCPP_INFO(this->get_logger(), "Got a command to run.");
      response->message = "Run.";
    } else {
      RCLCPP_INFO(this->get_logger(), "Got a command to stop.");
      response->message = "Stop.";
    }
  }

  //! Process the buffered data
  BT::NodeStatus ProcData();

  BT::NodeStatus IsIdling() {
    RCLCPP_DEBUG(this->get_logger(), "Running IsIdling");
    com_.linear.x = 0.0;
    com_.angular.z = 0.0;
  return ((running_) ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS);
  }
  BT::NodeStatus TurnLeft() {
    RCLCPP_DEBUG(this->get_logger(), "Running TurnLeft");
    if (ranges_min_[0] < 0.3 ||
        ranges_min_[1] < 0.3 ||
        ranges_min_[0] < ranges_min_[6] ||
        ranges_min_[1] < ranges_min_[6] ||
        ranges_min_[2] < ranges_min_[6]) {
      // turn left
      com_.linear.x = 0.0;
      com_.angular.z = 0.5;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
  BT::NodeStatus GoAroundRightCorner() {
    RCLCPP_DEBUG(this->get_logger(), "Running GoAroundRightCorner");
    if (ranges_min_[7] > ranges_min_[6] * 1.5) {
      // go around the right corner
      com_.linear.x = 0.1;
      com_.angular.z = -0.3;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
  BT::NodeStatus SteerLeft() {
    RCLCPP_DEBUG(this->get_logger(), "Running SteerLeft");
    if (ranges_min_[7] < ranges_min_[5] * 0.9 ||
        ranges_min_[6] < 0.2) {
      // steer left
      com_.linear.x = 0.1;
      com_.angular.z = 0.3;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
  BT::NodeStatus SteerRight() {
    RCLCPP_DEBUG(this->get_logger(), "Running SteerRight");
    if (ranges_min_[5] < ranges_min_[7] * 0.9 ||
        ranges_min_[6] > 0.3) {
      // steer right
      com_.linear.x = 0.1;
      com_.angular.z = -0.3;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
  BT::NodeStatus GoStraight() {
    RCLCPP_DEBUG(this->get_logger(), "Running GoStraight");
    // go straight
    com_.linear.x = 0.2;
    com_.angular.z = 0.0;
    return BT::NodeStatus::SUCCESS;
  }

  //! Publish the command
  BT::NodeStatus PublishCom() {
    RCLCPP_DEBUG(this->get_logger(), "Running PublishCom");
    com_pub_->publish(com_);
    return BT::NodeStatus::SUCCESS;
  }

  //! publisher for command
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr com_pub_;
  //! subscriber for laser scan
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  //! ROS service to set the running flag
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_running_srv_;

  //! the running flag indicating if the node generates non-zero command
  bool running_ = false;

  //! mutex for the laser scan data buffer
  std::mutex scan_mutex_;
  //! buffer for the laser scan data
  sensor_msgs::msg::LaserScan scan_msg_buff_;

  //! number of direction areas
  static constexpr int kNumDirections = 8;
  //! list of minimum ranges in the separate directions
  std::vector<double> ranges_min_;

  //! Velocity command to publish
  geometry_msgs::msg::Twist com_;

  //! Behavior tree factory
  BT::Tree bt_;
  //! Thread to run the BT
  std::thread th_bt_;
};

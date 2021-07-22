// controller.cpp

// TODO: modify the sample code below so that it performs the simple controller.

// the code was made by combining the two pieces of the sample code from
// https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

using std::placeholders::_1;

class MinimalPubSub : public rclcpp::Node
{
  public:
    MinimalPubSub()
    : Node("minimal_pubsub"), count_(0)
    {
      publisher_
        = this->create_publisher<std_msgs::msg::String>("topic", 10);

      subscription_
        = this->create_subscription<std_msgs::msg::String>(
          "topic", 10, std::bind(&MinimalPubSub::topic_callback, this, _1));

      timer_
        = this->create_wall_timer(
          500ms, std::bind(&MinimalPubSub::timer_callback, this));
    }

  private: rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  private: rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  private: rclcpp::TimerBase::SharedPtr timer_;
  private: size_t count_;

  private: void topic_callback(
                  const std_msgs::msg::String::SharedPtr msg) const;
  private: void timer_callback();
};

void MinimalPubSub::topic_callback(
  const std_msgs::msg::String::SharedPtr msg) const
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

void MinimalPubSub::timer_callback()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPubSub>());
  rclcpp::shutdown();
  return 0;
}

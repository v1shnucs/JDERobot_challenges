#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class DemoPublisher : public rclcpp::Node {
public:
  DemoPublisher() : Node("demo_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("demo_topic", 10);
    timer_ = this->create_wall_timer(
        1000ms, std::bind(&DemoPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello! ROS2 is fun.";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DemoPublisher>());
  rclcpp::shutdown();
  return 0;
}
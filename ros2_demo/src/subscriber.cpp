#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class DemoSubscriber : public rclcpp::Node {
public:
  DemoSubscriber() : Node("demo_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "demo_topic", 10, std::bind(&DemoSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DemoSubscriber>());
  rclcpp::shutdown();
  return 0;
}
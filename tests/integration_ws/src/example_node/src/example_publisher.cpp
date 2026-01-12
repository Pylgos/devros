// Example ROS 2 publisher for integration testing
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ExamplePublisher : public rclcpp::Node
{
public:
  ExamplePublisher()
  : Node("example_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("example_topic", 10);
    
    // For testing, we just log and exit after one iteration
    RCLCPP_INFO(this->get_logger(), "Example publisher initialized");
  }

  void publish_message()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello from devros! Count: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExamplePublisher>();
  
  // Publish one message and exit (for testing purposes)
  node->publish_message();
  
  // Give time for the message to be sent
  rclcpp::spin_some(node);
  
  rclcpp::shutdown();
  return 0;
}

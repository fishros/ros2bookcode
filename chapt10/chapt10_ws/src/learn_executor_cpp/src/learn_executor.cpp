#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sstream>

class LearnExecutorNode : public rclcpp::Node {
public:
  LearnExecutorNode() : Node("learn_executor") {
    publisher_ =
        this->create_publisher<std_msgs::msg::String>("string_topic", 10);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&LearnExecutorNode::timer_callback, this));
    service_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);  // 互斥回调组
    service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
        "add_two_ints",
        std::bind(&LearnExecutorNode::add_two_ints_callback, this,
                  std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, service_callback_group_);
  }

private:
  void timer_callback() {
    auto msg = std_msgs::msg::String();
    msg.data = "话题发布：" + thread_info();
    RCLCPP_INFO(this->get_logger(), msg.data.c_str());
    publisher_->publish(msg);
  }

  std::string thread_info() {
    std::ostringstream thread_str;
    thread_str << "线程ID：" << std::this_thread::get_id();
    return thread_str.str();
  }

  void add_two_ints_callback(
      const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request>
          request,
      std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
    RCLCPP_INFO(this->get_logger(), "服务开始处理：%s", thread_info().c_str());
    std::this_thread::sleep_for(std::chrono::seconds(10));
    response->sum = request->a + request->b;
    RCLCPP_INFO(this->get_logger(), "服务处理完成：%s", thread_info().c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LearnExecutorNode>();
//   auto executor = rclcpp::executors::SingleThreadedExecutor();
  auto executor = rclcpp::executors::MultiThreadedExecutor();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
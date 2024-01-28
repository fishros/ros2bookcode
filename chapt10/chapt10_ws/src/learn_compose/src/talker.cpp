#include <chrono>
#include "learn_compose/talker.hpp"

namespace learn_compose {

using namespace std::chrono_literals;

Talker::Talker(const rclcpp::NodeOptions &options) : Node("talker", options) {
  pub_ = this->create_publisher<std_msgs::msg::Int32>("count", 10);
  auto callback = [&]() -> void {
    std_msgs::msg::Int32::UniquePtr msg(new std_msgs::msg::Int32());
    msg->data = count_++;
    RCLCPP_INFO(this->get_logger(), "发布数据:%d(0x%lX)", msg->data,
                reinterpret_cast<std::uintptr_t>(msg.get()));
    pub_->publish(std::move(msg));
  };
  timer_ = this->create_wall_timer(1s, callback);
}
} // namespace  learn_compose



#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(learn_compose::Talker)
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LearnLifeCycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
  LearnLifeCycleNode()
      : rclcpp_lifecycle::LifecycleNode("lifecyclenode") {
    timer_period_ = 1.0;
    timer_ = nullptr;
    RCLCPP_INFO(get_logger(), "%s: 已创建", get_name());
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override {
    (void)state;
    timer_period_ = 1.0;
    RCLCPP_INFO(get_logger(), "on_configure():配置周期 timer_period");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override {
    (void)state;
    timer_ = create_wall_timer(
        std::chrono::seconds(static_cast<int>(timer_period_)),
        [this]() { RCLCPP_INFO(get_logger(), "定时器打印进行中..."); });
    RCLCPP_INFO(get_logger(), "on_activate():处理激活指令，创建定时器");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override {
    (void)state;
    timer_.reset();
    RCLCPP_INFO(get_logger(), "on_deactivate():处理失活指令，停止定时器");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override {
    (void)state;
    timer_.reset();
    RCLCPP_INFO(get_logger(), "on_shutdown():处理关闭指令");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
        CallbackReturn::SUCCESS;
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  double timer_period_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LearnLifeCycleNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
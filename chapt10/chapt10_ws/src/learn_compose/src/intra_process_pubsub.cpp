#include "learn_compose/listener.hpp"
#include "learn_compose/talker.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;

  rclcpp::NodeOptions options;           // 创建节点选项
  options.use_intra_process_comms(true); // 使用进程内通信
  auto talker = std::make_shared<learn_compose::Talker>(options);
  auto listener = std::make_shared<learn_compose::Listener>(options);

  executor.add_node(talker);
  executor.add_node(listener);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
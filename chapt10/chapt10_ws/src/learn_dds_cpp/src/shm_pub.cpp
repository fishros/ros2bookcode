#include "rclcpp/loaned_message.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class LoanedMessagePublisher : public rclcpp::Node {
public:
  LoanedMessagePublisher() : Node("loaned_message_publisher") {
    publisher_ =
        this->create_publisher<std_msgs::msg::Int32>("loaned_int_topic", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1), [&]() {
      auto message = publisher_->borrow_loaned_message(); // 1.租借消息
      message.get().data = count_++;  // 2.放入数据
      RCLCPP_INFO(this->get_logger(), "发布数据:%d", message.get().data);
      publisher_->publish(std::move(message));  // 3.发布数据
    });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int32_t count_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LoanedMessagePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
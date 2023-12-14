#include <memory>
#include "geometry_msgs/msg/transform_stamped.hpp"  // 提供消息接口
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"  // 提供 tf2::Quaternion 类
#include "tf2/utils.h"                  // 提供 tf2::getEulerYPR 函数
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  // 提供消息类型转换函数
#include "tf2_ros/buffer.h"              // 提供 TF 缓冲类 Buffer
#include "tf2_ros/transform_listener.h"  // 提供坐标监听器类
#include <chrono>                                  // 引入时间相关头文件
using namespace std::chrono_literals;

class TFListener : public rclcpp::Node {
 public:
  TFListener() : Node("tf_listener") {
    buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, this);
    timer_ = this->create_wall_timer(5s, std::bind(&TFListener::getTransform, this));
  }

  void getTransform() {
    try {
      // 等待变换可用
      const auto transform = buffer_->lookupTransform(
          "base_link", "target_point", this->get_clock()->now(),
          rclcpp::Duration::from_seconds(1.0f));
      // 转换结果及打印
      const auto &translation = transform.transform.translation;
      const auto &rotation = transform.transform.rotation;
      double yaw, pitch, roll;
      tf2::getEulerYPR(rotation, yaw, pitch, roll);  // 四元数转欧拉角
      RCLCPP_INFO(get_logger(), "平移分量: (%f, %f, %f)", translation.x,
                  translation.y, translation.z);
      RCLCPP_INFO(get_logger(), "旋转分量: (%f, %f, %f)", roll, pitch, yaw);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "异常: %s", ex.what());  // 处理异常
    }
  }

 private:
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TFListener>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

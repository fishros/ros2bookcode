#include <chrono>
#include <cstdlib>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "chapt4_interfaces/srv/patrol.hpp"
#include <chrono> // 引入时间相关头文件
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

// 使用时间单位的字面量，可以在代码中使用 s 和 ms 表示时间
using namespace std::chrono_literals;
using Patrol = chapt4_interfaces::srv::Patrol;
using SetP = rcl_interfaces::srv::SetParameters;

class PatrolClient : public rclcpp::Node
{
public:
  PatrolClient() : Node("patrol_client")
  {
    patrol_client_ = this->create_client<Patrol>("patrol");
    timer_ = this->create_wall_timer(10s, std::bind(&PatrolClient::timer_callback, this));
    srand(time(NULL)); // 初始化随机数种子，使用当前时间作为种子
  }
  void update_server_param_k(double k)
  {
    // 1.创建一个参数对象
    auto param = rcl_interfaces::msg::Parameter();
    param.name = "k";
    // 2.创建参数值对象并赋值
    auto param_value = rcl_interfaces::msg::ParameterValue();
    param_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    param_value.double_value = k;
    param.value = param_value;
    // 3.请求更新参数并处理
    auto response = call_set_parameters(param);
    if (response == nullptr)
    {
      RCLCPP_WARN(this->get_logger(), "参数修改失败");
      return;
    }
    else
    {
      for (auto result : response->results)
      {
        if (result.successful)
        {
          RCLCPP_INFO(this->get_logger(), "参数k 已修改为：%f", k);
        }
        else
        {
          RCLCPP_WARN(this->get_logger(), "参数k 失败原因：%s", result.reason.c_str());
        }
      }
    }
  }

  std::shared_ptr<SetP::Response> call_set_parameters(
      rcl_interfaces::msg::Parameter &parameter)
  {
    // 1. 创建客户端等待服务上线
    auto param_client = this->create_client<SetP>(
        "/turtle_controller/set_parameters");
    while (!param_client->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
        return nullptr;
      }
      RCLCPP_INFO(this->get_logger(), "等待参数设置服务端上线中");
    }
    // 2. 创建请求对象
    auto request =
        std::make_shared<SetP::Request>();
    request->parameters.push_back(parameter);
    // 3. 异步调用、等待并返回响应结果
    auto future = param_client->async_send_request(request);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
    auto response = future.get();
    return response;
  }

  void timer_callback()
  {
    // 1.等待服务端上线
    while (!patrol_client_->wait_for_service(1s))
    {
      // 等待时检测rclcpp的状态
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }
    // 2.构造请求的
    auto request = std::make_shared<Patrol::Request>();
    request->target_x = rand() % 15;
    request->target_y = rand() % 15;
    RCLCPP_INFO(this->get_logger(), "请求巡逻：(%f,%f)", request->target_x, request->target_y);
    // 3.发送异步请求，然后等待返回，返回时调用回调函数
    patrol_client_->async_send_request(
        request,
        [&](rclcpp::Client<Patrol>::SharedFuture result_future) -> void
        {
          auto response = result_future.get();
          if (response->result == Patrol::Response::SUCCESS)
          {
            RCLCPP_INFO(this->get_logger(), "目标点处理成功");
          }
          else if (response->result == Patrol::Response::FAIL)
          {
            RCLCPP_INFO(this->get_logger(), "目标点处理失败");
          }
        });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<Patrol>::SharedPtr patrol_client_;
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrolClient>();
  node->update_server_param_k(1.5);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

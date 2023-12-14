#include <string>
#include "rclcpp/rclcpp.hpp"

class PersonNode : public rclcpp::Node
{
private:
  std::string name_;
  int age_;

public:
  PersonNode(const std::string &node_name,
             const std::string &name,
             const int &age) : Node(node_name)
  {
    this->name_ = name;
    this->age_ = age;
  };

  void eat(const std::string &food_name)
  {
    RCLCPP_INFO(this->get_logger(), "我是%s，今年%d岁，我现在正在吃%s",
                name_.c_str(), age_, food_name.c_str());
  };
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PersonNode>("cpp_node", "法外狂徒张三", 18);
  node->eat("鱼香ROS");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

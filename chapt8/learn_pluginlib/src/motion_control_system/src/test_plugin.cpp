#include "motion_control_system/motion_control_interface.hpp"
#include <pluginlib/class_loader.hpp>

int main(int argc, char **argv) {
  // 判断参数数量是否合法
  if (argc != 2)
    return 0;
  // 通过命令行参数，选择要加载的插件,argv[0]是可执行文件名，argv[1]表示参数名
  std::string controller_name = argv[1];
  // 1.通过功能包名称和基类名称创建控制器加载器
  pluginlib::ClassLoader<motion_control_system::MotionController>
      controller_loader("motion_control_system",
                        "motion_control_system::MotionController");
  // 2.使用加载器加载指定名称的插件，返回的是指定插件类的对象的指针
  auto controller = controller_loader.createSharedInstance(controller_name);
  // 3.调用插件的方法
  controller->start();
  controller->stop();
  return 0;
}
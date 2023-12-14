cmake_minimum_required (VERSION 3.8)
project (ros2_cpp)
add_executable(ros2_cpp_node ros2_cpp_node.cpp)
# 查找rclcpp头文件和库文件的路径
find_package(rclcpp REQUIRED) 
# 给可执行文件包含头文件
target_include_directories(ros2_cpp_node PUBLIC ${rclcpp_INCLUDE_DIRS})
# 给可执行文件链接库文件
target_link_libraries(ros2_cpp_node ${rclcpp_LIBRARIES})

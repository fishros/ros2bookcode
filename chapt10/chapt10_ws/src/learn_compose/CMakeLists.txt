cmake_minimum_required(VERSION 3.8)
project(learn_compose)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
include_directories(include)
add_executable(intra_process_pubsub 
    src/intra_process_pubsub.cpp
    src/talker.cpp
    src/listener.cpp
)
ament_target_dependencies(intra_process_pubsub std_msgs rclcpp rclcpp_components)

install(TARGETS intra_process_pubsub
  DESTINATION lib/${PROJECT_NAME})


find_package(rclcpp_components REQUIRED)
add_library(talker_component SHARED src/talker.cpp)
ament_target_dependencies(talker_component "std_msgs" "rclcpp" "rclcpp_components")
rclcpp_components_register_nodes(talker_component "learn_compose::Talker")

add_library(listener_component SHARED src/listener.cpp)
ament_target_dependencies(listener_component "std_msgs" "rclcpp" "rclcpp_components")
rclcpp_components_register_nodes(listener_component "learn_compose::Listener")

install(TARGETS talker_component listener_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()

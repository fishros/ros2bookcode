import launch
import launch_ros

def generate_launch_description():
    action_declare_arg_max_spped = launch.actions.DeclareLaunchArgument('launch_max_speed', default_value='2.0')

    action_node_turtle_control = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable="turtle_control",
        output='screen',
        parameters=[{'max_speed': launch.substitutions.LaunchConfiguration(
  'launch_max_speed', default='2.0')}],

    )
    action_node_patrol_client = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable="patrol_client",
        output='log',
    )
    action_node_turtlesim_node = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        output='both',
    )
   # 合成启动描述并返回
    launch_description = launch.LaunchDescription([
        action_declare_arg_max_spped,
        action_node_turtle_control,
        action_node_patrol_client,
        action_node_turtlesim_node
    ])
    return launch_description

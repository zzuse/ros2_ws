import launch
import launch_ros


def generate_launch_description():
    action_declare_launch_arg_background_green = launch.actions.DeclareLaunchArgument(
        'background_green',
        default_value='150',
        description='Green background color of the turtlesim node'
    )
    action_node_turtle_sim_node = launch_ros.actions.Node(
        package='turtlesim',
        executable='turtlesim_node',
        parameters=[{
            'background_g': launch.substitutions.LaunchConfiguration('background_green',default='150')
        }],
        output='screen'
    )
    action_node_turtle_control_node = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable='turtle_control',
        output='both'
    )
    action_node_patrol_client_node = launch_ros.actions.Node(
        package='demo_cpp_service',
        executable='patrol_client',
        output='log'
    )
    return launch.LaunchDescription([
        action_node_turtle_sim_node,
        action_node_turtle_control_node,
        action_node_patrol_client_node,
        action_declare_launch_arg_background_green
    ])
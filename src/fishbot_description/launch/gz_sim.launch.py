import launch
import launch_ros.actions
import launch_ros
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ros_gz_bridge.actions import RosGzBridge
import os


def generate_launch_description():
    urdf_package_path = get_package_share_directory('fishbot_description')
    default_xacro_path = os.path.join(urdf_package_path, 'urdf', 'fishbot.urdf.xacro')
    default_gz_world_path = os.path.join(urdf_package_path, 'world', 'custom_room.sdf')
    default_bridge_yaml_path = os.path.join(urdf_package_path,'config','bridge.yaml')
    action_declare_arg_model_path = DeclareLaunchArgument(
        name='model',
        default_value=str(default_xacro_path),
        description='Path to the URDF file'
    )
    command_result = launch.substitutions.Command(['xacro ', LaunchConfiguration('model')])
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(command_result, value_type=str)
    action_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_value}]
    )

    action_launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r {}'.format(default_gz_world_path)}.items(),
    )

    # ros_gz_sim create: Spawns robots/models from SDF or URDF files at runtime.
    spawn_model = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'fishbot',
        ],
        output='screen'
    )

    bridge_name = DeclareLaunchArgument('bridge_name', default_value='ros_gz_bridge', description='Name of the ROS-Gazebo bridge node')
    config_file = DeclareLaunchArgument('config_file', default_value=default_bridge_yaml_path, description='Path to the ROS-Gazebo bridge configuration YAML file')
    ros_gz_bridge = RosGzBridge(
        bridge_name=LaunchConfiguration('bridge_name'),
        config_file=LaunchConfiguration('config_file')
    )

    action_load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    return launch.LaunchDescription([
        action_declare_arg_model_path,
        action_robot_state_publisher,
        action_launch_gazebo,
        spawn_model,
        bridge_name,
        config_file,
        ros_gz_bridge,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_model,
                on_exit=[action_load_joint_state_controller]
            ))
    ])
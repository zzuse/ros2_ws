import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ros_gz_bridge.actions import RosGzBridge

def generate_launch_description():
    pkg_fishbot_description = get_package_share_directory('fishbot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    default_model_path = os.path.join(pkg_fishbot_description, 'urdf', 'fishbot_ros2_control.urdf.xacro')
    default_world_path = os.path.join(pkg_fishbot_description, 'world', 'custom_room1a.world')
    default_bridge_yaml_path = os.path.join(pkg_fishbot_description, 'config', 'bridge.yaml')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model_path,
        description='Path to robot urdf file'
    )

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    # Note: use_sim_time is set to True
    action_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    action_launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {default_world_path}'}.items(),
    )

    # 设置 Gazebo 资源路径
    gazebo_resource_path = launch.actions.SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_fishbot_description, 'world'),
            ':',
            os.path.join(pkg_fishbot_description, 'materials', 'textures')
        ]
    )

    ign_gazebo_resource_path = launch.actions.SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_fishbot_description, 'world'),
            ':',
            os.path.join(pkg_fishbot_description, 'materials', 'textures')
        ]
    )
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-entity', 'fishbot'],
        output='screen'
    )
    bridge_name = DeclareLaunchArgument('bridge_name', default_value='ros_gz_bridge', description='Name of the ROS-Gazebo bridge node')
    ros2_gz_bridge = RosGzBridge(
        bridge_name=LaunchConfiguration('bridge_name'),
        config_file=default_bridge_yaml_path
    )

    action_load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    action_load_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'effort_controller'],
        output='screen'
    )
    action_load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_controller'],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        action_robot_state_publisher_node,
        gazebo_resource_path,
        ign_gazebo_resource_path,
        action_launch_gazebo,
        spawn_entity,
        bridge_name,
        ros2_gz_bridge,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[action_load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=action_load_joint_state_broadcaster,
                on_exit=[action_load_diff_drive_controller],
            )
        ),
    ])

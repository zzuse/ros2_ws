import os
import launch
import launch_ros
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_fishbot_nav2 = get_package_share_directory('fishcar_navigation2')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    rviz2_config_path = os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(pkg_fishbot_nav2, 'maps', 'room.yaml'))
    nav2_params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_fishbot_nav2, 'config', 'nav2_params.yaml'))
    # Create a launch description
    return launch.LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, description='Use simulation time'),
        DeclareLaunchArgument('map', default_value=map_yaml_file, description='Full path to map yaml file'),
        DeclareLaunchArgument('params_file', default_value=nav2_params_file, description='Full path to nav2 parameters file'),

        # Include the navigation2 launch file
        launch.actions.GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([pkg_nav2_bringup, '/launch', '/bringup_launch.py']),
                    launch_arguments={
                        'map': map_yaml_file,
                        'use_sim_time': use_sim_time,
                        'params_file': nav2_params_file
                    }.items(),
                ),
            ]
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz2_config_path],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])

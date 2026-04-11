import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    action_delcalre_rqt_startup = launch.actions.DeclareLaunchArgument(
        'rqt_startup',
        default_value='False',
        description='start rqt when the launch file is executed'
    )
    rqt_start = launch.substitutions.LaunchConfiguration('rqt_startup', default='False')
    multisim_launch_file_path = [get_package_share_directory('turtlesim'), '/launch/', 'multisim.launch.py']
    action_include_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            multisim_launch_file_path 
        )
    )
    action_log_info = launch.actions.LogInfo(
        msg=str(multisim_launch_file_path)
    )
    action_topic_list = launch.actions.ExecuteProcess(
        cmd=['ros2', 'topic', 'list'],
        output='screen'
    )
    action_rqt = launch.actions.ExecuteProcess(
        condition=launch.conditions.IfCondition(rqt_start),
        cmd=['rqt'],
        output='screen'
    )
    action_group_action = launch.actions.GroupAction([
        launch.actions.TimerAction(period=2.0, actions=[action_include_launch]),
        launch.actions.TimerAction(period=4.0, actions=[action_topic_list]),
        launch.actions.TimerAction(period=6.0, actions=[action_rqt])
    ])
    return launch.LaunchDescription([
        action_delcalre_rqt_startup,
        action_group_action,
        action_log_info
    ])


import os
import launch
import launch_ros
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription


def generate_launch_description():
    robot_navi_dir = get_package_share_directory('navigation')
    bringup = get_package_share_directory("nav2_bringup")

    rviz = os.path.join(bringup, 'rviz', 'nav2_default_view.rviz')

    # jin xing can shu pei zhi de jian li
    time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    map_path = os.path.join(robot_navi_dir, 'maps', 'room.yaml')
    map_yaml = launch.substitutions.LaunchConfiguration('map', default=map_path)
    nav_param_path = os.path.join(robot_navi_dir, 'config', 'nav2_params.yaml')
    nav_param = launch.substitutions.LaunchConfiguration('params_file', default=nav_param_path)

    return launch.LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=time,
                              description='time'),
        DeclareLaunchArgument('map', default_value=map_path,
                              description='map file to load'),
        DeclareLaunchArgument('params_file', default_value=nav_param,
                              description="param file to load"),

        IncludeLaunchDescription(PythonLaunchDescriptionSource([bringup, '/launch', '/bringup_launch.py']),
                                 # jin xing can shu xiu gai
                                 launch_arguments={
                                     'map': map_path,
                                     'use_sim_time': time,
                                     'params__file': nav_param
                                 }.items(),
                                 ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz],
            parameters=[{'use_sim_time': time}],
            output='screen'
        ),
    ])
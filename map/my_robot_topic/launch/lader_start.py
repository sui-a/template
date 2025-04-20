from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
import launch
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    #jin xing lu jing cha zhao
    my_robot_topic_path = get_package_share_directory("my_robot_topic")
    ydlidar_ros2_dir = get_package_share_directory('ydlidar')

    #jin xing urdf zhuan tf qi dong
    urdf2tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([my_robot_topic_path, '/launch', "/urdf_start.py"]),
    )

    #qi dong odom bian huan
    odom2tf = Node(
        package = 'my_robot_topic',
        executable = 'odom_tf',
        name = 'odom2tf',
        output = 'screen',
    )

    #jin xing odom fa bu
    odom_start = Node(
        package = 'my_robot_topic',
        executable = 'my_odom',
        name = 'odom',
        output = 'screen',
    )

    #jin xing wang luo jie shou bing zhuan huan cheng chuan kou
    ros_serail2wifi = Node(
        package = 'ros_serail2wifi',
        executable= 'tcp_server',
        parameters=[{'serial_port' : '/tmp/tty_laser'}],
        output = 'screen'
    )

    #qi dong /scan fa bu
    ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ydlidar_ros2_dir, '/launch', "/ydlidar_launch.py"]),
    )

    #jin xing yan shi
    ydlidar_delay = launch.actions.TimerAction(period=5.0, actions=[ydlidar])

    return LaunchDescription([
        urdf2tf,
        odom2tf,
        odom_start,
        ros_serail2wifi,
        ydlidar_delay,
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    urdf_file = "/home/ubuntu/vsco_project/my_robot/lader/install/my_robot_topic/share/my_robot_topic/urdf/robot.urdf.xacro"

    robot_description = Command(['xacro ', urdf_file])
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        name = 'robot_state_publisher',
        output = 'screen',
        parameters = [{'robot_description' : robot_description}]
    )

    # jin xing guan jie xiao xi fa bu
    joint_state_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
    )

    return LaunchDescription([robot_state_publisher_node, joint_state_node])
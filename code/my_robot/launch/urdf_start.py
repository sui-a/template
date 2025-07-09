# minimal_robot_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command

def generate_launch_description():
    # 1. 加载URDF文件
    urdf_path = PathJoinSubstitution([
        FindPackageShare('my_robot'),  # 替换为您的功能包名
        'urdf',
        'robot.urdf.xacro'  # 替换为您的URDF文件名
    ])

    urdf_path2 = "/home/sui/output.urdf"

    # 2. 启动机器人状态发布节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'publish_frequency': 50.0  # 提高TF发布频率
        }]
    )

    # 3. 启动关节状态发布节点（基础版）
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'use_sim_time': False,
            'rate': 50  # 发布频率(Hz)
        }]
    )

    return LaunchDescription([
        robot_state_publisher,
        #joint_state_publisher
    ])
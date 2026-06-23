"""天链机械臂 TCB710 + F710 手柄遥操作 — 真机启动文件。

用法：
  ros2 launch tl_teleop_f710 tl_teleop_tcb710.launch.py
  ros2 launch tl_teleop_f710 tl_teleop_tcb710.launch.py joy_dev:=/dev/input/js1
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('tl_teleop_f710')
    config_path = os.path.join(pkg_share, 'config', 'tl_teleop_tcb710.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_dev', default_value='/dev/input/js0',
            description='F710 手柄设备路径',
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': LaunchConfiguration('joy_dev'),
                'deadzone': 0.1,
                'autorepeat_rate': 30.0,
            }],
        ),
        Node(
            package='tl_teleop_f710',
            executable='tl_teleop_f710_node',
            name='tl_teleop_f710_node',
            output='screen',
            parameters=[config_path],
        ),
    ])

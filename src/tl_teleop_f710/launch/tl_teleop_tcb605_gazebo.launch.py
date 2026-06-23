"""天链机械臂 TCB605 + F710 手柄遥操作 — Gazebo 仿真启动文件。

IK 由仿真桥接节点内部使用 Pinocchio 本地求解，无需 MoveIt2。

用法：
  ros2 launch tl_teleop_f710 tl_teleop_tcb605_gazebo.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    tl_teleop_f710_share = get_package_share_directory('tl_teleop_f710')
    tl_gazebo_share = get_package_share_directory('tl_gazebo')
    config_path = os.path.join(
        tl_teleop_f710_share, 'config', 'tl_teleop_tcb605_sim.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    tl_gazebo_share, 'launch', 'gazebo_tcb605_f710_sim.launch.py')
            ),
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='joy',
                    executable='joy_node',
                    name='joy_node',
                    parameters=[{
                        'dev': LaunchConfiguration(
                            'joy_dev', default='/dev/input/js0'),
                        'deadzone': 0.1,
                        'autorepeat_rate': 30.0,
                    }],
                ),
            ],
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='tl_teleop_f710',
                    executable='tl_teleop_f710_node',
                    name='tl_teleop_f710_node',
                    output='screen',
                    parameters=[config_path],
                ),
            ],
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='tl_teleop_f710',
                    executable='tl_teleop_f710_sim_bridge',
                    name='tl_teleop_f710_sim_bridge',
                    output='screen',
                ),
            ],
        ),
    ])

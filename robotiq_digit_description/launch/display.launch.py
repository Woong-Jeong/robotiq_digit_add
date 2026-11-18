"""Display the measured, fixed CAD pose; no joint state publisher is needed."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('robotiq_digit_description'))
    description = (share / 'urdf' / 'robotiq_digit_assembly.urdf').read_text()
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'rviz' / 'assembly.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])

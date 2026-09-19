# 실행 예:
#   ros2 launch cybergear_arm arm_2dof.launch.py               (실제 모터)
#   ros2 launch cybergear_arm arm_2dof.launch.py transport:=sim (시뮬레이터)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('cybergear_arm'), 'config', 'arm_2dof.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('transport', default_value='serial'),
        Node(
            package='cybergear_arm',
            executable='arm_controller',
            name='arm_controller',
            output='screen',
            parameters=[config, {'transport': LaunchConfiguration('transport')}],
        ),
    ])

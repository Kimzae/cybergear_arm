# 실행 예:
#   ros2 launch cybergear_arm arm.launch.py dof:=2                  (2축 실제 모터)
#   ros2 launch cybergear_arm arm.launch.py dof:=4 transport:=sim   (4축 시뮬레이터)
#
#   dof:=N  ->  config/arm_Ndof.yaml (제어기 설정) + config/model_Ndof.yaml (동역학 모델)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def setup(context):
    share = get_package_share_directory('cybergear_arm')
    dof = context.launch_configurations['dof']
    transport = context.launch_configurations['transport']
    params = [os.path.join(share, 'config', f'arm_{dof}dof.yaml'),
              {'model_file': os.path.join(share, 'config', f'model_{dof}dof.yaml')}]
    if transport:
        params.append({'transport': transport})
    return [Node(package='cybergear_arm', executable='arm_controller', name='arm_controller',
                 output='screen', parameters=params)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dof', default_value='2', description='2 또는 4'),
        DeclareLaunchArgument('transport', default_value='', description='serial | socketcan | sim (비우면 yaml 값)'),
        OpaqueFunction(function=setup),
    ])

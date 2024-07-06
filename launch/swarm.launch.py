from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    package_dir = FindPackageShare('ros2vr_swarm').find('ros2vr_swarm')
    return LaunchDescription([
         Node(
            package='ros2vr_swarm',
            executable='vr_node',
            name=f'vrobot_control_node_{i}',
            output='screen',
            parameters=[{'sys_id': i}]
        ) for i in range(4)
    ])
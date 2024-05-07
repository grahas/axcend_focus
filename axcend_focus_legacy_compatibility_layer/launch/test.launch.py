# test.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='my_package', executable='node1'),
        Node(package='my_package', executable='node2'),
        # Add more nodes as needed
    ])
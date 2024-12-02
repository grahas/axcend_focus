from launch import LaunchDescription
from launch_ros.actions import Node
import socket

def generate_launch_description():
    # Get the hostname, replacing any '-' or '.' characters with '_'
    hostname = socket.gethostname().replace('-', '_').replace('.', '_')

    return LaunchDescription([
        Node(
            package='axcend_focus_ros2_firmware_bridge',
            executable='firmware_bridge',
            name='firmware_bridge',
            namespace=hostname
        ),
        Node(
            package='axcend_focus_legacy_compatibility_layer',
            executable='legacy_compatibility_interface',
            name='legacy_compatibility_interface',
            namespace=hostname
        ),
        Node(
            package='axcend_focus_front_panel_button',
            executable='front_panel_button_controller',
            name='front_panel_button_controller',
            namespace=hostname
        ),
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            namespace=hostname
        ),
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi_node',
            namespace=hostname
        )
    ])

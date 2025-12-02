from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vmd3_radar_driver',
            executable='driver',
            name='vmd3_radar_node',
            output='screen',
            # Example: Pass parameters declared in your driver.py file
            parameters=[
                {'address': '192.168.100.201'},
                {'sensitivity': 14},
            ],
        ),
    ])
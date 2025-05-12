from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Odometry
        Node(
            package='uart_odom_publisher',
            executable='uart_odom_node',
            name='uart_odom_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyAMA1',
                'baudrate': 115200
            }]
        ),
    ])

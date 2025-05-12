from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rplidar_launch_file = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch', 'rplidar_a1_launch.py'
    )
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_file)
        )
    ])
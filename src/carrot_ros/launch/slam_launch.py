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
        # RPLIDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_file)
        ),
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
        # Static transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0.0', '0.0', '0.133', '0.0', '0.0', '0.0', 'base_link', 'laser']
        ),
        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                '/home/carrot/ros2_ws/src/carrot_ros/config/slam_toolbox.yaml',
                {'use_sim_time': False}
            ]
        ),
        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['slam_toolbox'],
                'bond_timeout': 10.0
            }]
        )
    ])

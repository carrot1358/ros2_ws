from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # โหลดไฟล์ URDF
    urdf_file_name = 'carrot_robot.urdf'
    urdf_file = os.path.join(
        get_package_share_directory('carrot_ros'),
        'urdf',
        urdf_file_name)
    
    # อ่านไฟล์ URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([      
        # Robot State Publisher (ใช้แทน Static transform)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
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
                'bond_timeout': 30.0,
                'attempt_respawn_reconnection': True
            }]
        )
    ])

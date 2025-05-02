#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # ประกาศ launch parameter
    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')
    max_speed = LaunchConfiguration('max_speed')
    max_turn = LaunchConfiguration('max_turn')
    
    return LaunchDescription([
        # ประกาศพารามิเตอร์ที่ผู้ใช้สามารถส่งผ่าน command line
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='พอร์ต serial ที่เชื่อมต่อกับ Pico'
        ),
        
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Baudrate ของการเชื่อมต่อ serial'
        ),
        
        DeclareLaunchArgument(
            'max_speed',
            default_value='50',
            description='ความเร็วสูงสุดของมอเตอร์ (0-100)'
        ),
        
        DeclareLaunchArgument(
            'max_turn',
            default_value='30',
            description='ความเร็วเลี้ยวสูงสุด'
        ),
        
        # เริ่ม teleop node
        Node(
            package='uart_odom_publisher',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'serial_port': serial_port,
                    'baudrate': baudrate,
                    'max_speed': max_speed,
                    'max_turn': max_turn,
                }
            ]
        ),
    ]) 
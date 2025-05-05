#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # ประกาศ launch parameter
    serial_port = LaunchConfiguration('serial_port')
    baudrate = LaunchConfiguration('baudrate')
    
    return LaunchDescription([
        # ประกาศพารามิเตอร์ที่ผู้ใช้สามารถส่งผ่าน command line
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyAMA1',  # รับข้อมูล odometry จาก Pico GPIO 12 -> Pi GPIO 1
            description='พอร์ต serial (ttyAMA1) ที่ใช้รับข้อมูล odometry จาก Pico'
        ),
        
        DeclareLaunchArgument(
            'baudrate',
            default_value='115200',
            description='Baudrate ของการเชื่อมต่อ serial'
        ),
        
        # เริ่ม uart_odom_node
        Node(
            package='uart_odom_publisher',
            executable='uart_odom_node',
            name='uart_odom_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'serial_port': serial_port,
                    'baudrate': baudrate,
                }
            ]
        ),
    ]) 
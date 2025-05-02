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
    linear_axis = LaunchConfiguration('linear_axis')
    angular_axis = LaunchConfiguration('angular_axis')
    
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
        
        DeclareLaunchArgument(
            'linear_axis',
            default_value='1',  # แกน Y ของ joystick ซ้าย
            description='แกนควบคุมการเคลื่อนที่เดินหน้า/ถอยหลัง'
        ),
        
        DeclareLaunchArgument(
            'angular_axis',
            default_value='0',  # แกน X ของ joystick ซ้าย
            description='แกนควบคุมการหมุนซ้าย/ขวา'
        ),
        
        # เริ่ม joy node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
            }]
        ),
        
        # เริ่ม teleop_joy node
        Node(
            package='uart_odom_publisher',
            executable='teleop_joy',
            name='teleop_joy',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'serial_port': serial_port,
                    'baudrate': baudrate,
                    'max_speed': max_speed,
                    'max_turn': max_turn,
                    'linear_axis': linear_axis,
                    'angular_axis': angular_axis,
                }
            ]
        ),
    ]) 
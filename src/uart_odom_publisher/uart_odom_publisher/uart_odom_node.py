#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, TransformStamped
import serial
import json
import math
from rclpy.qos import QoSProfile
import tf2_ros

class UartOdomPublisher(Node):
    def __init__(self):
        super().__init__('uart_odom_node')
        qos = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(Odometry, 'odom', qos)

        # Initialize TF2 broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # เพิ่ม parameter สำหรับพอร์ต serial
        self.declare_parameter('serial_port', '/dev/ttyAMA1')
        self.declare_parameter('baudrate', 115200)
        
        # อ่านค่าพารามิเตอร์
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        
        # odom data
        self.current_odom_data = None
        self.last_odom_data = None

        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.5)
            self.get_logger().info(f"Opened {self.serial_port} at {self.baudrate} baud")
        except Exception as e:
            self.get_logger().error(f"Failed to open UART: {e}")
            exit(1)

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        self.last_valid_data = None  # เก็บข้อมูลที่ถูกต้องล่าสุด

    def timer_callback(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                self.get_logger().debug("No data received")
                return
            
            # กรองข้อความที่ไม่ใช่ JSON
            if line.startswith("OK:") or line.startswith("Error:"):
                self.get_logger().debug(f"Ignoring command response: {line}")
                
                # หากมี last_valid_data ให้ใช้ข้อมูลล่าสุดที่ถูกต้อง
                if self.last_valid_data:
                    data = self.last_valid_data
                    self.get_logger().debug("Using last valid odometry data")
                else:
                    return
            # กรองข้อความคำสั่งมอเตอร์จาก teleop_joy
            elif line.startswith("CMD:"):
                self.get_logger().debug(f"Ignoring motor command: {line}")
                
                # หากมี last_valid_data ให้ใช้ข้อมูลล่าสุดที่ถูกต้อง
                if self.last_valid_data:
                    data = self.last_valid_data
                    self.get_logger().debug("Using last valid odometry data")
                else:
                    return
            else:
                self.get_logger().debug(f"Received data: {line}")
                
                try:
                    data = json.loads(line)
                    self.get_logger().debug(f"Parsed JSON: {data}")
                    
                    # Validate expected fields
                    required_fields = ['x', 'y', 'theta', 'v', 'w']
                    if not all(field in data for field in required_fields):
                        self.get_logger().error(f"Missing required fields in data: {data}")
                        return
                    
                    # เก็บข้อมูลที่ถูกต้องไว้
                    self.last_valid_data = data
                    
                except json.JSONDecodeError as je:
                    self.get_logger().error(f"JSON parsing error: {je}, raw data: {line}")
                    
                    # หากมี last_valid_data ให้ใช้ข้อมูลล่าสุดที่ถูกต้อง
                    if self.last_valid_data:
                        data = self.last_valid_data
                        self.get_logger().debug("Using last valid odometry data")
                    else:
                        return
                
            odom_msg = Odometry()

            # Header
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_footprint"

            # Pose
            odom_msg.pose.pose.position = Point(x=data['x'], y=data['y'], z=0.0)
            # Convert theta to quaternion
            qz = math.sin(data['theta'] / 2.0)
            qw = math.cos(data['theta'] / 2.0)
            odom_msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)

            # Twist (velocity)
            odom_msg.twist.twist.linear.x = data['v']
            odom_msg.twist.twist.angular.z = data['w']

            # Publish odometry
            self.publisher_.publish(odom_msg)
            self.current_odom_data = data
            if self.current_odom_data != self.last_odom_data:
                self.get_logger().info(f"Published odom: x={data['x']:.4f}, y={data['y']:.4f}, theta={data['theta']:.4f}, v={data['v']:.4f}, w={data['w']:.4f}")
            self.last_odom_data = self.current_odom_data
            # Publish TF (odom -> base_footprint)
            t = TransformStamped()
            t.header.stamp = odom_msg.header.stamp
            t.header.frame_id = "odom"
            t.child_frame_id = "base_footprint"
            t.transform.translation.x = data['x']
            t.transform.translation.y = data['y']
            t.transform.translation.z = 0.0
            t.transform.rotation = odom_msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f"Failed to read/parse: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = UartOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

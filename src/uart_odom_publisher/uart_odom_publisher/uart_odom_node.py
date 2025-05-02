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

        try:
            self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.5)
            self.get_logger().info("Opened /dev/ttyAMA0")
        except Exception as e:
            self.get_logger().error(f"Failed to open UART: {e}")
            exit(1)

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

    def timer_callback(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                self.get_logger().debug("No data received")
                return
            
            self.get_logger().debug(f"Received data: {line}")
            
            try:
                data = json.loads(line)
                self.get_logger().debug(f"Parsed JSON: {data}")
            except json.JSONDecodeError as je:
                self.get_logger().error(f"JSON parsing error: {je}, raw data: {line}")
                return
                
            # Validate expected fields
            required_fields = ['x', 'y', 'theta', 'v', 'w']
            if not all(field in data for field in required_fields):
                self.get_logger().error(f"Missing required fields in data: {data}")
                return
                
            odom_msg = Odometry()

            # Header
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"

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
            self.get_logger().info(f"Published odom: x={data['x']:.4f}, y={data['y']:.4f}, theta={data['theta']:.4f}, v={data['v']:.4f}, w={data['w']:.4f}")

            # Publish TF (odom -> base_link)
            t = TransformStamped()
            t.header.stamp = odom_msg.header.stamp
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
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

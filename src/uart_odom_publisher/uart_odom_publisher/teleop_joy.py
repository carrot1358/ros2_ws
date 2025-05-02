#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import threading
import serial
import json
import time

class TeleopJoy(Node):
    def __init__(self):
        super().__init__('teleop_joy')
        
        # รับพารามิเตอร์จาก launch file หรือ command line
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('max_speed', 50)  # ความเร็วสูงสุดของมอเตอร์ (0-100)
        self.declare_parameter('max_turn', 30)   # ความเร็วเลี้ยวสูงสุด
        self.declare_parameter('linear_axis', 1)  # แกน Y ของ joystick ซ้าย (PS4/Xbox)
        self.declare_parameter('angular_axis', 0)  # แกน X ของ joystick ซ้าย (PS4/Xbox)
        
        # อ่านค่าพารามิเตอร์
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_turn = self.get_parameter('max_turn').value
        self.linear_axis = self.get_parameter('linear_axis').value
        self.angular_axis = self.get_parameter('angular_axis').value
        
        # เปิด Serial port
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.get_logger().info(f'เชื่อมต่อกับ {self.serial_port} ที่ {self.baudrate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'ไม่สามารถเปิด serial port: {e}')
            raise
        
        # Subscribe to Joy messages
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # ตัวแปรสำหรับเก็บค่าความเร็ว
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        
        # สร้าง thread สำหรับส่งคำสั่งอย่างต่อเนื่อง
        self.running = True
        self.send_thread = threading.Thread(target=self.send_command_loop)
        self.send_thread.daemon = True
        self.send_thread.start()
        
        self.get_logger().info('เริ่มต้น TeleopJoy Node แล้ว')
    
    def joy_callback(self, msg):
        """รับคำสั่งจาก joystick"""
        if len(msg.axes) > max(self.linear_axis, self.angular_axis):
            # อ่านค่าจาก joystick
            # สำหรับแกน Y เราต้องกลับค่า (เพราะ +Y หมายถึงเดินหน้า แต่ joystick ค่า Y ลบหมายถึงดันไปข้างหน้า)
            self.linear_speed = -msg.axes[self.linear_axis] 
            self.angular_speed = -msg.axes[self.angular_axis]
            
            # ปรับค่า deadzone
            if abs(self.linear_speed) < 0.1:
                self.linear_speed = 0.0
            if abs(self.angular_speed) < 0.1:
                self.angular_speed = 0.0
    
    def send_command_loop(self):
        """ส่งคำสั่งควบคุมมอเตอร์ไปยัง Pico"""
        while self.running:
            left_speed, right_speed = self.calculate_motor_speeds()
            command = {'left': left_speed, 'right': right_speed}
            
            try:
                self.ser.write((json.dumps(command) + '\r\n').encode())
                self.get_logger().info(f'ส่งคำสั่ง: {command}')
                time.sleep(0.05)  # ส่งคำสั่งทุก 50ms
            except Exception as e:
                self.get_logger().error(f'เกิดข้อผิดพลาดในการส่งคำสั่ง: {e}')
    
    def calculate_motor_speeds(self):
        """คำนวณความเร็วมอเตอร์จากความเร็วเชิงเส้นและความเร็วเชิงมุม"""
        # แปลงค่า joystick เป็นความเร็วมอเตอร์
        left_speed = int(self.max_speed * self.linear_speed - self.max_turn * self.angular_speed)
        right_speed = int(self.max_speed * self.linear_speed + self.max_turn * self.angular_speed)
        
        # จำกัดค่าให้อยู่ในช่วง -100 ถึง 100
        left_speed = max(min(left_speed, 100), -100)
        right_speed = max(min(right_speed, 100), -100)
        
        return left_speed, right_speed
    
    def shutdown(self):
        """หยุดการทำงานและปิด serial port"""
        self.running = False
        self.send_thread.join(timeout=1.0)
        self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    
    node = TeleopJoy()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
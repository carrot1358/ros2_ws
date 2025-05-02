#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import serial
import json
import sys
import termios
import tty
import select
import time

# คีย์สำหรับควบคุม
KEYS = {
    'w': (1, 0),     # เดินหน้า
    's': (-1, 0),    # ถอยหลัง
    'a': (0, 1),     # หมุนซ้าย
    'd': (0, -1),    # หมุนขวา
    'q': (0, 0)      # หยุด
}

# ข้อความแสดงวิธีใช้
MSG = """
ควบคุมหุ่นยนต์ด้วยคีย์บอร์ด
---------------------------
w/s : เดินหน้า/ถอยหลัง
a/d : เลี้ยวซ้าย/เลี้ยวขวา
q   : หยุด
space : หยุด
CTRL-C เพื่อออก
"""

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        # รับพารามิเตอร์จาก launch file หรือ command line
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('max_speed', 50)  # ความเร็วสูงสุดของมอเตอร์ (0-100)
        self.declare_parameter('max_turn', 30)   # ความเร็วเลี้ยวสูงสุด
        
        # อ่านค่าพารามิเตอร์
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_turn = self.get_parameter('max_turn').value
        
        # เปิด Serial port
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.get_logger().info(f'เชื่อมต่อกับ {self.serial_port} ที่ {self.baudrate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'ไม่สามารถเปิด serial port: {e}')
            raise
        
        # Subscribe to Twist messages (optional, for joystick)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
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
        
        self.get_logger().info('เริ่มต้น Teleop Node แล้ว')
        self.get_logger().info(MSG)
    
    def cmd_vel_callback(self, msg):
        """รับคำสั่ง velocity จาก topic cmd_vel (สำหรับ joystick)"""
        self.linear_speed = msg.linear.x
        self.angular_speed = msg.angular.z
    
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
        # แปลงค่า Twist เป็นความเร็วมอเตอร์
        left_speed = int(self.max_speed * self.linear_speed - self.max_turn * self.angular_speed)
        right_speed = int(self.max_speed * self.linear_speed + self.max_turn * self.angular_speed)
        
        # จำกัดค่าให้อยู่ในช่วง -100 ถึง 100
        left_speed = max(min(left_speed, 100), -100)
        right_speed = max(min(right_speed, 100), -100)
        
        return left_speed, right_speed
    
    def set_speeds_from_key(self, key):
        """ตั้งค่าความเร็วตามปุ่มที่กด"""
        if key in KEYS:
            linear, angular = KEYS[key]
            self.linear_speed = linear
            self.angular_speed = angular
            return True
        return False
    
    def shutdown(self):
        """หยุดการทำงานและปิด serial port"""
        self.running = False
        self.send_thread.join(timeout=1.0)
        self.ser.close()

def get_key(settings):
    """อ่านปุ่มที่กดจากคีย์บอร์ด"""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    
    try:
        # ตรวจสอบว่ารันในโหมด interactive terminal หรือไม่
        settings = termios.tcgetattr(sys.stdin)
        is_interactive_terminal = True
    except termios.error:
        # ถ้าไม่ใช่ interactive terminal จะใช้โหมด topic แทน
        is_interactive_terminal = False
        print("ไม่สามารถใช้งานคีย์บอร์ดได้ในสภาพแวดล้อมนี้")
        print("ใช้งานผ่าน topic cmd_vel แทน:")
        print("  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"linear: {x: 0.5}\" -1")
    
    node = TeleopNode()
    
    if is_interactive_terminal:
        try:
            # สร้าง thread สำหรับ ROS spin
            spin_thread = threading.Thread(target=lambda: rclpy.spin(node))
            spin_thread.daemon = True
            spin_thread.start()
            
            # ลูปหลักสำหรับรับปุ่มกด
            while rclpy.ok():
                key = get_key(settings)
                
                # กด Ctrl+C เพื่อออกจากโปรแกรม
                if key == '\x03':
                    break
                    
                # กดปุ่ม space เพื่อหยุด
                if key == ' ':
                    node.linear_speed = 0.0
                    node.angular_speed = 0.0
                else:
                    node.set_speeds_from_key(key)
                
        except Exception as e:
            print(f"เกิดข้อผิดพลาด: {e}")
        
        finally:
            # คืนค่า terminal เดิม
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    else:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
            
    # หยุดการทำงานและปิด node
    node.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
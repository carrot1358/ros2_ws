#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import serial
import json
import sys
import time
import select

# คีย์สำหรับควบคุม
KEYS = {
    'w': (1, 0),     # เดินหน้า
    's': (-1, 0),    # ถอยหลัง
    'a': (0, 1),     # หมุนซ้าย
    'd': (0, -1),    # หมุนขวา
    'q': (0, 0),     # หยุด
    ' ': (0, 0)      # space = หยุด
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
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
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
        self.current_left_speed = 0  # เพิ่มตัวแปรเก็บความเร็วล้อซ้าย
        self.current_right_speed = 0  # เพิ่มตัวแปรเก็บความเร็วล้อขวา
        
        # สร้าง timer สำหรับส่งคำสั่งอย่างต่อเนื่อง
        self.timer = self.create_timer(0.1, self.send_command)  # ส่งคำสั่งทุก 100ms
        
        # แสดงข้อความเริ่มต้น
        self.get_logger().info('เริ่มต้น Teleop Node แล้ว')
        self.get_logger().info(MSG)
    
    def cmd_vel_callback(self, msg):
        """รับคำสั่ง velocity จาก topic cmd_vel (สำหรับ joystick)"""
        self.linear_speed = msg.linear.x
        self.angular_speed = msg.angular.z
    
    def send_command(self):
        """ส่งคำสั่งควบคุมมอเตอร์ไปยัง Pico"""
        left_speed, right_speed = self.calculate_motor_speeds()
        
        # ตรวจสอบว่าค่าเปลี่ยนแปลงหรือไม่
        if left_speed != self.current_left_speed or right_speed != self.current_right_speed:
            # อัพเดตค่าปัจจุบัน
            self.current_left_speed = left_speed
            self.current_right_speed = right_speed
            
            command = {'left': left_speed, 'right': right_speed}
            
            try:
                self.ser.write((json.dumps(command) + '\r\n').encode())
                self.get_logger().info(f'ส่งคำสั่ง: {command} (linear={self.linear_speed}, angular={self.angular_speed})')
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
            print(f"ตั้งค่าความเร็ว: เดินหน้า={self.linear_speed}, หมุน={self.angular_speed}")
            return True
        return False
    
    def shutdown(self):
        """หยุดการทำงานและปิด serial port"""
        if hasattr(self, 'ser'):
            # ส่งคำสั่งหยุดก่อนปิด
            command = {'left': 0, 'right': 0}
            try:
                self.ser.write((json.dumps(command) + '\r\n').encode())
                self.get_logger().info('ส่งคำสั่งหยุดก่อนปิด')
                time.sleep(0.1)  # รอให้คำสั่งถูกส่ง
            except:
                pass
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    
    node = TeleopNode()
    
    print("กรุณาป้อนคำสั่ง: w=เดินหน้า, s=ถอยหลัง, a=เลี้ยวซ้าย, d=เลี้ยวขวา, q=หยุด")
    print("พิมพ์แล้วกด Enter หรือกด Ctrl+C เพื่อออก")
    
    # สร้าง thread สำหรับ ROS spin
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node))
    spin_thread.daemon = True
    spin_thread.start()
    
    try:
        # ลูปหลักแบบง่าย รับคำสั่งเมื่อผู้ใช้กด Enter
        while rclpy.ok():
            try:
                # รับคีย์จากผู้ใช้
                key = input().strip().lower()
                
                if key == 'exit':
                    break
                
                # ถ้าไม่มีข้อมูล
                if not key:
                    continue
                
                # ใช้ตัวอักษรแรก
                first_key = key[0]
                if first_key in "wsadq ":
                    node.set_speeds_from_key(first_key)
                else:
                    print(f"ไม่รู้จักคำสั่ง: {first_key}")
                
            except KeyboardInterrupt:
                break
            
    except Exception as e:
        print(f"เกิดข้อผิดพลาด: {e}")
    
    finally:
        # หยุดการทำงานและปิด node
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
#!/usr/bin/env python3
# keyboard_control.py - โปรแกรมควบคุมด้วยคีย์บอร์ดแบบโดยตรง

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import time
import threading
import termios
import tty
import select

# คีย์สำหรับควบคุม
KEYS = {
    'w': (1.0, 0.0),    # เดินหน้า
    's': (-1.0, 0.0),   # ถอยหลัง
    'a': (0.0, 1.0),    # หมุนซ้าย
    'd': (0.0, -1.0),   # หมุนขวา
    'q': (0.0, 0.0),    # หยุด
    ' ': (0.0, 0.0)     # space = หยุด
}

# ข้อความแสดงวิธีใช้
MSG = """
ควบคุมหุ่นยนต์ด้วยคีย์บอร์ด (แบบง่าย ไม่ต้องกด Enter)
---------------------------
w : เดินหน้า (ค้างไว้ได้)
s : ถอยหลัง (ค้างไว้ได้)
a : เลี้ยวซ้าย (ค้างไว้ได้)
d : เลี้ยวขวา (ค้างไว้ได้)
q : หยุด
space : หยุด

r : เพิ่มความเร็ว
f : ลดความเร็ว
t : เพิ่มความเร็วเลี้ยว
g : ลดความเร็วเลี้ยว
v : แสดงค่าความเร็วปัจจุบัน

Ctrl+C : ออกจากโปรแกรม
"""

def get_key(settings):
    """อ่านปุ่มที่กดจากคีย์บอร์ด"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.linear_x = 0.0
        self.angular_z = 0.0
        
        # เพิ่มตัวแปรความเร็ว
        self.speed_factor = 1.0  # ตัวคูณความเร็วเชิงเส้น
        self.turn_factor = 1.0   # ตัวคูณความเร็วเชิงมุม
        self.max_speed = 2.0     # ความเร็วสูงสุด
        self.max_turn = 2.0      # ความเร็วเลี้ยวสูงสุด
        self.min_speed = 0.1     # ความเร็วต่ำสุด
        self.speed_step = 0.1    # ขนาดการเพิ่ม/ลดความเร็ว
        
        self.cmd_active = False
        
        self.get_logger().info('เริ่มต้น KeyboardControl Node แล้ว')
        
        # แสดงค่าความเร็วเริ่มต้น
        self.print_speed_info()
    
    def print_speed_info(self):
        """แสดงข้อมูลความเร็วปัจจุบัน"""
        print("\n--- ค่าความเร็วปัจจุบัน ---")
        print(f"ความเร็วเดินหน้า/ถอยหลัง: {self.speed_factor:.1f}x (สูงสุด: {self.max_speed:.1f})")
        print(f"ความเร็วเลี้ยวซ้าย/ขวา: {self.turn_factor:.1f}x (สูงสุด: {self.max_turn:.1f})")
        print("-------------------------\n")
    
    def adjust_speed(self, increase=True, is_turn=False):
        """ปรับความเร็ว"""
        if is_turn:
            if increase:
                self.turn_factor = min(self.turn_factor + self.speed_step, self.max_turn)
            else:
                self.turn_factor = max(self.turn_factor - self.speed_step, self.min_speed)
        else:
            if increase:
                self.speed_factor = min(self.speed_factor + self.speed_step, self.max_speed)
            else:
                self.speed_factor = max(self.speed_factor - self.speed_step, self.min_speed)
        
        self.print_speed_info()
    
    def send_cmd(self, linear_x, angular_z):
        # ปรับความเร็วตามตัวคูณ
        actual_linear_x = linear_x * self.speed_factor
        actual_angular_z = angular_z * self.turn_factor
        
        msg = Twist()
        msg.linear.x = float(actual_linear_x)
        msg.angular.z = float(actual_angular_z)
        
        self.linear_x = actual_linear_x
        self.angular_z = actual_angular_z
        
        # แสดงว่ากำลังส่งคำสั่งอะไร
        if actual_linear_x != 0.0 or actual_angular_z != 0.0:
            self.cmd_active = True
            sys.stdout.write("\rกำลังส่งคำสั่ง: linear.x={:.2f}, angular.z={:.2f} [v={:.1f}x, r={:.1f}x]       ".format(
                actual_linear_x, actual_angular_z, self.speed_factor, self.turn_factor))
            sys.stdout.flush()
        elif self.cmd_active:
            self.cmd_active = False
            sys.stdout.write("\rหยุดแล้ว [v={:.1f}x, r={:.1f}x]                              ".format(
                self.speed_factor, self.turn_factor))
            sys.stdout.flush()
        
        # ส่งคำสั่ง
        self.publisher.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    
    print(MSG)
    
    # สร้าง thread สำหรับ ROS spin
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node))
    spin_thread.daemon = True
    spin_thread.start()
    
    try:
        # ลูปรับคำสั่งจากคีย์บอร์ด
        key_pressed = False
        key_hold_start = 0
        
        while True:
            key = get_key(node.settings)
            
            # ออกจากโปรแกรมถ้ากด Ctrl+C
            if key == '\x03':
                break
                
            # ถ้ามีการกดปุ่ม
            if key in KEYS:
                linear_x, angular_z = KEYS[key]
                node.send_cmd(linear_x, angular_z)
                key_pressed = True
                key_hold_start = time.time()
            elif key == 'r':  # เพิ่มความเร็ว
                node.adjust_speed(increase=True, is_turn=False)
            elif key == 'f':  # ลดความเร็ว
                node.adjust_speed(increase=False, is_turn=False)
            elif key == 't':  # เพิ่มความเร็วเลี้ยว
                node.adjust_speed(increase=True, is_turn=True)
            elif key == 'g':  # ลดความเร็วเลี้ยว
                node.adjust_speed(increase=False, is_turn=True)
            elif key == 'v':  # แสดงค่าความเร็วปัจจุบัน
                node.print_speed_info()
            elif key == '':
                # ถ้าไม่มีการกดปุ่มใหม่และได้ถึงเวลาหยุด
                if key_pressed and time.time() - key_hold_start > 0.2:
                    node.send_cmd(0.0, 0.0)
                    key_pressed = False
            
            # หน่วงเวลาเล็กน้อย
            time.sleep(0.01)
            
    except Exception as e:
        print(f"เกิดข้อผิดพลาด: {e}")
    finally:
        # หยุดหุ่นยนต์และคืนค่า terminal
        node.send_cmd(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        
        # ปิด node
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
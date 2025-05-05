from machine import Pin, UART, Timer, PWM
import time
import ujson
import math


# UART สำหรับส่ง odometry (GPIO 4 -> Pi GPIO 1)
uart_odom = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))

# UART สำหรับรับคำสั่งมอเตอร์ (GPIO 0 <- Pi GPIO 14)
uart_motor = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1)) 

# Pin Encoder ล้อซ้าย (A, B)
left_a = Pin(10, Pin.IN, Pin.PULL_UP)
left_b = Pin(11, Pin.IN, Pin.PULL_UP)

# Pin Encoder ล้อขวา (A, B)
right_a = Pin(23, Pin.IN, Pin.PULL_UP)
right_b = Pin(24, Pin.IN, Pin.PULL_UP)

sw1 = Pin(9, Pin.IN)
bz = Pin(21, Pin.OUT)

# ตัวนับ encoder
left_ticks = 0
right_ticks = 0

# Constants
TICKS_PER_REV = 894        # จำนวน Tick ต่อการหมุน 1 รอบของล้อ (ticks/rev)
WHEEL_RADIUS = 0.0215        # รัศมีล้อ (เมตร)
WHEEL_BASE = 0.093         # ระยะห่างล้อ (เมตร)

# ตำแหน่งเริ่มต้น
x, y, theta = 0.0, 0.0, 0.0

# กำหนด PWM frequency
PWM_FREQ = 20000 

# กำหนดขา
pwm_left = PWM(Pin(7))   # สำหรับความเร็วล้อซ้าย
dir1_left = Pin(8, Pin.OUT)
dir2_left = Pin(6, Pin.OUT)

pwm_right = PWM(Pin(22))  # สำหรับความเร็วล้อขวา
dir1_right = Pin(19, Pin.OUT)
dir2_right = Pin(18, Pin.OUT)

# กำหนดความถี่ PWM
pwm_left.freq(PWM_FREQ)
pwm_right.freq(PWM_FREQ)

# แน่ใจว่า pins เริ่มต้นที่ 0 (มอเตอร์หยุด)
dir1_left.value(0)
dir2_left.value(0)
pwm_left.duty_u16(0)

dir1_right.value(0)
dir2_right.value(0)
pwm_right.duty_u16(0)

# ฟังก์ชัน map
def map_value(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

# ฟังก์ชันควบคุมมอเตอร์
def Motor(spl, spr, log = False):
    if log:
        print(f"กำลังตั้งค่ามอเตอร์: L={spl}, R={spr}")
    time.sleep_us(50)

    # ทดสอบการคำนวณค่า
    test_values = [-100, -50, 0, 50, 100]
    if log:
        print("ทดสอบการแปลงค่า map_value:")
        for test_val in test_values:
            mapped = map_value(test_val, -100, 100, -65535, 65535)
            print(f"  Input: {test_val} -> Output: {mapped}")

    sl = map_value(spl, -100, 100, -65535, 65535)
    sr = map_value(spr, -100, 100, -65535, 65535)

    if log:
        print(f"ค่า PWM คำนวณ: L={sl}, R={sr}")
    # Clamp ค่าไว้ในช่วง
    sl = max(min(sl, 65535), -65535)
    sr = max(min(sr, 65535), -65535)

    # แสดงค่าที่จะใช้จริงในกรณีที่มีการ clamp
    if log:
        print(f"ค่า PWM หลัง clamp: L={sl}, R={sr}")

    # ควบคุมล้อซ้าย - สลับทิศทาง
    if sl > 0:
        if log:
            print(f"L: Forward, PWM={sl}")
        dir1_left.value(0)  # เปลี่ยนจาก 1 เป็น 0
        dir2_left.value(1)  # เปลี่ยนจาก 0 เป็น 1
        pwm_left.duty_u16(sl)
    elif sl < 0:
        if log:
            print(f"L: Backward, PWM={-sl}")
        dir1_left.value(1)  # เปลี่ยนจาก 0 เป็น 1
        dir2_left.value(0)  # เปลี่ยนจาก 1 เป็น 0
        pwm_left.duty_u16(-sl)
    else:
        if log:
            print("L: Stop")
        dir1_left.value(0)
        dir2_left.value(0)
        pwm_left.duty_u16(0)

    # ควบคุมล้อขวา - สลับทิศทาง
    if sr > 0:
        if log:
            print(f"R: Forward, PWM={sr}")
        dir1_right.value(0) 
        dir2_right.value(1) 
        pwm_right.duty_u16(sr)
    elif sr < 0:
        if log:
            print(f"R: Backward, PWM={-sr}")
        dir1_right.value(1)
        dir2_right.value(0)
        pwm_right.duty_u16(-sr)
    else:
        if log:
            print("R: Stop")
        dir1_right.value(0)
        dir2_right.value(0)
        pwm_right.duty_u16(0)

def a_bz(sec):
    bz.value(1)
    time.sleep(sec)
    bz.value(0)
    time.sleep(sec)


a_bz(0.05)
a_bz(0.05)

# Check switch state
print("ตรวจสอบสถานะ Switch SW1:", "LOW (กดอยู่)" if sw1.value() == 0 else "HIGH (ไม่ได้กด)")

# รอสวิตช์ถูกกด
print("กดสวิตช์ SW1 เพื่อเริ่มต้น...")
while sw1.value() == 1:
    print("รอการกดสวิตช์...")
    time.sleep(0.1)
    
print("เริ่มต้นโปรแกรม!")
a_bz(0.3)

# ฟังก์ชัน callback สำหรับอ่าน encoder ของล้อซ้าย
def left_callback(pin):
    global left_ticks
    if left_a.value() != left_b.value():  # เปลี่ยนเงื่อนไขการตรวจสอบ (จาก == เป็น !=)
        left_ticks += 1  # หากค่า A และ B ไม่ตรงกัน หมุนไปข้างหน้า
    else:
        left_ticks -= 1  # ถ้าตรงกัน แสดงว่าเป็นการหมุนย้อนกลับ

# ฟังก์ชัน callback สำหรับอ่าน encoder ของล้อขวา
def right_callback(pin):
    global right_ticks
    if right_a.value() != right_b.value():  # เปลี่ยนเงื่อนไขการตรวจสอบ (จาก == เป็น !=)
        right_ticks += 1  # หากค่า A และ B ไม่ตรงกัน หมุนไปข้างหน้า
    else:
        right_ticks -= 1  # ถ้าตรงกัน แสดงว่าเป็นการหมุนย้อนกลับ

left_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=left_callback)
right_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=right_callback)

# === ฟังก์ชันส่ง Odometry ===
def send_odom(timer):
    global left_ticks, right_ticks, x, y, theta

    lt = left_ticks
    rt = right_ticks
    left_ticks = 0
    right_ticks = 0

    # คำนวณการเคลื่อนที่
    left_dist = 2 * math.pi * WHEEL_RADIUS * (lt / TICKS_PER_REV)
    right_dist = 2 * math.pi * WHEEL_RADIUS * (rt / TICKS_PER_REV)

    distance = (left_dist + right_dist) / 2.0
    delta_theta = (left_dist - right_dist) / WHEEL_BASE  
    
    # คำนวณตำแหน่งใหม่
    x += distance * math.cos(theta)
    y += distance * math.sin(theta)
    theta += delta_theta

    v = distance / 0.05   # linear velocity
    w = delta_theta / 0.05  # angular velocity

    odom = {
        "x": round(x, 4),
        "y": round(y, 4),
        "theta": round(theta, 4),
        "v": round(v, 4),
        "w": round(w, 4)
    }
    
    # print(odom)

    uart_odom.write(ujson.dumps(odom) + "\r\n")

# ฟังก์ชันรับคำสั่งจาก UART
def check_uart_motor(log = False):
    global uart_motor
    if uart_motor.any():
        if log:
            print("มีข้อมูลเข้ามาที่ UART")
        try:
            cmd = uart_motor.readline()
            if log:
                print("Raw data:", cmd)
            if cmd:
                cmd_str = cmd.decode('utf-8').strip()
                if log:
                    print("Decoded:", cmd_str)
                
                # ตรวจสอบและตัดคำนำหน้า CMD: ออก
                if cmd_str.startswith('CMD:'):
                    cmd_str = cmd_str[4:]  # ตัด CMD: ออก
                
                cmd_data = ujson.loads(cmd_str)
                if log:
                    print("Parsed JSON:", cmd_data)
                
                if 'left' in cmd_data and 'right' in cmd_data:
                    left_speed = int(cmd_data['left'])
                    right_speed = int(cmd_data['right'])
                    if log:
                        print(f"Motor command: L={left_speed}, R={right_speed}")
                    Motor(left_speed, right_speed)
                    # ทดสอบส่งคำตอบกลับ
                    uart_motor.write("OK: Motor command received\r\n")
        except Exception as e:
            print("Error parsing command:", e)
            # ส่งข้อความแสดงข้อผิดพลาดกลับไป
            uart_motor.write(f"Error: {str(e)}\r\n")

# === เริ่ม Timer ทุก 50ms สำหรับ odometry ===
timer = Timer()
timer.init(freq=20, mode=Timer.PERIODIC, callback=send_odom)

# === ลูปหลักสำหรับรับคำสั่งควบคุมมอเตอร์ ===
while True:
    check_uart_motor(True)
    time.sleep_ms(10) 








from machine import Pin, Timer
import time

# Pin สำหรับ Encoder (ล้อซ้าย A และ B) และ (ล้อขวา A และ B)
left_a = Pin(10, Pin.IN, Pin.PULL_UP)
left_b = Pin(11, Pin.IN, Pin.PULL_UP)

right_a = Pin(23, Pin.IN, Pin.PULL_UP)
right_b = Pin(24, Pin.IN, Pin.PULL_UP)

# ตัวแปรนับ ticks สำหรับล้อซ้ายและขวา
left_ticks = 0
right_ticks = 0

# ฟังก์ชัน callback สำหรับอ่าน encoder ของล้อซ้าย
def left_callback(pin):
    global left_ticks
    if left_a.value() == left_b.value():  # ตรวจสอบว่า A และ B มีค่าสอดคล้องกัน
        left_ticks += 1  # หากค่า A และ B ตรงกัน หมุนไปข้างหน้า
    else:
        left_ticks -= 1  # ถ้าไม่ตรงกัน แสดงว่าเป็นการหมุนย้อนกลับ

# ฟังก์ชัน callback สำหรับอ่าน encoder ของล้อขวา
def right_callback(pin):
    global right_ticks
    if right_a.value() == right_b.value():  # ตรวจสอบว่า A และ B มีค่าสอดคล้องกัน
        right_ticks += 1  # หากค่า A และ B ตรงกัน หมุนไปข้างหน้า
    else:
        right_ticks -= 1  # ถ้าไม่ตรงกัน แสดงว่าเป็นการหมุนย้อนกลับ

# กำหนด IRQ สำหรับ encoder ของล้อซ้ายและขวา
left_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=left_callback)
right_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=right_callback)

# ฟังก์ชันหลักที่ใช้แสดงผล
def main():
    global left_ticks, right_ticks
    print("เริ่มหมุนล้อและโปรแกรมจะแสดงจำนวน ticks ของแต่ละล้อ")

    # เริ่มนับ ticks ของทั้งสองล้อ
    while True:
        print(f"จำนวน ticks ล้อซ้าย: {left_ticks}, จำนวน ticks ล้อขวา: {right_ticks}")
        time.sleep(0.5)  # เช็คทุก 500ms เพื่อให้โปรแกรมไม่แสดงผลเร็วเกินไป
        
        # คุณสามารถหยุดการนับเองได้ตามที่ต้องการ
        # เมื่อหมุนครบ 1 รอบหรือตามที่คุณบอก

# เริ่มโปรแกรม
main()

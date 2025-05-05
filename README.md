# 🤖 ROS2 Workspace สำหรับระบบ SLAM

โปรเจคนี้เป็น ROS2 workspace สำหรับการพัฒนาระบบหุ่นยนต์และระบบอัตโนมัติ โดยเฉพาะการทำแผนที่และระบบนำทางด้วย SLAM (Simultaneous Localization and Mapping) ใช้กับฮาร์ดแวร์ Raspberry Pi และ RPLidar

## 📋 คุณสมบัติหลัก

- การสร้างแผนที่ด้วย SLAM Toolbox
- รองรับเซ็นเซอร์ RPLidar A1 สำหรับการสแกนสภาพแวดล้อม
- ระบบ Odometry ผ่าน UART สำหรับติดตามการเคลื่อนที่
- สร้างและจัดเก็บแผนที่สำหรับการนำทางอัตโนมัติ

## 🔧 การติดตั้ง

### ความต้องการเบื้องต้น

- Ubuntu 20.04 หรือ 22.04
- ROS2 (แนะนำ Humble/Galactic)
- Python 3.8+
- Raspberry Pi (ทดสอบบน Raspberry Pi 4)
- RPLidar A1

### การติดตั้ง ROS2

ก่อนที่จะใช้งานโปรเจคนี้ คุณต้องติดตั้ง ROS2 บนเครื่องของคุณ สามารถดูวิธีการติดตั้งได้ที่ [เอกสารการติดตั้ง ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### การติดตั้งแพ็คเกจที่จำเป็น

```bash
sudo apt update
sudo apt install -y python3-pip python3-serial ros-$ROS_DISTRO-slam-toolbox ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup
pip3 install pyserial
```

## 💻 การใช้งาน

### 1. โคลนโปรเจค

```bash
git clone https://github.com/carrot1358/ros2_ws.git
cd ~/ros2_ws
```

### 2. คอมไพล์โปรเจค

```bash
colcon build
source ~/ros2_ws/install/setup.bash
```

### 3. การเรียกใช้ระบบ SLAM แบบสมบูรณ์

คำสั่งนี้จะเริ่มการทำงานทุก node ที่จำเป็นพร้อมกัน:

```bash
ros2 launch carrot_ros slam_launch.py
```

ชุดคำสั่งนี้จะรันโหนดต่างๆ พร้อมกัน:
- uart_odom_node: โหนดสำหรับ odometry
- rplidar_a1_launch: โหนดสำหรับ RPLidar
- static_transform_publisher: โหนดสำหรับ TF transformation
- slam_toolbox: โหนดสำหรับการทำ SLAM
- lifecycle_manager: โหนดสำหรับการจัดการวงจรชีวิตของระบบ

### 4. การเรียกใช้โหนดแยกกัน

หากต้องการรันทีละโหนด:

```bash
# สำหรับ Odometry node
ros2 run uart_odom_publisher uart_odom_node

# สำหรับ RPLidar node
ros2 launch rplidar_ros rplidar_a1_launch.py 
```

### 5. การบันทึกแผนที่

เมื่อสร้างแผนที่เสร็จแล้ว ใช้คำสั่งต่อไปนี้เพื่อบันทึกแผนที่:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_map'}}"
```

## 📁 โครงสร้างของโปรเจค

```
ros2_ws/
├── src/                    # ไดเรกทอรีที่เก็บแพ็คเกจ ROS2
│   ├── carrot_ros/         # แพ็คเกจหลักสำหรับระบบ SLAM
│   │   ├── config/         # ไฟล์การตั้งค่าต่างๆ
│   │   │   └── slam_toolbox.yaml
│   │   └── launch/         # ไฟล์ launch 
│   │       └── slam_launch.py
│   ├── uart_odom_publisher/ # แพ็คเกจสำหรับ Odometry
│   └── rplidar_ros/         # แพ็คเกจสำหรับ RPLidar
├── build/                  # ไดเรกทอรีที่เก็บไฟล์ที่คอมไพล์แล้ว
├── install/                # ไดเรกทอรีที่เก็บไฟล์ที่ติดตั้งแล้ว
└── log/                    # ไดเรกทอรีที่เก็บไฟล์ล็อก
```

## ⚙️ การเพิ่มแพ็คเกจใหม่

### การสร้างแพ็คเกจ C++

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
```

### การสร้างแพ็คเกจ Python

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --node-name my_node my_python_package
```

## 🔍 การแก้ไขปัญหาเบื้องต้น

- **ปัญหาการเชื่อมต่อ RPLidar**: ตรวจสอบสิทธิ์การเข้าถึงพอร์ต USB ด้วยคำสั่ง `sudo chmod 666 /dev/ttyUSB0`
- **ปัญหา UART**: ตรวจสอบการตั้งค่า baudrate และพอร์ตใน slam_launch.py
- **แผนที่ไม่อัพเดท**: ตรวจสอบการตั้งค่าใน slam_toolbox.yaml

## 📞 การติดต่อ

หากมีคำถามหรือต้องการความช่วยเหลือ กรุณาติดต่อ "แครอท" ผู้ดูแลโปรเจค 
- อีเมล: carrot1358@gmail.com
- GitHub: [carrot1358](https://github.com/carrot1358)

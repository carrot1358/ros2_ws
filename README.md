# ROS2 Workspace

โปรเจคนี้เป็น ROS2 workspace สำหรับการพัฒนาระบบหุ่นยนต์และระบบอัตโนมัติ

## การติดตั้ง

ก่อนที่จะใช้งานโปรเจคนี้ คุณต้องติดตั้ง ROS2 บนเครื่องของคุณ สามารถดูวิธีการติดตั้งได้ที่ [เอกสารการติดตั้ง ROS2](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

## การใช้งาน

การคอมไพล์โปรเจค:
```bash
cd ~/ros2_ws
colcon build
```

การเรียกใช้โปรเจค:
```bash
source ~/ros2_ws/install/setup.bash
ros2 run <package_name> <executable_name>
```

## โครงสร้างของโปรเจค

- `src/` - ไดเรกทอรีที่เก็บแพ็คเกจ ROS2
- `build/` - ไดเรกทอรีที่เก็บไฟล์ที่คอมไพล์แล้ว
- `install/` - ไดเรกทอรีที่เก็บไฟล์ที่ติดตั้งแล้ว
- `log/` - ไดเรกทอรีที่เก็บไฟล์ล็อก

## การสร้างแพ็คเกจใหม่

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
```

## การติดต่อ

หากมีคำถามหรือต้องการความช่วยเหลือ กรุณาติดต่อ "แครอท" ผู้ดูแลโปรเจค

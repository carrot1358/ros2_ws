#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nlohmann/json.hpp>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>

using json = nlohmann::json;
using namespace std::chrono_literals;

class CmdVelToMotors : public rclcpp::Node
{
public:
  CmdVelToMotors() : Node("cmd_vel_to_motors")
  {
    // รับพารามิเตอร์
    this->declare_parameter("serial_port", "/dev/ttyAMA0");
    this->declare_parameter("baudrate", 115200);
    this->declare_parameter("max_speed", 80);  // เพิ่มความเร็วสูงสุดของมอเตอร์เป็น 80 (0-100)
    this->declare_parameter("max_turn", 50);   // เพิ่มความเร็วเลี้ยวสูงสุดเป็น 50
    this->declare_parameter("linear_scale", 4.0);  // เพิ่มสเกลความเร็วเชิงเส้น (เดิม 2.0)
    this->declare_parameter("angular_scale", 2.0); // เพิ่มสเกลความเร็วเชิงมุม (เดิม 1.0)
    this->declare_parameter("run_motion_test", false); // เพิ่มพารามิเตอร์สำหรับเปิด/ปิดการทดสอบเคลื่อนที่

    serial_port_ = this->get_parameter("serial_port").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();
    max_speed_ = this->get_parameter("max_speed").as_int();
    max_turn_ = this->get_parameter("max_turn").as_int();
    linear_scale_ = this->get_parameter("linear_scale").as_double();
    angular_scale_ = this->get_parameter("angular_scale").as_double();
    run_motion_test_ = this->get_parameter("run_motion_test").as_bool();

    // เปิด serial port
    serial_fd_ = open_serial_port(serial_port_, baudrate_);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "ไม่สามารถเปิด serial port: %s", serial_port_.c_str());
      throw std::runtime_error("Failed to open serial port");
    }
    RCLCPP_INFO(this->get_logger(), "เชื่อมต่อกับ %s ที่ %d baud", serial_port_.c_str(), baudrate_);
    RCLCPP_INFO(this->get_logger(), "ตั้งค่า max_speed=%d, max_turn=%d, linear_scale=%.1f, angular_scale=%.1f", 
                max_speed_, max_turn_, linear_scale_, angular_scale_);

    // Subscribe to cmd_vel
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, 
      std::bind(&CmdVelToMotors::cmd_vel_callback, this, std::placeholders::_1));

    // สร้าง thread สำหรับส่งคำสั่งอย่างต่อเนื่อง
    running_ = true;
    send_thread_ = std::thread(&CmdVelToMotors::send_command_loop, this);

    RCLCPP_INFO(this->get_logger(), "เริ่มต้น CmdVelToMotors Node แล้ว");
    
    // ทดสอบการเคลื่อนที่หากเปิดใช้งานพารามิเตอร์
    if (run_motion_test_) {
      RCLCPP_INFO(this->get_logger(), "จะเริ่มการทดสอบเคลื่อนที่ในอีก 3 วินาที...");
      motion_test_timer_ = this->create_wall_timer(
        3000ms, std::bind(&CmdVelToMotors::run_direct_motion_test, this));
    }
  }

  ~CmdVelToMotors()
  {
    shutdown();
  }

  void shutdown()
  {
    if (running_) {
      running_ = false;
      if (send_thread_.joinable()) {
        send_thread_.join();
      }
      
      // ยกเลิก timer ถ้ายังทำงานอยู่
      if (motion_test_timer_) {
        motion_test_timer_->cancel();
      }
      
      if (serial_fd_ >= 0) {
        // ส่งคำสั่งหยุดมอเตอร์ก่อนปิด
        int left_speed = 0;
        int right_speed = 0;
        json cmd = {{"left", left_speed}, {"right", right_speed}};
        std::string cmd_str = "CMD:" + cmd.dump() + "\r\n";
        write(serial_fd_, cmd_str.c_str(), cmd_str.length());
        RCLCPP_INFO(this->get_logger(), "ส่งคำสั่งหยุดมอเตอร์ก่อนปิด: %s", cmd.dump().c_str());
        
        // ปิด serial port
        close(serial_fd_);
        serial_fd_ = -1;
      }
    }
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // หลังจากแก้ไข URDF เรียบร้อยแล้ว ไม่จำเป็นต้องกลับทิศทางอีกต่อไป
    linear_speed_ = msg->linear.x;  // ใช้ค่าโดยตรงไม่ต้องกลับทิศทาง
    angular_speed_ = msg->angular.z;  // ใช้ค่าโดยตรงไม่ต้องกลับทิศทาง
    
    RCLCPP_INFO(this->get_logger(), "Received cmd_vel - linear: %.2f, angular: %.2f", 
              msg->linear.x, msg->angular.z);
  }
  
  // ฟังก์ชันทดสอบแบบใหม่ที่ทำงานโดยตรงไม่ผ่านเธรด
  void run_direct_motion_test() 
  {
    // ยกเลิกไทเมอร์หลังจากทำงานครั้งแรก
    if (motion_test_timer_) {
      motion_test_timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "============= เริ่มการทดสอบเคลื่อนที่ =============");
    }

    // สร้าง Twist message
    auto twist = std::make_shared<geometry_msgs::msg::Twist>();
    
    // เดินหน้า
    RCLCPP_INFO(this->get_logger(), "📊 ทดสอบการเดินหน้า...");
    
    // ตั้งค่าสำหรับเดินหน้า
    twist->linear.x = 0.2;  // เดินหน้า 0.2 m/s
    twist->angular.z = 0.0;
    
    // ส่งคำสั่งโดยตรง
    {
      std::lock_guard<std::mutex> lock(mutex_);
      linear_speed_ = twist->linear.x;  // ใช้ค่าโดยตรงไม่ต้องกลับทิศทาง
      angular_speed_ = twist->angular.z;
    }
    
    // ปล่อยให้เคลื่อนที่เดินหน้า 3 วินาที
    for (int i = 0; i < 30; i++) {
      // ทุก 10 รอบ (ประมาณ 1 วินาที) แสดงสถานะการทดสอบ
      if (i % 10 == 0) {
        RCLCPP_INFO(this->get_logger(), "⏳ กำลังเดินหน้า... (เวลาผ่านไป %d วินาที)", i/10);
      }
      std::this_thread::sleep_for(100ms);
    }
    
    // หยุดนิ่ง
    RCLCPP_INFO(this->get_logger(), "🛑 หยุดนิ่ง...");
    {
      std::lock_guard<std::mutex> lock(mutex_);
      linear_speed_ = 0.0;
      angular_speed_ = 0.0;
    }
    // หยุด 2 วินาที
    std::this_thread::sleep_for(2s);
    
    // ถอยหลัง
    RCLCPP_INFO(this->get_logger(), "📊 ทดสอบการถอยหลัง...");
    {
      std::lock_guard<std::mutex> lock(mutex_);
      linear_speed_ = -0.2;  // ถอยหลัง (ค่าลบคือถอยหลัง)
      angular_speed_ = 0.0;
    }
    // ถอยหลัง 3 วินาที
    for (int i = 0; i < 30; i++) {
      if (i % 10 == 0) {
        RCLCPP_INFO(this->get_logger(), "⏳ กำลังถอยหลัง... (เวลาผ่านไป %d วินาที)", i/10);
      }
      std::this_thread::sleep_for(100ms);
    }
    
    // หยุดนิ่ง
    RCLCPP_INFO(this->get_logger(), "🛑 หยุดนิ่ง...");
    {
      std::lock_guard<std::mutex> lock(mutex_);
      linear_speed_ = 0.0;
      angular_speed_ = 0.0;
    }
    // หยุด 2 วินาที
    std::this_thread::sleep_for(2s);
    
    // หมุนซ้าย
    RCLCPP_INFO(this->get_logger(), "📊 ทดสอบการหมุนซ้าย...");
    {
      std::lock_guard<std::mutex> lock(mutex_);
      linear_speed_ = 0.0;
      angular_speed_ = 0.5;  // หมุนซ้าย
    }
    // หมุนซ้าย 3 วินาที
    for (int i = 0; i < 30; i++) {
      if (i % 10 == 0) {
        RCLCPP_INFO(this->get_logger(), "⏳ กำลังหมุนซ้าย... (เวลาผ่านไป %d วินาที)", i/10);
      }
      std::this_thread::sleep_for(100ms);
    }
    
    // หมุนขวา
    RCLCPP_INFO(this->get_logger(), "📊 ทดสอบการหมุนขวา...");
    {
      std::lock_guard<std::mutex> lock(mutex_);
      linear_speed_ = 0.0;
      angular_speed_ = -0.5;  // หมุนขวา
    }
    // หมุนขวา 3 วินาที
    for (int i = 0; i < 30; i++) {
      if (i % 10 == 0) {
        RCLCPP_INFO(this->get_logger(), "⏳ กำลังหมุนขวา... (เวลาผ่านไป %d วินาที)", i/10);
      }
      std::this_thread::sleep_for(100ms);
    }
    
    // สิ้นสุดการทดสอบ
    RCLCPP_INFO(this->get_logger(), "🛑 หยุดนิ่ง (จบการทดสอบ)");
    {
      std::lock_guard<std::mutex> lock(mutex_);
      linear_speed_ = 0.0;
      angular_speed_ = 0.0;
    }
    
    RCLCPP_INFO(this->get_logger(), "============= จบการทดสอบเคลื่อนที่ =============");
  }

  void send_command_loop()
  {
    json last_command;
    
    while (running_) {
      int left_speed, right_speed;
      
      {
        std::lock_guard<std::mutex> lock(mutex_);
        calculate_motor_speeds(linear_speed_, angular_speed_, left_speed, right_speed);
      }
      
      json command = {{"left", left_speed}, {"right", right_speed}};
      
      // ส่งคำสั่งเฉพาะเมื่อมีการเปลี่ยนแปลง
      if (command != last_command) {
        try {
          // เพิ่มคำนำหน้า "CMD:" เพื่อให้ฝั่ง odometry สามารถแยกแยะได้
          std::string cmd_str = "CMD:" + command.dump() + "\r\n";
          write(serial_fd_, cmd_str.c_str(), cmd_str.length());
          RCLCPP_INFO(this->get_logger(), "ส่งคำสั่ง: %s (linear=%.2f, angular=%.2f)", 
                    command.dump().c_str(), linear_speed_, angular_speed_);
          last_command = command;
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "เกิดข้อผิดพลาดในการส่งคำสั่ง: %s", e.what());
        }
      }
      
      std::this_thread::sleep_for(50ms);  // ส่งคำสั่งทุก 50ms
    }
  }

  void calculate_motor_speeds(double linear, double angular, int& left_speed, int& right_speed)
  {
    // แปลงค่า cmd_vel เป็นความเร็วมอเตอร์
    // ค่า linear และ angular อยู่ในหน่วย m/s และ rad/s
    // ต้องปรับให้เป็นเปอร์เซ็นต์ความเร็วมอเตอร์ (0-100)
    
    // เพิ่มค่าสเกลมากขึ้นจากเดิม
    // linear_scale = 4.0 หมายถึง 0.25 m/s -> 100%
    // angular_scale = 2.0 หมายถึง 0.5 rad/s -> 100%
    
    int linear_percent = static_cast<int>(linear * linear_scale_ * 100);
    int angular_percent = static_cast<int>(angular * angular_scale_ * 100);
    
    // จำกัดค่า
    linear_percent = std::min(std::max(linear_percent, -100), 100);
    angular_percent = std::min(std::max(angular_percent, -100), 100);
    
    // เพิ่มค่า speed boost สำหรับพื้นผิวที่มีแรงเสียดทาน
    int static_boost = 0;
    if (linear_percent != 0) {
      // เพิ่มแรงเริ่มต้น 20 หน่วยเมื่อมีการเคลื่อนที่ (เพิ่มจาก 10 เป็น 20)
      static_boost = (linear_percent > 0) ? 20 : -20;
    }
    
    // แปลงเป็นความเร็วมอเตอร์
    left_speed = static_cast<int>(max_speed_ * linear_percent / 100.0 - max_turn_ * angular_percent / 100.0);
    right_speed = static_cast<int>(max_speed_ * linear_percent / 100.0 + max_turn_ * angular_percent / 100.0);
    
    // เพิ่ม boost ให้กับทั้งสองล้อเมื่อมีการเคลื่อนที่
    if (linear_percent != 0) {
      if (left_speed > 0) {
        left_speed += static_boost;
      } else if (left_speed < 0) {
        left_speed -= static_boost;
      }
      
      if (right_speed > 0) {
        right_speed += static_boost;
      } else if (right_speed < 0) {
        right_speed -= static_boost;
      }
    }
    
    // จำกัดค่าให้อยู่ในช่วง -100 ถึง 100
    left_speed = std::min(std::max(left_speed, -100), 100);
    right_speed = std::min(std::max(right_speed, -100), 100);
    
    // เพิ่ม Debug Log
    static std::string last_data;
    std::string current_data = "Motor calculation: lin=" + std::to_string(linear_percent) + "%, ang=" + std::to_string(angular_percent) + "%, L=" + std::to_string(left_speed) + ", R=" + std::to_string(right_speed) + " (boost=" + std::to_string(static_boost) + ")";
    if (current_data != last_data) {
      RCLCPP_INFO(rclcpp::get_logger("cmd_vel_to_motors"), 
                current_data.c_str());
      last_data = current_data;
    }
  }

  int open_serial_port(const std::string& port, int baudrate)
  {
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
      RCLCPP_ERROR(this->get_logger(), "ไม่สามารถเปิด %s: %s", port.c_str(), strerror(errno));
      return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "ไม่สามารถอ่านค่า terminal attributes: %s", strerror(errno));
      close(fd);
      return -1;
    }

    // ตั้งค่า baud rate
    speed_t baud;
    switch (baudrate) {
      case 9600: baud = B9600; break;
      case 19200: baud = B19200; break;
      case 38400: baud = B38400; break;
      case 57600: baud = B57600; break;
      case 115200: baud = B115200; break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Baudrate ไม่รองรับ: %d", baudrate);
        close(fd);
        return -1;
    }

    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cflag &= ~PARENB;        // ปิด parity
    tty.c_cflag &= ~CSTOPB;        // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;            // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS;       // ปิด RTS/CTS flow control
    tty.c_cflag |= CREAD | CLOCAL; // เปิดการอ่านและละเลย modem control lines

    tty.c_lflag &= ~ICANON;        // ปิด canonical mode
    tty.c_lflag &= ~ECHO;          // ปิด echo
    tty.c_lflag &= ~ECHOE;         // ปิด echo erase
    tty.c_lflag &= ~ECHONL;        // ปิด echo new line
    tty.c_lflag &= ~ISIG;          // ปิด interpretation of INTR, QUIT and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // ปิด XON/XOFF flow control
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // ปิด special handling

    tty.c_oflag &= ~OPOST;         // ปิด output processing
    tty.c_oflag &= ~ONLCR;         // ปิด conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 10;          // timeout 1 second
    tty.c_cc[VMIN] = 0;            // อ่านไม่รอ

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "ไม่สามารถตั้งค่า terminal attributes: %s", strerror(errno));
      close(fd);
      return -1;
    }

    return fd;
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::string serial_port_;
  int baudrate_;
  int max_speed_;
  int max_turn_;
  double linear_scale_;   // เพิ่มตัวแปรเก็บค่าสเกลความเร็วเชิงเส้น
  double angular_scale_;  // เพิ่มตัวแปรเก็บค่าสเกลความเร็วเชิงมุม
  int serial_fd_ = -1;
  
  double linear_speed_ = 0.0;
  double angular_speed_ = 0.0;
  
  std::thread send_thread_;
  std::mutex mutex_;
  bool running_ = false;
  
  // ตัวแปรสำหรับการทดสอบเคลื่อนที่
  bool run_motion_test_ = false;
  rclcpp::TimerBase::SharedPtr motion_test_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelToMotors>();
  rclcpp::spin(node);
  node->shutdown();
  rclcpp::shutdown();
  return 0;
} 
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class ClickNavigationNode : public rclcpp::Node
{
public:
  ClickNavigationNode() : Node("click_navigation_node")
  {
    RCLCPP_INFO(this->get_logger(), "Starting Click Navigation Node");

    // สร้าง subscription รับ geometry_msgs/PointStamped จากการคลิกบน RViz
    // (RViz ส่ง /clicked_point เมื่อใช้ Publish Point tool)
    click_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10, 
      std::bind(&ClickNavigationNode::clickPointCallback, this, std::placeholders::_1));

    // สร้าง subscription รับ geometry_msgs/PoseStamped จากการกำหนดเป้าหมายบน RViz
    // (RViz ส่ง /goal_pose เมื่อใช้ 2D Nav Goal tool)
    goal_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10, 
      std::bind(&ClickNavigationNode::clickPoseCallback, this, std::placeholders::_1));

    // สร้าง publisher สำหรับแสดงเครื่องหมายเส้นทาง
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoint_markers", 10);

    // สร้าง action client สำหรับส่งเป้าหมายให้กับ Nav2
    nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, "navigate_to_pose");

    // ตั้งค่าแรกเริ่ม
    waypoint_id_ = 0;
    navigating_ = false;
    
    // สร้าง timer สำหรับตรวจสอบสถานะและส่งเป้าหมายถัดไป
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&ClickNavigationNode::timerCallback, this));
  }

private:
  // รับจุดคลิกจาก RViz (Publish Point tool)
  void clickPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose.position = msg->point;
    pose.pose.orientation.w = 1.0;  // ไม่มีการหมุน (quaternion)
    
    RCLCPP_INFO(this->get_logger(), "เพิ่มจุดพิกัดจากการคลิกพอยท์: %.2f, %.2f", 
                msg->point.x, msg->point.y);
    
    // เพิ่มเป้าหมายและเริ่มการนำทาง
    addWaypoint(pose);
  }
  
  // รับเป้าหมายจากการใช้ 2D Nav Goal บน RViz
  void clickPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "เพิ่มจุดพิกัดจากเป้าหมาย 2D: %.2f, %.2f", 
                msg->pose.position.x, msg->pose.position.y);
    
    // เพิ่มเป้าหมายและเริ่มการนำทาง
    addWaypoint(*msg);
  }
  
  // เพิ่มจุดหมายและเริ่มการนำทาง
  void addWaypoint(const geometry_msgs::msg::PoseStamped& pose)
  {
    waypoints_.push_back(pose);
    
    // เพิ่มเครื่องหมายแสดงเป้าหมาย
    addWaypointMarker(pose);
    
    // หากยังไม่ได้กำลังนำทาง ให้เริ่มนำทางไปยังเป้าหมายแรก
    if (!navigating_ && waypoints_.size() == 1) {
      navigateToNextWaypoint();
    }
  }
  
  // เพิ่มเครื่องหมายแสดงเป้าหมายบนแผนที่
  void addWaypointMarker(const geometry_msgs::msg::PoseStamped& pose)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;
    
    marker.header = pose.header;
    marker.ns = "waypoints";
    marker.id = waypoint_id_++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose.pose;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime.sec = 0; // ไม่หมดอายุ
    
    marker_array.markers.push_back(marker);
    marker_publisher_->publish(marker_array);
  }
  
  // นำทางไปยังเป้าหมายถัดไป
  void navigateToNextWaypoint()
  {
    if (waypoints_.empty()) {
      navigating_ = false;
      RCLCPP_INFO(this->get_logger(), "การนำทางเสร็จสิ้น - ไม่มีจุดพิกัดที่เหลือ");
      return;
    }
    
    // ตรวจสอบว่า action client พร้อมใช้งาน
    if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server ไม่พร้อมใช้งาน!");
      return;
    }
    
    // สร้าง goal
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = waypoints_.front(); // ใช้เป้าหมายแรกในรายการ
    
    RCLCPP_INFO(this->get_logger(), "กำลังนำทางไปยังจุดพิกัด: %.2f, %.2f", 
              goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y);
    
    navigating_ = true;
    
    // ส่ง goal
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
      std::bind(&ClickNavigationNode::goalResponseCallback, this, std::placeholders::_1);
    
    send_goal_options.feedback_callback =
      std::bind(&ClickNavigationNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    
    send_goal_options.result_callback =
      std::bind(&ClickNavigationNode::resultCallback, this, std::placeholders::_1);
    
    nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
  }
  
  // callback เมื่อได้รับการตอบกลับจาก goal request
  void goalResponseCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr& goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "เป้าหมายถูกปฏิเสธ!");
      navigating_ = false;
    } else {
      RCLCPP_INFO(this->get_logger(), "เป้าหมายได้รับการยอมรับ");
    }
  }
  
  // callback เมื่อได้รับข้อมูลความคืบหน้า
  void feedbackCallback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
  {
    // สามารถใช้ feedback ในอนาคตหากต้องการ
    auto distance = feedback->distance_remaining;
    RCLCPP_DEBUG(this->get_logger(), "ระยะทางคงเหลือ: %.2f เมตร", distance);
  }
  
  // callback เมื่อได้รับผลลัพธ์
  void resultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result)
  {
    navigating_ = false;
    
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "การนำทางสำเร็จ!");
        
        // นำเป้าหมายที่เพิ่งไปเสร็จออกจากรายการ
        if (!waypoints_.empty()) {
          waypoints_.erase(waypoints_.begin());
        }
        
        // ไปยังเป้าหมายถัดไปหากมี
        if (!waypoints_.empty()) {
          navigateToNextWaypoint();
        }
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "เป้าหมายถูกยกเลิก");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "เป้าหมายถูกยกเลิก");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "ผลลัพธ์ไม่ทราบสถานะ");
        break;
    }
  }
  
  void timerCallback()
  {
    // ตรวจสอบสถานะและดำเนินการหากจำเป็น
    if (!navigating_ && !waypoints_.empty()) {
      navigateToNextWaypoint();
    }
  }
  
  // ตัวแปรสมาชิก
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  int waypoint_id_;
  bool navigating_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ClickNavigationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 
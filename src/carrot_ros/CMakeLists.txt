cmake_minimum_required(VERSION 3.8)
project(carrot_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)

# Add nlohmann_json dependency for cmd_vel_to_motors
include(FetchContent)
FetchContent_Declare(json
    URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz
)
FetchContent_MakeAvailable(json)

# Add include directories
include_directories(include)

# สร้างและติดตั้ง click_navigation_node
add_executable(click_navigation_node src/click_navigation_node.cpp)
ament_target_dependencies(click_navigation_node
  rclcpp
  geometry_msgs
  visualization_msgs
  nav2_msgs
  rclcpp_action
  tf2_ros
)

# สร้างและติดตั้ง cmd_vel_to_motors
add_executable(cmd_vel_to_motors src/cmd_vel_to_motors.cpp)
ament_target_dependencies(cmd_vel_to_motors
  rclcpp
  geometry_msgs
)
target_link_libraries(cmd_vel_to_motors nlohmann_json::nlohmann_json)

# Install launch and config files
install(DIRECTORY
  launch
  config
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install executables
install(TARGETS
  click_navigation_node
  cmd_vel_to_motors
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

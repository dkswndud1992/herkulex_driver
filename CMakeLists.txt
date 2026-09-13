cmake_minimum_required(VERSION 3.8)
project(herkulex_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ─── Find dependencies ─────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ─── Generate interfaces (msg/srv) ─────────────────────────────
set(msg_files
  "msg/ServoStatus.msg"
  "msg/ServoStatusArray.msg"
  "msg/SyncAngleCmd.msg"
  "msg/SyncPositionCmd.msg"
)

set(srv_files
  "srv/SetPosition.srv"
  "srv/SetAngle.srv"
  "srv/SetSpeed.srv"
  "srv/SetTorque.srv"
  "srv/SetLed.srv"
  "srv/GetPosition.srv"
  "srv/Reboot.srv"
  "srv/ClearError.srv"
  "srv/SetID.srv"
  "srv/WriteRegistry.srv"
  "srv/SetGain.srv"
  "srv/GetGain.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs builtin_interfaces
)

# ─── HerkuleX serial library ───────────────────────────────────
add_library(herkulex_serial SHARED
  src/herkulex_serial.cpp
)

target_include_directories(herkulex_serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(herkulex_serial
  rclcpp
)

# ─── HerkuleX ROS2 node ────────────────────────────────────────
add_executable(herkulex_node
  src/herkulex_node.cpp
)

target_include_directories(herkulex_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(herkulex_node
  herkulex_serial
)

ament_target_dependencies(herkulex_node
  rclcpp
  rclcpp_components
  std_msgs
)

# Link generated interfaces to the node executable
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(herkulex_node "${cpp_typesupport_target}")

# ─── Install ────────────────────────────────────────────────────
install(TARGETS herkulex_serial
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS herkulex_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ─── Testing ────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ─── Export ─────────────────────────────────────────────────────
ament_export_include_directories(include)
ament_export_libraries(herkulex_serial)
ament_export_dependencies(rclcpp std_msgs)

ament_package()

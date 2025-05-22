cmake_minimum_required(VERSION 3.8)
project(api_handler)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED) # For Bool, Int8 messages

find_package(rosidl_default_generators REQUIRED)

# Define custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ScanBarcode.srv"
  "srv/GetLastBarcode.srv"
  "srv/ToggleDoorState.srv"
  "srv/ToggleEmergencyButtonState.srv"
  "srv/SetStackLightState.srv" # This is the new service
)

# Export dependencies for other packages that might use your interfaces
# This is how Python-only packages expose their dependencies and generated interfaces
ament_export_dependencies(
  rclpy
  std_msgs
  rosidl_default_runtime
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
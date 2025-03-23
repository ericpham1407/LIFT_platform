cmake_minimum_required(VERSION 3.8)
project(luci_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UTMOrigin.msg"
  "msg/GPSFix.msg"
  "msg/Position.msg"
  "msg/OrientationQuaternion.msg"
  "msg/BodyLinearVelocity.msg"
  "msg/BodyAngularVelocity.msg"
  "msg/PoseVel.msg"
  "msg/MotorCommand.msg"
  "msg/MotorReadout.msg"
  "msg/MotorStatus.msg"
  "msg/VehicleActuation.msg"
  "msg/VehicleState.msg"
  )

ament_package()
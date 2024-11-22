cmake_minimum_required(VERSION 3.8)
project(kuka_rsi_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
    "msg/ProgramState.msg"
    "msg/RobotState.msg"
)

#
# Testing
#
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()

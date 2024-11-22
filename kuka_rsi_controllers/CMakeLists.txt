cmake_minimum_required(VERSION 3.8)
project(kuka_rsi_controllers)

find_package(ament_cmake REQUIRED)

#
# Find dependencies
#
set(ROS_DEPENDENCIES
  controller_interface
  kuka_rsi_interfaces
  lifecycle_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)
foreach(DEP ${ROS_DEPENDENCIES})
  find_package(${DEP} REQUIRED)
endforeach()

#
# Create controller library
#
add_library(kuka_rsi_controllers
  SHARED
)
target_sources(kuka_rsi_controllers
  PRIVATE
    src/status_broadcaster.cpp
)
target_compile_features(kuka_rsi_controllers
  PUBLIC
    c_std_11
    cxx_std_20
)
target_include_directories(kuka_rsi_controllers
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(kuka_rsi_controllers
  ${ROS_DEPENDENCIES}
)

#
# Install target
#
install(
  DIRECTORY include/
  DESTINATION include
)
pluginlib_export_plugin_description_file(
  controller_interface
  kuka_rsi_controllers.xml
)
install(
  TARGETS
    kuka_rsi_controllers
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#
# Testing
#
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#
# Create ament package
#
ament_export_dependencies(
  ${ROS_DEPENDENCIES}
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  kuka_rsi_controllers
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()

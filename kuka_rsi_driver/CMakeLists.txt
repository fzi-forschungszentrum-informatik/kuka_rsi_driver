cmake_minimum_required(VERSION 3.8)
project(kuka_rsi_driver)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

option(TRACING_ENABLED "Explicitly disable support for tracing" OFF)
if(TRACING_ENABLED)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LTTNG REQUIRED lttng-ust)

  message(STATUS "Tracing enabled")
else()
  message(STATUS "Tracing disabled")
endif()

#
# Create config file to signal if tracing is enabled
#
configure_file(
  include/${PROJECT_NAME}/config.h.in
  include/${PROJECT_NAME}/config.h
)

#
# Find dependencies
#
set(ROS_DEPENDENCIES
  EXPAT
  fmt
  hardware_interface
  lifecycle_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
foreach(DEP ${ROS_DEPENDENCIES})
  find_package(${DEP} REQUIRED)
endforeach()
find_package(Boost CONFIG REQUIRED)
find_package(orocos_kdl REQUIRED)

#
# Create hardware interface
#
add_library(kuka_rsi_driver
  SHARED
)
target_sources(kuka_rsi_driver
  PRIVATE
    src/control_buffer.cpp
    src/control_thread.cpp
    src/hardware_interface.cpp
    src/phase_estimator.cpp
    src/rsi_factory.cpp
    src/rsi_parser.cpp
    src/rsi.cpp
    src/rsi_writer.cpp
    src/udp_server.cpp
)
if(TRACING_ENABLED)
  target_sources(kuka_rsi_driver
    PRIVATE
      src/tracing.cpp
      src/tracepoints.cpp
  )
endif()
target_compile_features(kuka_rsi_driver
  PUBLIC
    c_std_11
    cxx_std_20
)
target_include_directories(kuka_rsi_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(kuka_rsi_driver
  fmt::fmt
  Boost::boost
  orocos-kdl
  atomic
)
if(TRACING_ENABLED)
  target_link_libraries(kuka_rsi_driver
    ${LTTNG_LIBRARIES}
  )
endif()
ament_target_dependencies(kuka_rsi_driver
  ${ROS_DEPENDENCIES}
)

pluginlib_export_plugin_description_file(
  hardware_interface
  hardware_interface_plugin.xml
)

#
# Install target
#
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY
    config
    urdf
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY
    krl/
  DESTINATION share/${PROJECT_NAME}/krl
)
install(
  TARGETS
    kuka_rsi_driver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_python_install_package(
  ${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

#
# Testing
#
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tf2 REQUIRED)

  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_add_gtest(kuka_rsi_driver_test
    test/control_buffer.cpp
    test/rsi_parser.cpp
    test/rsi_writer.cpp
  )
  target_link_libraries(kuka_rsi_driver_test
    kuka_rsi_driver
    tf2::tf2
  )

  set(AMENT_LINT_AUTO_FILE_EXCLUDE doc/conf.py)

  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  ${ROS_DEPENDENCIES}
  Boost
)
ament_export_include_directories(
  include
)
ament_export_libraries(
  kuka_rsi_driver
  ${LTTNG_LIBRARIES}
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(clearpath_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED) 

set(TEKNIC_SDK_INCLUDE_DIR
  /home/naodai/Workspace/rotary/docking/Linux_Software/inc/inc-pub
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ClearpathState.msg"
  "srv/AxisEnable.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(
  cpp_typesupport_target           
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)


add_library(clearpath_robot_hw SHARED
  lib/clearpath_robot_hw.cpp
  lib/clearpath_motor.cpp
  lib/manager.cpp
  lib/clearpath_motor_hw.cpp
  lib/motor_hw_base.cpp
)


add_executable(clearpath_robot_node 
              src/clearpath_robot_node.cpp)

ament_target_dependencies(clearpath_robot_hw
  rclcpp 
  controller_manager
  hardware_interface
  diagnostic_updater
  rclcpp_lifecycle
  pluginlib
)

target_include_directories(clearpath_robot_hw
  PUBLIC
    include
    ${TEKNIC_SDK_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

target_include_directories(clearpath_robot_node
  PUBLIC
    include
    ${TEKNIC_SDK_INCLUDE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)




target_link_libraries(clearpath_robot_hw
  # PRIVATE
  ${cpp_typesupport_target}
    /usr/local/lib/libsFoundation20.so.1
)


ament_target_dependencies(clearpath_robot_node
  rclcpp
)


rosidl_target_interfaces(clearpath_robot_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

pluginlib_export_plugin_description_file(hardware_interface hardware_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

# install(TARGETS clearpath_robot_node
#   DESTINATION lib/${PROJECT_NAME}
# )

install(
  TARGETS clearpath_robot_hw
  EXPORT export_clearpath_robot_hw
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  FILES hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(clearpath_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(TEKNIC_SDK_INCLUDE_DIR
  /home/naodai/Workspace/rotary/docking/Linux_Software/inc/inc-pub
)

add_executable(clearpath_node 
              src/clearpath_node.cpp
              lib/clearpath_motor.cpp)


ament_target_dependencies(clearpath_node
  rclcpp 
)

target_include_directories(clearpath_node
  PUBLIC
    include
    ${TEKNIC_SDK_INCLUDE_DIR}  
)

target_link_libraries(clearpath_node
  # PRIVATE
    /usr/local/lib/libsFoundation20.so.1
)

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

install(TARGETS clearpath_node
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

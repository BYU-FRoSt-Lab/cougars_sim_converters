cmake_minimum_required(VERSION 3.8)
project(keyboard_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(std_msgs REQUIRED)
find_package(holoocean_interfaces REQUIRED)


# add_executable(coug_kinematics tutorials/coug_kinematics.cpp)
# ament_target_dependencies(couguv_teleop_key rclcpp holoocean_interfaces)

add_executable(couguv_teleop_key tutorials/teleop_couguv_key.cpp)
ament_target_dependencies(couguv_teleop_key rclcpp holoocean_interfaces)

install(TARGETS couguv_teleop_key 
  DESTINATION lib/${PROJECT_NAME})


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

ament_export_include_directories(include)
ament_package()

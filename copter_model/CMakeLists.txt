cmake_minimum_required(VERSION 3.8)
project(copter_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)
include_directories(
  ${GAZEBO_MODELS_DIRS}
)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

set(dependencies
  "geometry_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "tf2_geometry_msgs"
)

add_executable(copter_plugin src/copter_plugin.cpp)
ament_target_dependencies(copter_plugin rclcpp geometry_msgs tf2 tf2_geometry_msgs)

install(DIRECTORY launch include src models worlds scripts config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  scripts/laser_dist_plugin.py
  scripts/ins_nav_plugin.py
  scripts/aion_plugin.py
  scripts/gps_plugin.py
  scripts/ins_imu_plugin.py
  scripts/spawn_copter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  copter_plugin
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_package()

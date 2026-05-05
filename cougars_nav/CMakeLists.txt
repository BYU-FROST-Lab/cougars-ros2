cmake_minimum_required(VERSION 3.8)
project(cougars_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cougars_interfaces REQUIRED)
find_package(seatrac_interfaces REQUIRED)
find_package(dvl_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) # Add this to be safe, as you use it directly

# Waypoints dependencies
# Find yaml-cpp package
find_package(yaml-cpp REQUIRED)


include_directories(include/${PROJECT_NAME})

# Add executable for the waypoint follower node
# add_executable(waypoint_follower src/waypoint_follower_old.cpp)
# ament_target_dependencies(waypoint_follower
#   rclcpp
#   nav_msgs
#   sensor_msgs
#   cougars_interfaces
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
#   yaml-cpp  # Add yaml-cpp dependency
# )

add_executable(waypoint_iterator src/waypoint_iterator.cpp)
ament_target_dependencies(waypoint_iterator
  rclcpp
  geographic_msgs
  cougars_interfaces
)

add_executable(waypoint_controller src/waypoint_controller.cpp)
ament_target_dependencies(waypoint_controller
  rclcpp
  geographic_msgs
  cougars_interfaces
  geometry_msgs
  tf2_ros
)


# Install executables
install(TARGETS
  waypoint_iterator
  waypoint_controller
  # waypoint_follower #added for waypoint follower
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# Install mission configuration files
install(DIRECTORY
  waypoint_missions/ # Assuming your YAML files are in a 'waypoint_missions' subdir of cougars_control
  DESTINATION share/${PROJECT_NAME}/waypoint_missions
)

# Install Python executables
install(PROGRAMS
  scripts/manual_mission.py
  scripts/setpoint_transformer.py
  # scripts/dummy_factor_starter.py
  scripts/fins_manual.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install mission configuration files
install(DIRECTORY
  waypoint_missions/ # Assuming your YAML files are in a 'waypoint_missions' subdir of cougars_control
  DESTINATION share/${PROJECT_NAME}/waypoint_missions
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

#Install Launch file
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()

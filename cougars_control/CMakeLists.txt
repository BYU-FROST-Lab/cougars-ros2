cmake_minimum_required(VERSION 3.8)
project(cougars_control)

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
find_package(nav_msgs REQUIRED)
find_package(cougars_interfaces REQUIRED)
find_package(seatrac_interfaces REQUIRED)
find_package(dvl_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
# find_package(MOOS 10 REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(robot_localization REQUIRED) #add if using the EKF
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) # Add this to be safe, as you use it directly

# Waypoints dependencies
# Find yaml-cpp package
find_package(yaml-cpp REQUIRED)


include_directories(include/${PROJECT_NAME})

add_executable(coug_controls src/coug_controls.cpp src/pid.cpp)
ament_target_dependencies(coug_controls rclcpp std_msgs cougars_interfaces sensor_msgs geometry_msgs std_srvs dvl_msgs)

add_executable(coug_kinematics src/coug_kinematics.cpp)
ament_target_dependencies(coug_kinematics rclcpp std_msgs cougars_interfaces std_srvs)

# add_executable(moos_bridge src/moos_bridge.cpp)
# ament_target_dependencies(moos_bridge rclcpp std_msgs cougars_interfaces MOOS nav_msgs seatrac_interfaces)

# add_executable(moos_bridge_sim src/moos_bridge_sim.cpp)
# ament_target_dependencies(moos_bridge_sim rclcpp std_msgs cougars_interfaces MOOS nav_msgs seatrac_interfaces geometry_msgs)

# add_executable(moos_bridge_gps src/moos_bridge_gps.cpp)
# ament_target_dependencies(moos_bridge_gps rclcpp std_msgs cougars_interfaces MOOS nav_msgs seatrac_interfaces geometry_msgs)


add_executable(emergency_protocols src/emergency_protocols.cpp)
ament_target_dependencies(emergency_protocols rclcpp std_msgs gps_msgs dvl_msgs cougars_interfaces nav_msgs seatrac_interfaces geometry_msgs std_srvs sensor_msgs)

# Add executable for the waypoint follower node
add_executable(waypoint_follower src/waypoint_follower.cpp)
ament_target_dependencies(waypoint_follower
  rclcpp
  nav_msgs
  sensor_msgs
  cougars_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  yaml-cpp  # Add yaml-cpp dependency
)
add_executable(waypoint_follower_gps src/waypoint_follower_gps.cpp) 
ament_target_dependencies(waypoint_follower_gps
  rclcpp
  nav_msgs
  sensor_msgs
  cougars_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  yaml-cpp  # Add yaml-cpp dependency
)

add_executable(mc_serial_node src/mc_serial_node.cpp)
ament_target_dependencies(mc_serial_node rclcpp std_srvs std_msgs cougars_interfaces sensor_msgs)

# Link libraries
target_link_libraries(mc_serial_node
  serialport
)
target_link_libraries(waypoint_follower yaml-cpp)
target_link_libraries(waypoint_follower_gps yaml-cpp)
# Link libraries
target_link_libraries(coug_controls)
target_link_libraries(coug_kinematics)
# target_link_libraries(moos_bridge)
# target_link_libraries(moos_bridge_sim)
# target_link_libraries(moos_bridge_gps)
target_link_libraries(emergency_protocols)

# Install executables
install(TARGETS
  coug_controls
  coug_kinematics
  mc_serial_node
  emergency_protocols
  waypoint_follower #added for waypoint follower
  waypoint_follower_gps #added for waypoint follower
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/manual_mission.py
  scripts/dummy_factor_starter.py
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

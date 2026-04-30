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

include_directories(include/${PROJECT_NAME})

add_executable(coug_controls src/coug_controls.cpp src/pid.cpp)
ament_target_dependencies(coug_controls rclcpp std_msgs cougars_interfaces sensor_msgs geometry_msgs std_srvs dvl_msgs)

add_executable(coug_kinematics src/coug_kinematics.cpp)
ament_target_dependencies(coug_kinematics rclcpp std_msgs cougars_interfaces std_srvs)

add_executable(emergency_protocols src/emergency_protocols.cpp)
ament_target_dependencies(emergency_protocols rclcpp std_msgs gps_msgs dvl_msgs cougars_interfaces nav_msgs seatrac_interfaces geometry_msgs std_srvs sensor_msgs)

add_executable(mc_serial_node src/mc_serial_node.cpp)
ament_target_dependencies(mc_serial_node rclcpp std_msgs cougars_interfaces sensor_msgs)

# Link libraries
target_link_libraries(mc_serial_node
  serialport
)
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
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/dummy_factor_starter.py
  DESTINATION lib/${PROJECT_NAME}
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

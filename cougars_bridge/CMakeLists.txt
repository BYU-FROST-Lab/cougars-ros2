cmake_minimum_required(VERSION 3.8)
project(cougars_bridge)

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
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include/${PROJECT_NAME})

add_executable(depth_converter src/depth_converter.cpp)
ament_target_dependencies(depth_converter rclcpp std_msgs std_srvs cougars_interfaces geometry_msgs sensor_msgs)

add_executable(dvl_converter src/dvl_converter.cpp)
ament_target_dependencies(dvl_converter rclcpp std_msgs cougars_interfaces geometry_msgs dvl_msgs)

add_executable(seatrac_imu_converter src/seatrac_imu_converter.cpp)
ament_target_dependencies(
  seatrac_imu_converter 
  rclcpp 
  std_msgs 
  seatrac_interfaces
  geometry_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)

add_executable(dvl_global src/dvl_global.cpp)
ament_target_dependencies(dvl_global rclcpp std_msgs geometry_msgs tf2_ros tf2_geometry_msgs)

add_executable(static_tf_publisher src/static_tf_publisher.cpp)
ament_target_dependencies(static_tf_publisher rclcpp std_msgs geometry_msgs tf2_ros tf2_geometry_msgs)

# Link libraries
target_link_libraries(depth_converter)
target_link_libraries(dvl_converter)
target_link_libraries(seatrac_imu_converter)
target_link_libraries(dvl_global)

# Install executables
install(TARGETS
  depth_converter
  dvl_converter
  seatrac_imu_converter
  dvl_global
  static_tf_publisher
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/gps_odom.py
  scripts/dvl_manager.py
  scripts/nmea_constructor.py
  scripts/mavlink_bridge.py
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

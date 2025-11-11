cmake_minimum_required(VERSION 3.8)
project(cougars_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dvl_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cougars_interfaces REQUIRED)
find_package(seatrac_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(bag_recorder src/bag_recorder.cpp)
ament_target_dependencies(bag_recorder rclcpp std_msgs cougars_interfaces sensor_msgs dvl_msgs seatrac_interfaces gps_msgs nav_msgs rosbag2_cpp diagnostic_msgs)

target_link_libraries(bag_recorder)

# Install executables
install(TARGETS
  bag_recorder
  DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/startup_call.py
  scripts/param_reloader.py
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

cmake_minimum_required(VERSION 3.8)
project(cougars_coms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(seatrac_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cougars_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dvl_msgs REQUIRED)

find_package(seatrac_driver QUIET)
if(NOT TARGET seatrac_driver)
    include(FetchContent)
    FetchContent_Declare(seatrac_driver
        GIT_REPOSITORY https://github.com/BenWashburn1457/seatrac_driver.git
        GIT_TAG        main
    )
    FetchContent_MakeAvailable(seatrac_driver)
endif()

include_directories(include)

add_executable(cougars_coms src/cougars_coms.cpp)
ament_target_dependencies(cougars_coms rclcpp std_srvs seatrac_interfaces geometry_msgs cougars_interfaces nav_msgs sensor_msgs dvl_msgs)
target_link_libraries(cougars_coms seatrac_driver)

add_executable(vehicle_pinger src/vehicle_pinger.cpp)
ament_target_dependencies(vehicle_pinger rclcpp seatrac_interfaces)
target_link_libraries(vehicle_pinger seatrac_driver)

install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME})

install(TARGETS
  cougars_coms
  vehicle_pinger
  DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(
  PROGRAMS
    rf_bridge/rf_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

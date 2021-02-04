cmake_minimum_required(VERSION 3.5)
project(control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vehicle_interface REQUIRED)
find_package(geometry_msgs REQUIRED)


add_library(pid SHARED
  src/PID.cpp
)
target_include_directories(pid PUBLIC include)
ament_target_dependencies(pid
  "rclcpp"
)

add_executable(wheel_controller_node src/wheel_controller_node.cpp)
target_include_directories(wheel_controller_node PUBLIC include)
target_link_libraries(wheel_controller_node pid)
ament_target_dependencies(wheel_controller_node rclcpp vehicle_interface)

add_executable(vehicle_controller_node src/vehicle_controller_node.cpp)
target_include_directories(vehicle_controller_node PUBLIC include)
ament_target_dependencies(vehicle_controller_node rclcpp geometry_msgs vehicle_interface)


install(TARGETS
  wheel_controller_node vehicle_controller_node pid
  DESTINATION lib/${PROJECT_NAME}
)


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

ament_package()

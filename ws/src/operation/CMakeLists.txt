cmake_minimum_required(VERSION 3.5)
project(operation)

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
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vehicle_interface REQUIRED)

add_library(manual_joy SHARED src/manual_operation_joy.cpp)
target_include_directories(manual_joy PUBLIC include)
ament_target_dependencies(manual_joy 
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "vehicle_interface"
)
rclcpp_components_register_nodes(manual_joy "ManualOperationJoyNode")

add_library(waypoint_publisher SHARED src/waypoint_publisher.cpp)
target_include_directories(waypoint_publisher PUBLIC include)
ament_target_dependencies(waypoint_publisher
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "visualization_msgs"
  "vehicle_interface"
)
rclcpp_components_register_nodes(waypoint_publisher "WaypointPublisherNode")

add_library(wheel_stepper SHARED src/wheel_stepper.cpp)
target_include_directories(wheel_stepper PUBLIC include)
ament_target_dependencies(wheel_stepper
  "rclcpp"
  "rclcpp_components"
  "vehicle_interface"
)
rclcpp_components_register_nodes(wheel_stepper "WheelStepper")



install(TARGETS
  manual_joy waypoint_publisher wheel_stepper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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

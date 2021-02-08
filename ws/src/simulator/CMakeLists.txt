cmake_minimum_required(VERSION 3.5)
project(simulator)

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
find_package(ament_cmake_python REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(vehicle_interface REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})


add_library(vehicle_body SHARED
  src/gazebo_plugins/vehicle_body.cpp
)
target_include_directories(vehicle_body PUBLIC include)
ament_target_dependencies(vehicle_body
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "rclcpp"
)
add_library(wheel SHARED
  src/gazebo_plugins/wheel.cpp
)
target_include_directories(wheel PUBLIC include)
ament_target_dependencies(wheel
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "vehicle_interface"
)

ament_export_libraries(vehicle_body wheel)
install(TARGETS
  vehicle_body wheel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


# install python modules
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  simulator/spawn_vehicle.py
  DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)



# Export package dependencies
ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclpy)

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

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
find_package(rclcpp_components REQUIRED)
find_package(vehicle_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ignition-math6 REQUIRED)

add_library(pid SHARED
  src/PID.cpp
)
target_include_directories(pid PUBLIC include)
ament_target_dependencies(pid
  "rclcpp"
)

add_library(pathlib SHARED
  "src/path/path_line.cpp"
  "src/path/path_spiral.cpp"
  "src/path/path_collection.cpp"
  "src/path/path.cpp"
)
target_include_directories(pathlib PUBLIC include)
ament_target_dependencies(pathlib
  "ignition-math6"
)

add_executable(pathlib_testing 
  "src/pathlib_testing.cpp"
)
target_include_directories(pathlib_testing PUBLIC include)
target_link_libraries(pathlib_testing pathlib)
ament_target_dependencies(pathlib_testing
  "ignition-math6"  
)


add_library(wheel_controller SHARED src/wheel_controller_node.cpp)
target_include_directories(wheel_controller PUBLIC include)
target_link_libraries(wheel_controller pid)
ament_target_dependencies(wheel_controller 
  "rclcpp"
  "rclcpp_components"
  "vehicle_interface"
)
rclcpp_components_register_nodes(wheel_controller "WheelControllerNode")

add_library(vehicle_controller SHARED src/vehicle_controller_node.cpp)
target_include_directories(vehicle_controller PUBLIC include ${Eigen3_INCLUDE_DIRS})
ament_target_dependencies(vehicle_controller 
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "vehicle_interface"
)
rclcpp_components_register_nodes(vehicle_controller "VehicleControllerNode")

install(TARGETS
  pid pathlib
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  wheel_controller vehicle_controller 
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

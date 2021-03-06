cmake_minimum_required(VERSION 3.5)
project(vehicle_interface)

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
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(geometry_msgs REQUIRED)

set(MSG_FILES
  "msg/Rotation.msg"
  "msg/Guide.msg"
  "msg/GuidanceInfo.msg"
  "msg/DriveMode.msg"
  "msg/VehicleControllerInfo.msg"
  "msg/WheelControllerInfo.msg"
  "msg/Waypoints.msg"
  "msg/WheelCommand.msg"
  "msg/WheelState.msg"
  "msg/WheelLoad.msg"
  "msg/WheelLoads.msg"
  "msg/YawReference.msg"
  "msg/YawrateReference.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES geometry_msgs
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

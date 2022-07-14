cmake_minimum_required(VERSION 3.5)
project(project_part2)
set(SUB_SCAN_PUB_CMD_NODE_NAME subs_scan_pub_cmd)
set(PRE_APPROACH_NODE_NAME pre_approach)
set(DETECT_SHELF_NODE_NAME detect_shelf)
set(ENTER_CART_NODE_NAME enter_cart)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
set(CMAKE_C_COMPILER "/usr/bin/clang")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(phase2_custom_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # remove the line when this package is a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

add_executable(${SUB_SCAN_PUB_CMD_NODE_NAME} src/${SUB_SCAN_PUB_CMD_NODE_NAME}.cpp)
add_executable(${PRE_APPROACH_NODE_NAME} src/${PRE_APPROACH_NODE_NAME}.cpp)
add_executable(${DETECT_SHELF_NODE_NAME} src/${DETECT_SHELF_NODE_NAME}.cpp)
add_executable(${ENTER_CART_NODE_NAME} src/${ENTER_CART_NODE_NAME}.cpp)

ament_target_dependencies(${SUB_SCAN_PUB_CMD_NODE_NAME}
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "std_msgs")
ament_target_dependencies(${PRE_APPROACH_NODE_NAME}
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs")
ament_target_dependencies(${DETECT_SHELF_NODE_NAME}
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2_ros")
ament_target_dependencies(${ENTER_CART_NODE_NAME}
  "rclcpp"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2_ros")

install(TARGETS
   ${SUB_SCAN_PUB_CMD_NODE_NAME}
   DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
   ${PRE_APPROACH_NODE_NAME}
   DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
   ${DETECT_SHELF_NODE_NAME}
   DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
   ${ENTER_CART_NODE_NAME}
   DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

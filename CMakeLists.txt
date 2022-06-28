cmake_minimum_required(VERSION 3.5)
project(project_part2 VERSION 0.1.0 LANGUAGES CXX)
set(NODE_NAME subs_scan_pub_cmd)
# set(PRE_APPROACH_NODE_NAME pre_approach)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
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
find_package(rclcpp_components REQUIRED)
find_package(composition REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

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

include_directories(include)

add_library(${NODE_NAME} SHARED src/subs_scan_pub_cmd.cpp)
# add_library(${PRE_APPROACH_NODE_NAME} SHARED src/pre_approach.cpp)
# add_executable(${PRE_APPROACH_NODE_NAME}
#   src/pre_approach_main.cpp
# )

target_compile_definitions(${NODE_NAME} PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(${NODE_NAME}
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs")
# ament_target_dependencies(${PRE_APPROACH_NODE_NAME}
#   "rclcpp"
#   "rclcpp_components"
#   "sensor_msgs"
#   "geometry_msgs"
#   "nav_msgs")

rclcpp_components_register_nodes(${NODE_NAME} "${PROJECT_NAME}::SubsScanPubCmd")
set(node_plugins "${node_plugins}${PROJECT_NAME}::SubsScanPubCmd;$<TARGET_FILE:${NODE_NAME}>\n")

install(TARGETS
  ${NODE_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
# install(TARGETS
#   ${PRE_APPROACH_NODE_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# install(
#   TARGETS ${PRE_APPROACH_NODE_NAME}
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()

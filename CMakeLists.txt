cmake_minimum_required(VERSION 3.8)
project(ssis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(smart_tether REQUIRED)
find_package(tether_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  ${smart_tether_INCLUDE_DIRS}
)

add_executable(uav_position_publisher src/uav_position_publisher.cpp)
ament_target_dependencies(
  uav_position_publisher
  rclcpp
  geometry_msgs
)

add_executable(uav1_frame_publisher src/uav1_frame_publisher.cpp)
ament_target_dependencies(uav1_frame_publisher
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)

add_executable(uav2_frame_publisher src/uav2_frame_publisher.cpp)
ament_target_dependencies(uav2_frame_publisher
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)

add_executable(uav3_frame_publisher src/uav3_frame_publisher.cpp)
ament_target_dependencies(uav3_frame_publisher
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)

add_executable(uav_frame_publisher_3tethers src/uav_frame_publisher_3tethers.cpp)
ament_target_dependencies(uav_frame_publisher_3tethers
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)


add_executable(min_tuav_tethers src/min_tuav_tethers.cpp)
ament_target_dependencies(min_tuav_tethers
  smart_tether
  geometry_msgs
  tether_msgs
  rclcpp
  tf2
  tf2_ros
)
target_link_libraries(min_tuav_tethers nlopt)
# target_include_directories(min_tuav_1tether
#   PUBLIC
#   ${smart_tether_INCLUDE_DIRS}
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

add_executable(visualization_node src/visualization_node)
ament_target_dependencies(visualization_node
  rclcpp
  geometry_msgs
  visualization_msgs
  tether_msgs
  tf2
)

add_executable(get_result_node src/get_result_node)
ament_target_dependencies(get_result_node
  rclcpp
  geometry_msgs
  tether_msgs
)

install(TARGETS
    uav_position_publisher
    uav1_frame_publisher
    uav2_frame_publisher
    uav3_frame_publisher
    uav_frame_publisher_3tethers
    min_tuav_tethers
    visualization_node
    get_result_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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


ament_package()

cmake_minimum_required(VERSION 3.8)
project(gazebo-ros2-test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(timestamp_aligner_node src/timestamp_aligner_node.cpp)
add_executable(tracked2vision src/tracked2vision.cpp)
add_executable(cartographer_laser_transfer src/laser_transfer.cpp)

ament_target_dependencies(timestamp_aligner_node rclcpp message_filters sensor_msgs geometry_msgs)
ament_target_dependencies(tracked2vision rclcpp geometry_msgs)
ament_target_dependencies(cartographer_laser_transfer rclcpp geometry_msgs tf2_ros sensor_msgs tf2_geometry_msgs)
# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  timestamp_aligner_node
  tracked2vision
  cartographer_laser_transfer  
  DESTINATION lib/${PROJECT_NAME})
  

  
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

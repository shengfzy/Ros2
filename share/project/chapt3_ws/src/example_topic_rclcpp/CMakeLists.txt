cmake_minimum_required(VERSION 3.8)
project(example_topic_rclcpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)

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

add_executable(topic_publisher_01 src/topic_publisher_01.cpp)
ament_target_dependencies(topic_publisher_01 rclcpp std_msgs)

install(
    TARGETS topic_publisher_01
    DESTINATION lib/${PROJECT_NAME}
)

add_executable(topic_subscriber_01 src/topic_subscriber_01.cpp)
ament_target_dependencies(topic_subscriber_01 rclcpp std_msgs)

install(
    TARGETS topic_subscriber_01
    DESTINATION lib/${PROJECT_NAME}
)

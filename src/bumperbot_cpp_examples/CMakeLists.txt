cmake_minimum_required(VERSION 3.8)
project(bumperbot_cpp_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(simple_publisher_cpp src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher_cpp rclcpp std_msgs)
add_executable(twist_publisher_cpp src/twist_publisher.cpp)
ament_target_dependencies(twist_publisher_cpp rclcpp geometry_msgs)
add_executable(simple_subscriber_cpp src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber_cpp rclcpp std_msgs)

install(TARGETS
  simple_publisher_cpp
  simple_subscriber_cpp
  twist_publisher_cpp
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

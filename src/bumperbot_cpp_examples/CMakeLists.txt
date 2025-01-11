cmake_minimum_required(VERSION 3.8)
project(bumperbot_cpp_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(turtlesim REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(simple_publisher_cpp src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher_cpp rclcpp std_msgs)
add_executable(twist_publisher_cpp src/twist_publisher.cpp)
ament_target_dependencies(twist_publisher_cpp rclcpp geometry_msgs)
add_executable(simple_subscriber_cpp src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber_cpp rclcpp std_msgs)
add_executable(twist_pubsub_cpp src/twist_pub_sub.cpp)
ament_target_dependencies(twist_pubsub_cpp rclcpp geometry_msgs)
add_executable(turtle_ref_pose src/turtle_ref_pose.cpp)
ament_target_dependencies(turtle_ref_pose rclcpp turtlesim)
add_executable(simple_tf_kinematics src/simple_tf_kinematics.cpp)
ament_target_dependencies(simple_tf_kinematics rclcpp geometry_msgs tf2_ros)


install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  simple_publisher_cpp
  simple_subscriber_cpp
  twist_publisher_cpp
  twist_pubsub_cpp
  turtle_ref_pose
  simple_tf_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(my_robot_topic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(my_odom src/my_odom.cpp)
add_executable(odom_tf src/odom_tf.cpp)

ament_target_dependencies(my_odom
rclcpp
geometry_msgs
nav_msgs)

ament_target_dependencies(odom_tf
rclcpp
tf2
tf2_ros
geometry_msgs
nav_msgs)

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

install(TARGETS my_odom
DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS odom_tf
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

install(DIRECTORY urdf
DESTINATION share/${PROJECT_NAME})
ament_package()

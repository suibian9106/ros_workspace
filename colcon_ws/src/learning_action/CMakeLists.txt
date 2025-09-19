cmake_minimum_required(VERSION 3.5)
project(learning_action)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 添加依赖
find_package(rclcpp REQUIRED)
find_package(learning_action_interface REQUIRED)
find_package(rclcpp_action REQUIRED)

# 添加可执行文件
add_executable(fibonacci_action_server src/fibonacci_action_server.cpp)
ament_target_dependencies(fibonacci_action_server rclcpp learning_action_interface rclcpp_action)

add_executable(fibonacci_action_client src/fibonacci_action_client.cpp)
ament_target_dependencies(fibonacci_action_client rclcpp learning_action_interface rclcpp_action)

# 添加 install 部分，以便 ros2 run 可以找到你的可执行文件
install(TARGETS
  fibonacci_action_server
  fibonacci_action_client
  DESTINATION lib/${PROJECT_NAME})

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

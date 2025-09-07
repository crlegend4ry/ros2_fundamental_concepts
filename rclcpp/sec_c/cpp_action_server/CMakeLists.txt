cmake_minimum_required(VERSION 3.8)
project(cpp_action_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_interfaces REQUIRED)
find_package(tf2 REQUIRED)

add_executable(absolute_rotate_action_server absolute_rotate_action_server.cpp)
ament_target_dependencies(absolute_rotate_action_server rclcpp rclcpp_action geometry_msgs nav_msgs action_interfaces tf2)

add_executable(accumulated_rotate_action_server accumulated_rotate_action_server.cpp)
ament_target_dependencies(accumulated_rotate_action_server rclcpp rclcpp_action geometry_msgs nav_msgs action_interfaces tf2)

target_include_directories(absolute_rotate_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(accumulated_rotate_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(absolute_rotate_action_server PUBLIC c_std_99 cxx_std_17)
target_compile_features(accumulated_rotate_action_server PUBLIC c_std_99 cxx_std_17)  

install(TARGETS absolute_rotate_action_server
                accumulated_rotate_action_server
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
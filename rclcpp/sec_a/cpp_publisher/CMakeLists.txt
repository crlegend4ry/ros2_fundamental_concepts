cmake_minimum_required(VERSION 3.8)
project(cpp_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(circle_publisher_with_lin_vel circle_publisher_with_lin_vel.cpp)
ament_target_dependencies(circle_publisher_with_lin_vel rclcpp geometry_msgs)

add_executable(circle_publisher_with_lin_ang_vel circle_publisher_with_lin_ang_vel.cpp)
ament_target_dependencies(circle_publisher_with_lin_ang_vel rclcpp geometry_msgs)

target_include_directories(circle_publisher_with_lin_vel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(circle_publisher_with_lin_ang_vel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(circle_publisher_with_lin_vel PUBLIC c_std_99 cxx_std_17)
target_compile_features(circle_publisher_with_lin_ang_vel PUBLIC c_std_99 cxx_std_17)

install(TARGETS circle_publisher_with_lin_vel
                circle_publisher_with_lin_ang_vel
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
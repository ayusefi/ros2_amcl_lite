cmake_minimum_required(VERSION 3.8)
project(ros2_amcl_lite)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

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



add_library(particle_filter src/particle_filter.cpp)
add_library(likelihood_field src/likelihood_field.cpp)
add_library(dynamic_object_detector src/dynamic_object_detector.cpp)
add_library(dynamic_aware_sensor_model src/dynamic_aware_sensor_model.cpp)

target_include_directories(particle_filter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(likelihood_field PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(dynamic_object_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(dynamic_aware_sensor_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(likelihood_field nav_msgs)
ament_target_dependencies(dynamic_object_detector sensor_msgs nav_msgs geometry_msgs)
ament_target_dependencies(dynamic_aware_sensor_model sensor_msgs nav_msgs geometry_msgs)

add_executable(amcl_lite_node src/amcl_lite_node.cpp)
ament_target_dependencies(amcl_lite_node rclcpp sensor_msgs nav_msgs geometry_msgs)
target_link_libraries(amcl_lite_node particle_filter likelihood_field dynamic_object_detector dynamic_aware_sensor_model)

install(TARGETS amcl_lite_node particle_filter likelihood_field dynamic_object_detector dynamic_aware_sensor_model
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()

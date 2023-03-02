cmake_minimum_required(VERSION 3.8)
project(mock_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(test_gazebo_plugin SHARED src/test_gazebo_plugin.cpp)

target_include_directories(test_gazebo_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(test_gazebo_plugin
  "gazebo_ros"
  "gazebo_dev"
  "rclcpp"
  "sensor_msgs"
  "std_srvs"
  "std_msgs"
)
ament_export_libraries(test_gazebo_plugin)

install(TARGETS
  test_gazebo_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

  install(DIRECTORY worlds urdf launch meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

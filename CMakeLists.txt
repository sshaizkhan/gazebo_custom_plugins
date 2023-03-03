cmake_minimum_required(VERSION 3.8)

file(READ ${CMAKE_CURRENT_SOURCE_DIR}/package.xml PACKAGE_XML_CONTENTS)
string(REGEX MATCH "<version>[0-9]+\\.[0-9]+\\.[0-9]+</version>" PACKAGE_VERSION_REGEX "${PACKAGE_XML_CONTENTS}")
string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" PACKAGE_VERSION "${PACKAGE_VERSION_REGEX}")

message(STATUS "Package version: ${PACKAGE_VERSION}")

# Set the version in the project
set(PROJECT_VERSION ${PACKAGE_VERSION})
set(PROJECT_VERSION_MAJOR ${PACKAGE_VERSION_MAJOR})
set(PROJECT_VERSION_MINOR ${PACKAGE_VERSION_MINOR})
set(PROJECT_VERSION_PATCH ${PACKAGE_VERSION_PATCH})

project(gazebo_custom_plugins VERSION ${PACKAGE_VERSION})

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Adding Windows-OS support
if(WIN32)
  add_compile_definitions(

    # For math constants
    _USE_MATH_DEFINES

    # Minimize Windows namespace collision
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
link_directories(${gazebo_dev_LIBRARY_DIRS})

# Ignition Profiler Build
option(ENABLE_PROFILER "Enable Ignition Profiler" FALSE)

if(ENABLE_PROFILER)
  find_package(ignition-common3 QUIET COMPONENTS profiler)

  if(ignition-common3_FOUND)
    add_definitions("-DIGN_PROFILER_ENABLE=1" "-DIGN_PROFILER_REMOTERY=1")
    message(STATUS "Profiler is active")
  else()
    message(WARNING "Can't find Ignition common3. Profiler will not be actived")
    add_definitions("-DIGN_PROFILER_ENABLE=0" "-DIGN_PROFILER_REMOTERY=0")
  endif()
endif()

add_library(conveyor_belt_plugin SHARED src/conveyor_belt_plugin.cpp)

target_include_directories(conveyor_belt_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(conveyor_belt_plugin
  "gazebo_ros"
  "gazebo_dev"
  "rclcpp"
  "sensor_msgs"
  "std_srvs"
  "std_msgs"
)
ament_export_libraries(conveyor_belt_plugin)

install(DIRECTORY include/
        DESTINATION include)

install(TARGETS
  conveyor_belt_plugin
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

if(NOT WIN32)
  if(NOT APPLE)
    set(
      AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_gazebo_plugins
      "prepend-non-duplicate;LD_LIBRARY_PATH;${GAZEBO_PLUGIN_PATH}")
  else()
    set(
      AMENT_CMAKE_ENVIRONMENT_HOOKS_DESC_gazebo_plugins
      "prepend-non-duplicate;DYLD_LIBRARY_PATH;${GAZEBO_PLUGIN_PATH}")
  endif()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")


ament_package()

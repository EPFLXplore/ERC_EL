cmake_minimum_required(VERSION 3.5)
project(roco-ros-interface)

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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(BRoCo REQUIRED)
find_package(avionics_interfaces REQUIRED)

include_directories(
  ${BRoCo_INCLUDE_DIRS} 
  ${CMAKE_CURRENT_SOURCE_DIR}/../BRoCo/include
  include/roco-ros-interface
  include/utils
)

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

add_executable(roco_interface
  src/roco_interface.cpp
  src/BRoCoManager.cpp
  src/BRoCoPublisher.cpp
  src/BRoCoSubscriber.cpp

  src/Utils.cpp
)

ament_target_dependencies(roco_interface rclcpp sensor_msgs BRoCo avionics_interfaces)

install(TARGETS
  roco_interface
  DESTINATION lib/${PROJECT_NAME})

ament_package()

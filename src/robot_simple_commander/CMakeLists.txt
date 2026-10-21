cmake_minimum_required(VERSION 3.8)
project(robot_simple_commander)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# --- RViz Follow panel (Start / Stop) ---
add_library(robot_simple_commander_follow_panel SHARED
  src/follow_panel.cpp
  src/speed_panel.cpp
  src/map_save_panel.cpp
  include/robot_simple_commander/follow_panel.hpp
  include/robot_simple_commander/speed_panel.hpp
  include/robot_simple_commander/map_save_panel.hpp
)
ament_target_dependencies(robot_simple_commander_follow_panel
  pluginlib
  rclcpp
  rviz_common
  std_msgs
  std_srvs
)
target_include_directories(robot_simple_commander_follow_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${Qt5Widgets_INCLUDE_DIRS}
)
target_link_libraries(robot_simple_commander_follow_panel
  Qt5::Widgets
  rviz_common::rviz_common
)
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(TARGETS robot_simple_commander_follow_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES plugins_description.xml
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)

# --- Python package + ros2 run entries ---
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  bin/record_path
  bin/follow_path
  bin/cancel_navigation
  bin/safety_gate
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  maps
  paths
  recorded
  rviz
  scripts
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_path_io test/test_path_io.py)
endif()

ament_package()

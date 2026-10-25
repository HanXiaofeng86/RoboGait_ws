cmake_minimum_required(VERSION 3.8)
project(robogait_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

if(DEFINED ENV{ORBBEC_K4A_ROOT})
  set(ORBBEC_K4A_ROOT "$ENV{ORBBEC_K4A_ROOT}")
else()
  set(ORBBEC_K4A_ROOT "$ENV{HOME}/orbbec_wrapper_v2")
endif()
find_path(K4A_INCLUDE_DIR k4a/k4a.h PATHS ${ORBBEC_K4A_ROOT}/include NO_DEFAULT_PATH)
find_library(K4A_LIB k4a PATHS ${ORBBEC_K4A_ROOT}/lib NO_DEFAULT_PATH)
find_library(K4ABT_LIB k4abt PATHS ${ORBBEC_K4A_ROOT}/lib)

if(K4A_INCLUDE_DIR AND K4A_LIB AND K4ABT_LIB)
  add_executable(person_sdk src/person_sdk.cpp)
  target_include_directories(person_sdk BEFORE PRIVATE ${K4A_INCLUDE_DIR})
  target_link_libraries(person_sdk ${K4A_LIB} ${K4ABT_LIB})
  ament_target_dependencies(
    person_sdk
    rclcpp
    sensor_msgs
    geometry_msgs
    std_msgs
    visualization_msgs
    tf2
    tf2_ros
  )
  set_target_properties(person_sdk PROPERTIES
    BUILD_RPATH "${ORBBEC_K4A_ROOT}/lib"
    INSTALL_RPATH "${ORBBEC_K4A_ROOT}/lib"
  )
  install(TARGETS person_sdk DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING
    "Orbbec K4A wrapper not found under ${ORBBEC_K4A_ROOT}. "
    "Skipping person_sdk. Python perception nodes still install. "
    "Set ORBBEC_K4A_ROOT if the SDK lives somewhere else.")
endif()

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  ${PROJECT_NAME}/nodes/distance_orbbec.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME distance_orbbec
)

install(DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

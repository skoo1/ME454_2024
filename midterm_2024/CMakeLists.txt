cmake_minimum_required(VERSION 3.5)
project(midterm_2024)

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
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(midterm_2024_msg REQUIRED)

include_directories(include)
find_library(MYMAT_LIB NAMES mymat PATHS "${CMAKE_CURRENT_SOURCE_DIR}/lib")
message(STATUS "Found mymat library: ${MYMAT_LIB}")

macro(macro1 arg)
  add_executable(sun_torque_generator_${arg} src/sun_torque_generator_${arg}.cpp)
  target_link_directories(sun_torque_generator_${arg} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib)
  target_link_libraries(sun_torque_generator_${arg} myQuaternion_${arg} mymat pthread)
  set_target_properties(sun_torque_generator_${arg} PROPERTIES INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib")
  target_include_directories(sun_torque_generator_${arg} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(sun_torque_generator_${arg} rclcpp std_msgs gazebo_msgs geometry_msgs midterm_2024_msg)

  add_executable(recorder_${arg} src/recorder_${arg}.cpp)
  target_link_directories(recorder_${arg} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib)
  target_link_libraries(recorder_${arg} myQuaternion_${arg} mymat pthread)
  set_target_properties(recorder_${arg} PROPERTIES INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib")
  target_include_directories(recorder_${arg} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(recorder_${arg} rclcpp std_msgs geometry_msgs midterm_2024_msg)

  install(TARGETS
          sun_torque_generator_${arg}
          recorder_${arg}
          DESTINATION lib/${PROJECT_NAME})
endmacro()

macro1(20230001)

install(DIRECTORY
  launch
  rsc
  worlds
  include
  lib
  models
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.16)
project(dyx3_motion_guard)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dyx3_interfaces REQUIRED)

# Pure C++ core: no ROS includes. The whole safety argument rests on these files.
add_library(dyx3_motion_guard_core STATIC
  src/limits.cpp
  src/estop_gate.cpp
  src/rtk_gate.cpp
  src/mission_gate.cpp
  src/fail_to_zero.cpp)
target_include_directories(dyx3_motion_guard_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(dyx3_motion_guard_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(dyx3_motion_guard_core PRIVATE -Wall -Wextra -Wpedantic)

add_library(dyx3_motion_guard_node_lib STATIC src/motion_guard_node.cpp)
target_link_libraries(dyx3_motion_guard_node_lib PUBLIC dyx3_motion_guard_core)
ament_target_dependencies(dyx3_motion_guard_node_lib PUBLIC rclcpp dyx3_interfaces)
target_compile_options(dyx3_motion_guard_node_lib PRIVATE -Wall -Wextra -Wpedantic)

add_executable(motion_guard_node src/main.cpp)
target_link_libraries(motion_guard_node dyx3_motion_guard_node_lib)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS motion_guard_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(guard_core_test test/guard_core_test.cpp)
  target_link_libraries(guard_core_test dyx3_motion_guard_core)
  # Real-time DDS fault injection: tens of seconds, above the 60 s ament default.
  ament_add_gtest(motion_guard_node_test test/motion_guard_node_test.cpp TIMEOUT 300)
  target_link_libraries(motion_guard_node_test dyx3_motion_guard_node_lib)
  ament_target_dependencies(motion_guard_node_test rclcpp dyx3_interfaces)
endif()

ament_package()

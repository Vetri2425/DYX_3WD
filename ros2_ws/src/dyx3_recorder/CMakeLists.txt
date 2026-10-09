cmake_minimum_required(VERSION 3.16)
project(dyx3_recorder)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dyx3_interfaces REQUIRED)

# Pure C++ core: std + POSIX only, no ROS includes.
add_library(dyx3_recorder_core STATIC
  src/run_manifest.cpp
  src/ulog_capture.cpp
  src/param_snapshot.cpp
  src/bag_writer.cpp
  src/run_lifecycle.cpp)
target_include_directories(dyx3_recorder_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(dyx3_recorder_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(dyx3_recorder_core PRIVATE -Wall -Wextra -Wpedantic)

add_library(dyx3_recorder_node_lib STATIC src/recorder_node.cpp)
target_link_libraries(dyx3_recorder_node_lib PUBLIC dyx3_recorder_core)
ament_target_dependencies(dyx3_recorder_node_lib PUBLIC rclcpp dyx3_interfaces)
target_compile_options(dyx3_recorder_node_lib PRIVATE -Wall -Wextra -Wpedantic)

add_executable(recorder_node src/main.cpp)
target_link_libraries(recorder_node dyx3_recorder_node_lib)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS recorder_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(recorder_core_test test/recorder_core_test.cpp)
  target_link_libraries(recorder_core_test dyx3_recorder_core)
  ament_add_gtest(recorder_node_test test/recorder_node_test.cpp TIMEOUT 300)
  target_link_libraries(recorder_node_test dyx3_recorder_node_lib)
  ament_target_dependencies(recorder_node_test rclcpp dyx3_interfaces)
  # REC-002: the default param_nodes list is checked against the launch graph in the source tree.
  target_compile_definitions(recorder_node_test PRIVATE
    DYX3_CONTROL_GRAPH_LAUNCH="${CMAKE_CURRENT_SOURCE_DIR}/../dyx3_bringup/launch/control_graph.launch.py")
endif()

ament_package()

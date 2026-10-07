cmake_minimum_required(VERSION 3.16)
project(dyx3_system_gateway)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dyx3_interfaces REQUIRED)
find_package(Threads REQUIRED)

# Pure C++ core: std + POSIX only, no ROS includes.
add_library(dyx3_gateway_core STATIC
  src/json.cpp
  src/command_validator.cpp
  src/telemetry_snapshot.cpp
  src/ipc_server.cpp)
target_include_directories(dyx3_gateway_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(dyx3_gateway_core PUBLIC Threads::Threads)
set_target_properties(dyx3_gateway_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(dyx3_gateway_core PRIVATE -Wall -Wextra -Wpedantic)

add_library(dyx3_gateway_node_lib STATIC src/gateway_node.cpp)
target_link_libraries(dyx3_gateway_node_lib PUBLIC dyx3_gateway_core)
ament_target_dependencies(dyx3_gateway_node_lib PUBLIC rclcpp dyx3_interfaces)
target_compile_options(dyx3_gateway_node_lib PRIVATE -Wall -Wextra -Wpedantic)

add_executable(gateway_node src/main.cpp)
target_link_libraries(gateway_node dyx3_gateway_node_lib)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS gateway_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(gateway_core_test test/gateway_core_test.cpp)
  target_link_libraries(gateway_core_test dyx3_gateway_core)
  ament_add_gtest(gateway_node_test test/gateway_node_test.cpp TIMEOUT 300)
  target_link_libraries(gateway_node_test dyx3_gateway_node_lib)
  ament_target_dependencies(gateway_node_test rclcpp dyx3_interfaces)
endif()

ament_package()

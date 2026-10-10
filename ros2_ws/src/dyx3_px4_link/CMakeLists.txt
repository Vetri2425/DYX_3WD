cmake_minimum_required(VERSION 3.16)
project(dyx3_px4_link)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(dyx3_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Pure C++ core: no ROS, no px4_msgs. Everything the safety argument rests on lives here.
add_library(dyx3_px4_link_core STATIC
  src/message_hash.cpp
  src/rover_setpoint_writer.cpp
  src/dds_session.cpp
  src/msg_version_handshake.cpp
  src/offboard_heartbeat.cpp
  src/vehicle_state_assembler.cpp)
target_sources(dyx3_px4_link_core PRIVATE src/spray_ack_tokens.cpp)
target_include_directories(dyx3_px4_link_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(dyx3_px4_link_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(dyx3_px4_link_core PRIVATE -Wall -Wextra -Wpedantic)

# ROS wiring: the only translation unit that includes px4_msgs and touches /fmu/**.
add_library(dyx3_px4_link_node_lib STATIC src/px4_link_node.cpp)
target_link_libraries(dyx3_px4_link_node_lib PUBLIC dyx3_px4_link_core)
ament_target_dependencies(dyx3_px4_link_node_lib PUBLIC rclcpp px4_msgs dyx3_interfaces ament_index_cpp)
target_compile_options(dyx3_px4_link_node_lib PRIVATE -Wall -Wextra -Wpedantic)

add_executable(px4_link_node src/main.cpp)
target_link_libraries(px4_link_node dyx3_px4_link_node_lib)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS px4_link_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(FIXTURES ${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures)
  foreach(t message_hash_test setpoint_gate_test link_core_test)
    ament_add_gtest(${t} test/${t}.cpp)
    target_link_libraries(${t} dyx3_px4_link_core)
    target_compile_definitions(${t} PRIVATE DYX3_FIXTURES="${FIXTURES}")
  endforeach()
  ament_add_gtest(spray_ack_tokens_test test/spray_ack_tokens_test.cpp)
  target_link_libraries(spray_ack_tokens_test dyx3_px4_link_core)
  # Real-time DDS fault injection: tens of seconds, above the 60 s ament default.
  ament_add_gtest(px4_link_node_test test/px4_link_node_test.cpp TIMEOUT 300)
  target_link_libraries(px4_link_node_test dyx3_px4_link_node_lib)
  ament_target_dependencies(px4_link_node_test rclcpp px4_msgs dyx3_interfaces)
  target_compile_definitions(px4_link_node_test PRIVATE DYX3_FIXTURES="${FIXTURES}")
  # The same suite in event-driven mode (the production default): every test must pass in both.
  ament_add_gtest(px4_link_node_event_test test/px4_link_node_test.cpp TIMEOUT 300)
  target_link_libraries(px4_link_node_event_test dyx3_px4_link_node_lib)
  ament_target_dependencies(px4_link_node_event_test rclcpp px4_msgs dyx3_interfaces)
  target_compile_definitions(px4_link_node_event_test PRIVATE DYX3_FIXTURES="${FIXTURES}"
                             DYX3_TEST_EVENT_DRIVEN=1)
endif()

ament_package()

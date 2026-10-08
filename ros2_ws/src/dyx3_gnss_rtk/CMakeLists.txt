cmake_minimum_required(VERSION 3.16)
project(dyx3_gnss_rtk)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dyx3_interfaces REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(nlohmann_json REQUIRED)

# Pure C++ core (no ROS): RTCM framing, chunking, NTRIP protocol and socket client, health and GGA.
add_library(dyx3_gnss_rtk_core STATIC
  src/rtcm_parser.cpp
  src/rtcm_transport.cpp
  src/correction_health.cpp
  src/gga_provider.cpp
  src/ntrip_client.cpp
  src/validated_frame.cpp
  src/serial_port.cpp
  src/transport_sink.cpp
  src/injection_authority.cpp
  src/lora_source.cpp
  src/rtk_config.cpp
  src/control_socket.cpp
  src/worker_state.cpp)
target_include_directories(dyx3_gnss_rtk_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(dyx3_gnss_rtk_core PUBLIC Threads::Threads OpenSSL::SSL OpenSSL::Crypto nlohmann_json::nlohmann_json)
set_target_properties(dyx3_gnss_rtk_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(dyx3_gnss_rtk_core PRIVATE -Wall -Wextra -Wpedantic)

add_library(dyx3_gnss_rtk_node_lib STATIC src/rtk_node.cpp)
target_link_libraries(dyx3_gnss_rtk_node_lib PUBLIC dyx3_gnss_rtk_core)
ament_target_dependencies(dyx3_gnss_rtk_node_lib PUBLIC rclcpp dyx3_interfaces)
target_compile_options(dyx3_gnss_rtk_node_lib PRIVATE -Wall -Wextra -Wpedantic)

add_executable(rtk_node src/main.cpp)
target_link_libraries(rtk_node dyx3_gnss_rtk_node_lib)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS rtk_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(rtcm_test test/rtcm_test.cpp)
  target_link_libraries(rtcm_test dyx3_gnss_rtk_core)
  ament_add_gtest(ntrip_equivalence_test test/ntrip_equivalence_test.cpp)
  target_link_libraries(ntrip_equivalence_test dyx3_gnss_rtk_core)
  target_compile_definitions(ntrip_equivalence_test PRIVATE DYX3_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  ament_add_gtest(ntrip_client_test test/ntrip_client_test.cpp TIMEOUT 120)
  target_link_libraries(ntrip_client_test dyx3_gnss_rtk_core)
  target_compile_definitions(ntrip_client_test PRIVATE DYX3_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  ament_add_gtest(rtk_node_test test/rtk_node_test.cpp TIMEOUT 120)
  target_link_libraries(rtk_node_test dyx3_gnss_rtk_node_lib)
  ament_target_dependencies(rtk_node_test rclcpp dyx3_interfaces)
  ament_add_gtest(production_core_test test/production_core_test.cpp TIMEOUT 120)
  target_link_libraries(production_core_test dyx3_gnss_rtk_core util)
endif()

ament_package()

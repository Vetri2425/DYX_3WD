cmake_minimum_required(VERSION 3.16)
project(dyx3_rpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(dyx3_geometry REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dyx3_interfaces REQUIRED)
find_package(dyx3_mission REQUIRED)

# Pure C++ core: no ROS includes in any algorithm module (CLAUDE.md section 7).
add_library(dyx3_rpp_core STATIC
  src/rpp_params.cpp
  src/guidance.cpp
  src/speed_profile.cpp
  src/stop_pivot_fsm.cpp
  src/terminal.cpp
  src/path_conditioner.cpp
  src/rpp_core.cpp
  src/rpp_command.cpp
  src/diagnostics.cpp
  src/motion_output.cpp)
target_include_directories(dyx3_rpp_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(dyx3_rpp_core PUBLIC dyx3_geometry::dyx3_geometry)
set_target_properties(dyx3_rpp_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(dyx3_rpp_core PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)

# ROS wiring: the node and its executable. The algorithm library above has no ROS include.
add_library(dyx3_rpp_node_lib STATIC src/rpp_node.cpp)
target_link_libraries(dyx3_rpp_node_lib PUBLIC dyx3_rpp_core dyx3_mission::dyx3_mission_core)
ament_target_dependencies(dyx3_rpp_node_lib PUBLIC rclcpp dyx3_interfaces)
target_compile_options(dyx3_rpp_node_lib PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)

add_executable(rpp_node src/main.cpp)
target_link_libraries(rpp_node dyx3_rpp_node_lib)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS rpp_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(FIXTURES ${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures)
  ament_add_gtest(gate4_equivalence_test test/gate4_equivalence_test.cpp)
  target_link_libraries(gate4_equivalence_test dyx3_rpp_core)
  target_compile_definitions(gate4_equivalence_test PRIVATE DYX3_FIXTURES="${FIXTURES}")
  ament_add_gtest(conditioner_equivalence_test test/conditioner_equivalence_test.cpp)
  target_link_libraries(conditioner_equivalence_test dyx3_rpp_core dyx3_mission::dyx3_mission_core)
  target_compile_definitions(conditioner_equivalence_test PRIVATE DYX3_FIXTURES="${FIXTURES}")
  ament_add_gtest(app_plan_boundary_test test/app_plan_boundary_test.cpp)
  target_link_libraries(app_plan_boundary_test dyx3_rpp_core dyx3_mission::dyx3_mission_core)
  target_compile_definitions(app_plan_boundary_test PRIVATE DYX3_FIXTURES="${FIXTURES}")
  ament_add_gtest(orchestrator_equivalence_test test/orchestrator_equivalence_test.cpp)
  target_link_libraries(orchestrator_equivalence_test dyx3_rpp_core)
  target_compile_definitions(orchestrator_equivalence_test PRIVATE DYX3_FIXTURES="${FIXTURES}")
  # DDS fault injection on an injected clock with work-driven delivery (test/dds_test_support.hpp):
  # seconds. The timeout only bounds a hang on a loaded runner.
  ament_add_gtest(rpp_node_test test/rpp_node_test.cpp TIMEOUT 120)
  target_link_libraries(rpp_node_test dyx3_rpp_node_lib)
  ament_target_dependencies(rpp_node_test rclcpp dyx3_interfaces)
  # The same suite in event-driven mode (the production default): every test must pass in both.
  ament_add_gtest(rpp_node_event_test test/rpp_node_test.cpp TIMEOUT 120)
  target_link_libraries(rpp_node_event_test dyx3_rpp_node_lib)
  ament_target_dependencies(rpp_node_event_test rclcpp dyx3_interfaces)
  target_compile_definitions(rpp_node_event_test PRIVATE DYX3_TEST_EVENT_DRIVEN=1)
  ament_add_gtest(rpp_core_test test/rpp_core_test.cpp)
  target_link_libraries(rpp_core_test dyx3_rpp_core)
  ament_add_gtest(rpp_modules_test test/rpp_modules_test.cpp)
  target_link_libraries(rpp_modules_test dyx3_rpp_core)
  ament_add_gtest(rpp_params_test test/rpp_params_test.cpp)
  target_link_libraries(rpp_params_test dyx3_rpp_core)
  target_compile_definitions(rpp_params_test PRIVATE
    DYX3_REGISTRY="${CMAKE_CURRENT_SOURCE_DIR}/../../../docs/tuning/parameter_registry.md")
endif()

ament_package()

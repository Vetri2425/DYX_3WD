cmake_minimum_required(VERSION 3.16)
project(dyx3_mission)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(dyx3_interfaces REQUIRED)

# Pure C++ core: FSM, path-artifact reader, SHA-256, point journal. No ROS includes.
add_library(dyx3_mission_core STATIC
  src/mission_fsm.cpp
  src/frame_placement.cpp
  src/sha256.cpp
  src/path_artifact.cpp
  src/point_journal.cpp
  src/run_state.cpp)
target_include_directories(dyx3_mission_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(dyx3_mission_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(dyx3_mission_core PRIVATE -Wall -Wextra -Wpedantic)

# ROS wiring.
add_library(dyx3_mission_node_lib STATIC src/mission_node.cpp)
target_link_libraries(dyx3_mission_node_lib PUBLIC dyx3_mission_core)
ament_target_dependencies(dyx3_mission_node_lib PUBLIC rclcpp rclcpp_action dyx3_interfaces)
target_compile_options(dyx3_mission_node_lib PRIVATE -Wall -Wextra -Wpedantic)

add_executable(mission_node src/main.cpp)
target_link_libraries(mission_node dyx3_mission_node_lib)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS mission_node DESTINATION lib/${PROJECT_NAME})
# The artifact reader is exported so dyx3_spray reads the SAME content-addressed file the mission loads (one reader).
install(TARGETS dyx3_mission_core EXPORT export_dyx3_mission_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin INCLUDES DESTINATION include)
ament_export_targets(export_dyx3_mission_core HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(FIXTURES ${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures)
  foreach(t mission_fsm_test frame_placement_test sha256_test path_artifact_test point_journal_test)
    ament_add_gtest(${t} test/${t}.cpp)
    target_link_libraries(${t} dyx3_mission_core)
    target_compile_definitions(${t} PRIVATE DYX3_FIXTURES="${FIXTURES}")
  endforeach()
  ament_add_gtest(mission_node_test test/mission_node_test.cpp)
  target_link_libraries(mission_node_test dyx3_mission_node_lib)
  ament_target_dependencies(mission_node_test rclcpp rclcpp_action dyx3_interfaces)
  target_compile_definitions(mission_node_test PRIVATE DYX3_FIXTURES="${FIXTURES}")
endif()

ament_package()

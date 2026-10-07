cmake_minimum_required(VERSION 3.16)
project(dyx3_spray)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dyx3_interfaces REQUIRED)
find_package(dyx3_mission REQUIRED)

# Pure C++ core: no ROS includes. The valve's behaviour rests on these files.
add_library(dyx3_spray_core STATIC
  src/spray_params.cpp
  src/spray_fsm.cpp
  src/flow_model.cpp
  src/boundary_projection.cpp
  src/safety_lease.cpp
  src/spray_gates.cpp
  src/spray_controller.cpp
  src/watchdog_core.cpp)
target_include_directories(dyx3_spray_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(dyx3_spray_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(dyx3_spray_core PRIVATE -Wall -Wextra -Wpedantic)

add_library(dyx3_spray_node_lib STATIC src/spray_node.cpp src/safety_watchdog_node.cpp)
target_link_libraries(dyx3_spray_node_lib PUBLIC dyx3_spray_core dyx3_mission::dyx3_mission_core)
ament_target_dependencies(dyx3_spray_node_lib PUBLIC rclcpp dyx3_interfaces)
target_compile_options(dyx3_spray_node_lib PRIVATE -Wall -Wextra -Wpedantic)

add_executable(spray_node src/main.cpp)
target_link_libraries(spray_node dyx3_spray_node_lib)
# Its own process: a crashed or frozen controller must not take the watchdog down.
add_executable(spray_watchdog src/watchdog_main.cpp)
target_link_libraries(spray_watchdog dyx3_spray_node_lib)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS spray_node spray_watchdog DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(FIXTURES ${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures)
  foreach(t spray_core_test spray_gates_test spray_defect_test spray_equivalence_test)
    ament_add_gtest(${t} test/${t}.cpp)
    target_link_libraries(${t} dyx3_spray_core)
    target_compile_definitions(${t} PRIVATE DYX3_FIXTURES="${FIXTURES}")
  endforeach()
  # Real-time DDS fault injection: tens of seconds, above the 60 s ament default.
  ament_add_gtest(spray_node_test test/spray_node_test.cpp TIMEOUT 300)
  target_link_libraries(spray_node_test dyx3_spray_node_lib)
  ament_target_dependencies(spray_node_test rclcpp dyx3_interfaces)
  target_compile_definitions(spray_node_test PRIVATE DYX3_FIXTURES="${FIXTURES}")
endif()

ament_package()

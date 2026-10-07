cmake_minimum_required(VERSION 3.16)
project(dyx3_rpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)

# Pure C++ core: no ROS includes in any algorithm module (CLAUDE.md section 7).
add_library(dyx3_rpp_core STATIC
  src/rpp_params.cpp)
target_include_directories(dyx3_rpp_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(dyx3_rpp_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(dyx3_rpp_core PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(rpp_params_test test/rpp_params_test.cpp)
  target_link_libraries(rpp_params_test dyx3_rpp_core)
  target_compile_definitions(rpp_params_test PRIVATE
    DYX3_REGISTRY="${CMAKE_CURRENT_SOURCE_DIR}/../../../docs/tuning/parameter_registry.md")
endif()

ament_package()

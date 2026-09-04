cmake_minimum_required(VERSION 3.16)
project(dyx3_geometry CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# This package has NO ROS dependency by design. Most of the rover's accuracy
# lives here, and it must be testable on a laptop in seconds rather than on the
# rover in an afternoon.
#
#   Native (no ROS):  cmake -S . -B build -DDYX3_NATIVE_TESTS=ON && ctest --test-dir build
#   In-workspace:     colcon build
option(DYX3_NATIVE_TESTS "Build standalone without ament/ROS" OFF)

# add_library(dyx3_geometry src/*.cpp) lands once the contract is written.
# target_include_directories(dyx3_geometry PUBLIC include)

if(DYX3_NATIVE_TESTS)
  enable_testing()
  # add_executable(geometry_test test/geometry_test.cpp)
  # target_link_libraries(geometry_test dyx3_geometry)
  # add_test(NAME geometry_test COMMAND geometry_test)
  return()
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()

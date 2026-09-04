cmake_minimum_required(VERSION 3.16)
project(dyx3_spray)

find_package(ament_cmake REQUIRED)

# Skeleton: no build targets yet.
# add_library()/add_executable() land once this package's contract in
# docs/contracts/ is written and reviewed.

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(<test_name> test/<file>.cpp) goes here once tests exist.
endif()

ament_package()

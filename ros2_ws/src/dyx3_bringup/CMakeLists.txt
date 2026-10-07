cmake_minimum_required(VERSION 3.16)
project(dyx3_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_control_graph_launch test/test_control_graph_launch.py)
endif()

ament_package()

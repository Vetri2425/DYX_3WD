cmake_minimum_required(VERSION 3.16)
project(dyx3_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Frozen in Milestone 2. Every other package is written against these.
# A field change requires a version bump and a migration note (CI-enforced).
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionSetpoint.msg"
  "msg/MotionSetpointStatus.msg"
  "msg/VehicleState.msg"
  "msg/EstimatorHealth.msg"
  "msg/RppStatus.msg"
  "msg/MissionState.msg"
  "msg/PointResult.msg"
  "msg/RtkStatus.msg"
  "msg/SprayState.msg"
  "msg/RecorderStatus.msg"
  "msg/SafetyGateStatus.msg"
  "msg/OperatorLinkStatus.msg"
  "msg/EmergencyStopState.msg"
  "msg/Px4LinkStatus.msg"
  "msg/GnssReport.msg"
  "msg/RtcmData.msg"
  "msg/UlogChunk.msg"
  "srv/StartMission.srv"
  "srv/PauseMission.srv"
  "srv/ResumeMission.srv"
  "srv/AbortMission.srv"
  "srv/SkipPoint.srv"
  "srv/SetEmergencyStop.srv"
  "srv/ArmDisarm.srv"
  "srv/SetOffboard.srv"
  "action/ExecuteMission.action"
  DEPENDENCIES builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(interface_abi_test test/interface_abi_test.cpp)
  if(TARGET interface_abi_test)
    rosidl_target_interfaces(interface_abi_test ${PROJECT_NAME} "rosidl_typesupport_cpp")
  endif()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()

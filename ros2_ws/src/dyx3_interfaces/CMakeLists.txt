cmake_minimum_required(VERSION 3.16)
project(dyx3_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Frozen in Milestone 2. Every other package is written against these.
# A field change requires a version bump and a migration note (CI-enforced).
#
# rosidl_generate_interfaces(${PROJECT_NAME}
#   msg/MotionSetpoint.msg
#   msg/MotionSetpointStatus.msg
#   msg/VehicleState.msg
#   msg/RppStatus.msg
#   msg/MissionState.msg
#   msg/PointResult.msg
#   msg/RtkStatus.msg
#   msg/SprayState.msg
#   msg/RecorderStatus.msg
#   srv/StartMission.srv
#   srv/PauseMission.srv
#   srv/ResumeMission.srv
#   srv/AbortMission.srv
#   srv/SkipPoint.srv
#   srv/SetEmergencyStop.srv
#   action/ExecuteMission.action
#   DEPENDENCIES builtin_interfaces
# )

ament_export_dependencies(rosidl_default_runtime)
ament_package()

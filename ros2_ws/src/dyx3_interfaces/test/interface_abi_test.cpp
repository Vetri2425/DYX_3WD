#include <gtest/gtest.h>

#include "builtin_interfaces/msg/time.hpp"
#include "dyx3_interfaces/action/execute_mission.hpp"
#include "dyx3_interfaces/msg/emergency_stop_state.hpp"
#include "dyx3_interfaces/msg/estimator_health.hpp"
#include "dyx3_interfaces/msg/gnss_report.hpp"
#include "dyx3_interfaces/msg/mission_state.hpp"
#include "dyx3_interfaces/msg/motion_setpoint.hpp"
#include "dyx3_interfaces/msg/motion_setpoint_status.hpp"
#include "dyx3_interfaces/msg/ntrip_status.hpp"
#include "dyx3_interfaces/msg/operator_link_status.hpp"
#include "dyx3_interfaces/msg/point_result.hpp"
#include "dyx3_interfaces/msg/px4_link_status.hpp"
#include "dyx3_interfaces/msg/recorder_status.hpp"
#include "dyx3_interfaces/msg/rpp_status.hpp"
#include "dyx3_interfaces/msg/rtcm_data.hpp"
#include "dyx3_interfaces/msg/rtk_status.hpp"
#include "dyx3_interfaces/msg/safety_gate_status.hpp"
#include "dyx3_interfaces/msg/spray_actuator_ack.hpp"
#include "dyx3_interfaces/msg/spray_actuator_command.hpp"
#include "dyx3_interfaces/msg/spray_lease.hpp"
#include "dyx3_interfaces/msg/spray_state.hpp"
#include "dyx3_interfaces/msg/spray_status.hpp"
#include "dyx3_interfaces/msg/spray_watchdog_status.hpp"
#include "dyx3_interfaces/msg/ulog_chunk.hpp"
#include "dyx3_interfaces/msg/vehicle_state.hpp"
#include "dyx3_interfaces/srv/abort_mission.hpp"
#include "dyx3_interfaces/srv/arm_disarm.hpp"
#include "dyx3_interfaces/srv/pause_mission.hpp"
#include "dyx3_interfaces/srv/resume_mission.hpp"
#include "dyx3_interfaces/srv/set_emergency_stop.hpp"
#include "dyx3_interfaces/srv/set_offboard.hpp"
#include "dyx3_interfaces/srv/set_spray_manual.hpp"
#include "dyx3_interfaces/srv/skip_point.hpp"
#include "dyx3_interfaces/srv/start_mission.hpp"

namespace {

void ExpectZeroTime(const builtin_interfaces::msg::Time &time) {
  EXPECT_EQ(time.sec, 0);
  EXPECT_EQ(time.nanosec, 0U);
}

TEST(MotionSetpointAbi, ConstantsAndSafeDefault) {
  using Msg = dyx3_interfaces::msg::MotionSetpoint;
  EXPECT_EQ(Msg::MODE_STOP, 0U);
  EXPECT_EQ(Msg::MODE_TRACK_HEADING, 1U);
  EXPECT_EQ(Msg::MODE_TRACK_RATE, 2U);
  EXPECT_EQ(Msg::MODE_PIVOT, 3U);
  EXPECT_EQ(Msg::MODE_CREEP, 4U);

  const Msg message{};
  ExpectZeroTime(message.stamp);
  EXPECT_EQ(message.seq, 0U);
  EXPECT_EQ(message.mode, Msg::MODE_STOP);
  EXPECT_FLOAT_EQ(message.speed_body_x, 0.0F);
  EXPECT_FLOAT_EQ(message.yaw_setpoint, 0.0F);
  EXPECT_FLOAT_EQ(message.yaw_rate_setpoint, 0.0F);
  EXPECT_FALSE(message.valid);
}

TEST(MotionSetpointStatusAbi, ConstantsAndSafeDefault) {
  using Msg = dyx3_interfaces::msg::MotionSetpointStatus;
  EXPECT_EQ(Msg::REASON_OK, 0U);
  EXPECT_EQ(Msg::REASON_INVALID_MESSAGE, 1U);
  EXPECT_EQ(Msg::REASON_STALE, 2U);
  EXPECT_EQ(Msg::REASON_SEQUENCE, 3U);
  EXPECT_EQ(Msg::REASON_MISSION_GATE, 4U);
  EXPECT_EQ(Msg::REASON_ESTOP, 5U);
  EXPECT_EQ(Msg::REASON_RTK_GATE, 6U);
  EXPECT_EQ(Msg::REASON_LIMIT_CLAMPED, 7U);
  EXPECT_EQ(Msg::REASON_PX4_LINK_UNHEALTHY, 8U);
  EXPECT_EQ(Msg::REASON_HEADING_UNHEALTHY, 9U);
  EXPECT_EQ(Msg::REASON_OPERATOR_LINK_LOST, 10U);
  EXPECT_EQ(Msg::REASON_ARMING_GATE, 11U);
  EXPECT_EQ(Msg::REASON_ESTIMATOR_UNHEALTHY, 12U);
  const Msg message{};
  ExpectZeroTime(message.stamp);
  EXPECT_EQ(message.input_seq, 0U);
  EXPECT_EQ(message.reason_code, Msg::REASON_OK);
  EXPECT_FALSE(message.accepted);
  EXPECT_FALSE(message.clamped);
  EXPECT_EQ(message.mode, 0U);
  EXPECT_FLOAT_EQ(message.speed_body_x, 0.0F);
  EXPECT_FLOAT_EQ(message.yaw_setpoint, 0.0F);
  EXPECT_FLOAT_EQ(message.yaw_rate_setpoint, 0.0F);
  EXPECT_FALSE(message.valid);
}

TEST(VehicleStateAbi, SafeDefault) {
  const dyx3_interfaces::msg::VehicleState message{};
  ExpectZeroTime(message.stamp);
  ExpectZeroTime(message.px4_sample_stamp);
  EXPECT_FALSE(message.position_valid);
  EXPECT_FALSE(message.velocity_valid);
  EXPECT_FALSE(message.attitude_valid);
  EXPECT_FLOAT_EQ(message.north_m, 0.0F);
  EXPECT_FLOAT_EQ(message.east_m, 0.0F);
  EXPECT_FLOAT_EQ(message.down_m, 0.0F);
  EXPECT_FLOAT_EQ(message.velocity_north_mps, 0.0F);
  EXPECT_FLOAT_EQ(message.velocity_east_mps, 0.0F);
  EXPECT_FLOAT_EQ(message.velocity_down_mps, 0.0F);
  for (const auto value : message.q_frd_to_ned) EXPECT_FLOAT_EQ(value, 0.0F);
  EXPECT_FLOAT_EQ(message.heading_rad, 0.0F);
  EXPECT_FLOAT_EQ(message.yaw_rate_radps, 0.0F);
  EXPECT_EQ(message.xy_reset_counter, 0U);
  EXPECT_FLOAT_EQ(message.delta_north_m, 0.0F);
  EXPECT_FLOAT_EQ(message.delta_east_m, 0.0F);
  EXPECT_FALSE(message.global_reference_valid);
  EXPECT_DOUBLE_EQ(message.reference_latitude_deg, 0.0);
  EXPECT_DOUBLE_EQ(message.reference_longitude_deg, 0.0);
  EXPECT_FLOAT_EQ(message.reference_altitude_m_amsl, 0.0F);
  EXPECT_EQ(message.arming_state, 0U);
  EXPECT_EQ(message.nav_state, 0U);
  EXPECT_FALSE(message.failsafe);
  EXPECT_FALSE(message.preflight_checks_pass);
}

TEST(EstimatorHealthAbi, DefaultIsUnhealthyByConstruction) {
  const dyx3_interfaces::msg::EstimatorHealth h{};
  ExpectZeroTime(h.stamp);
  ExpectZeroTime(h.px4_sample_stamp);
  // flags_valid=false means "no data": the guard must treat it as unhealthy.
  EXPECT_FALSE(h.flags_valid);
  EXPECT_FALSE(h.test_ratios_valid);
  EXPECT_FALSE(h.gnss_yaw_fusion_intended);
  EXPECT_FALSE(h.gnss_yaw_fault);
  EXPECT_FALSE(h.reject_yaw);
  EXPECT_FALSE(h.reject_hor_pos);
  EXPECT_FALSE(h.reject_hor_vel);
  EXPECT_FALSE(h.inertial_dead_reckoning);
  EXPECT_EQ(h.innovation_fault_status_changes, 0U);
  EXPECT_FLOAT_EQ(h.yaw_test_ratio, 0.0F);
  EXPECT_FLOAT_EQ(h.position_test_ratio, 0.0F);
  EXPECT_FLOAT_EQ(h.velocity_test_ratio, 0.0F);
}

TEST(SafetyMessagesAbi, DefaultsAreFailSafe) {
  const dyx3_interfaces::msg::SafetyGateStatus gate{};
  ExpectZeroTime(gate.stamp);
  EXPECT_FALSE(gate.ok);  // no data == not safe
  EXPECT_EQ(gate.reason_code, 0U);

  const dyx3_interfaces::msg::OperatorLinkStatus link{};
  ExpectZeroTime(link.stamp);
  EXPECT_FALSE(link.alive);
  EXPECT_FLOAT_EQ(link.age_s, 0.0F);

  const dyx3_interfaces::msg::EmergencyStopState estop{};
  ExpectZeroTime(estop.stamp);
  EXPECT_FALSE(
      estop.asserted);  // absence of a fresh message is what consumers must treat as asserted
  EXPECT_TRUE(estop.source.empty());
}

TEST(Px4LinkAbi, DefaultsAreUnhealthyAndConstantsAreFrozen) {
  using L = dyx3_interfaces::msg::Px4LinkStatus;
  const L l{};
  ExpectZeroTime(l.stamp);
  EXPECT_FALSE(l.session_alive);
  EXPECT_FALSE(l.handshake_ok);
  EXPECT_FALSE(l.offboard_heartbeat_active);
  EXPECT_FALSE(l.failing_to_zero);
  EXPECT_EQ(l.fault, L::FAULT_NONE);
  EXPECT_EQ(L::FAULT_NO_SESSION, 1U);
  EXPECT_EQ(L::FAULT_HANDSHAKE_MISMATCH, 2U);
  EXPECT_EQ(L::FAULT_TOPIC_STALE, 3U);
  EXPECT_EQ(L::FAULT_COMMAND_STALE, 4U);
  EXPECT_EQ(L::FAULT_LOOP_OVERRUN, 5U);
  EXPECT_EQ(L::FAULT_HANDSHAKE_PENDING, 6U);
  EXPECT_EQ(l.stale_topics_mask, 0U);
  EXPECT_EQ(l.loop_overrun_count, 0U);
  EXPECT_EQ(l.command_gap_events, 0U);
  EXPECT_EQ(l.session_resets, 0U);
  EXPECT_FALSE(l.timesync_valid);  // 0.7.0: not valid until a sample arrived
  EXPECT_EQ(l.timesync_offset_us, 0);
  EXPECT_EQ(l.timesync_round_trip_us, 0U);

  const dyx3_interfaces::msg::GnssReport g{};
  EXPECT_FALSE(g.valid);
  EXPECT_EQ(g.fix_type, 0U);
  EXPECT_EQ(g.latitude_deg, 0.0);
  EXPECT_EQ(g.hdop, 0.0F);
  const dyx3_interfaces::msg::NtripStatus n{};  // fail-safe default: no corrections
  EXPECT_FALSE(n.connected);
  EXPECT_FALSE(n.streaming);
  EXPECT_EQ(n.state, dyx3_interfaces::msg::NtripStatus::STATE_STARTING);
  EXPECT_EQ(n.frames_total, 0U);
  EXPECT_TRUE(n.last_error.empty());
  EXPECT_EQ(dyx3_interfaces::msg::NtripStatus::STATE_STREAMING, 2);
  const dyx3_interfaces::msg::SprayActuatorCommand sc{};  // default = OFF from the controller
  EXPECT_FALSE(sc.on);
  EXPECT_EQ(sc.source, dyx3_interfaces::msg::SprayActuatorCommand::SOURCE_CONTROLLER);
  const dyx3_interfaces::msg::SprayActuatorAck sa{};
  EXPECT_FALSE(sa.success);
  EXPECT_EQ(dyx3_interfaces::msg::SprayActuatorAck::RESULT_LINK_REFUSED, 255);
  const dyx3_interfaces::msg::SprayLease sl{};  // default = no ON
  EXPECT_FALSE(sl.allow_on);
  const dyx3_interfaces::msg::SprayWatchdogStatus sw{};
  EXPECT_FALSE(sw.off_authority_ready);
  EXPECT_FALSE(sw.allow_on);
  const dyx3_interfaces::msg::SprayStatus ss{};
  EXPECT_FALSE(ss.spraying);
  EXPECT_FALSE(ss.safety_ok);
  EXPECT_EQ(ss.fsm_state, dyx3_interfaces::msg::SprayStatus::FSM_OFF_UNCONFIRMED);
  EXPECT_FALSE(dyx3_interfaces::srv::SetSprayManual::Request{}.on);
  EXPECT_FALSE(dyx3_interfaces::srv::SetSprayManual::Response{}.accepted);
  const dyx3_interfaces::msg::RtcmData r{};
  EXPECT_TRUE(r.data.empty());
  const dyx3_interfaces::msg::UlogChunk u{};
  EXPECT_TRUE(u.data.empty());
  EXPECT_FALSE(dyx3_interfaces::srv::ArmDisarm::Request{}.arm);  // default request is "disarm"
  EXPECT_FALSE(dyx3_interfaces::srv::ArmDisarm::Response{}.accepted);
  EXPECT_FALSE(dyx3_interfaces::srv::SetOffboard::Request{}.enable);
  EXPECT_FALSE(dyx3_interfaces::srv::SetOffboard::Response{}.accepted);
}

TEST(StatusMessageAbi, ConstantsAndSafeDefaults) {
  using Rpp = dyx3_interfaces::msg::RppStatus;
  EXPECT_EQ(Rpp::STATE_IDLE, 0U);
  EXPECT_EQ(Rpp::STATE_TRACKING, 1U);
  EXPECT_EQ(Rpp::STATE_STOPPING, 2U);
  EXPECT_EQ(Rpp::STATE_PIVOTING, 3U);
  EXPECT_EQ(Rpp::STATE_CREEPING, 4U);
  EXPECT_EQ(Rpp::STATE_COMPLETE, 5U);
  EXPECT_EQ(Rpp::STATE_ERROR, 6U);
  EXPECT_EQ(Rpp::STATE_LOADED, 7U);
  const Rpp rpp{};
  ExpectZeroTime(rpp.stamp);
  EXPECT_EQ(rpp.state, Rpp::STATE_IDLE);
  EXPECT_EQ(rpp.mission_id, 0U);
  EXPECT_EQ(rpp.run_index, 0U);
  EXPECT_FLOAT_EQ(rpp.cross_track_right_m, 0.0F);
  EXPECT_FLOAT_EQ(rpp.heading_error_rad, 0.0F);
  EXPECT_FLOAT_EQ(rpp.commanded_speed_mps, 0.0F);
  EXPECT_FLOAT_EQ(rpp.commanded_yaw_rate_radps, 0.0F);
  EXPECT_FLOAT_EQ(rpp.loop_jitter_us, 0.0F);
  EXPECT_FLOAT_EQ(rpp.loop_jitter_max_us, 0.0F);
  EXPECT_EQ(rpp.loop_overrun_count, 0U);
  EXPECT_EQ(rpp.tick_state, 0);
  EXPECT_EQ(rpp.segment_state, 0U);
  EXPECT_FALSE(rpp.spray_request);  // the safe default: no request to open the valve
  EXPECT_EQ(rpp.handoff, 0U);
  EXPECT_EQ(rpp.rtk_reason, 0U);

  using Mission = dyx3_interfaces::msg::MissionState;
  EXPECT_EQ(Mission::STATE_IDLE, 0U);
  EXPECT_EQ(Mission::STATE_LOADING, 1U);
  EXPECT_EQ(Mission::STATE_READY, 2U);
  EXPECT_EQ(Mission::STATE_RUNNING, 3U);
  EXPECT_EQ(Mission::STATE_PAUSED, 4U);
  EXPECT_EQ(Mission::STATE_COMPLETED, 5U);
  EXPECT_EQ(Mission::STATE_ABORTED, 6U);
  EXPECT_EQ(Mission::STATE_ERROR, 7U);
  EXPECT_EQ(Mission::REASON_NONE, 0U);
  EXPECT_EQ(Mission::REASON_OPERATOR, 1U);
  EXPECT_EQ(Mission::REASON_SAFETY, 2U);
  EXPECT_EQ(Mission::REASON_RTK, 3U);
  EXPECT_EQ(Mission::REASON_PATH_ERROR, 4U);
  EXPECT_EQ(Mission::REASON_INTERNAL_ERROR, 5U);
  const Mission mission{};
  ExpectZeroTime(mission.stamp);
  EXPECT_EQ(mission.state, Mission::STATE_IDLE);
  EXPECT_EQ(mission.mission_id, 0U);
  EXPECT_EQ(mission.run_index, 0U);
  EXPECT_EQ(mission.point_index, 0U);
  EXPECT_EQ(mission.reason_code, Mission::REASON_NONE);
  EXPECT_TRUE(mission.path_artifact_sha256.empty());

  using Point = dyx3_interfaces::msg::PointResult;
  EXPECT_EQ(Point::RESULT_NONE, 0U);
  EXPECT_EQ(Point::RESULT_COMPLETED, 1U);
  EXPECT_EQ(Point::RESULT_SKIPPED, 2U);
  EXPECT_EQ(Point::RESULT_FAILED, 3U);
  const Point point{};
  ExpectZeroTime(point.stamp);
  EXPECT_EQ(point.mission_id, 0U);
  EXPECT_EQ(point.point_index, 0U);
  EXPECT_EQ(point.result_code, Point::RESULT_NONE);
  EXPECT_FLOAT_EQ(point.north_m, 0.0F);
  EXPECT_FLOAT_EQ(point.east_m, 0.0F);
  EXPECT_FLOAT_EQ(point.error_m, 0.0F);
}

TEST(HealthMessageAbi, ConstantsAndSafeDefaults) {
  using Rtk = dyx3_interfaces::msg::RtkStatus;
  EXPECT_EQ(Rtk::FIX_UNKNOWN, 0U);
  EXPECT_EQ(Rtk::FIX_NONE, 1U);
  EXPECT_EQ(Rtk::FIX_2D, 2U);
  EXPECT_EQ(Rtk::FIX_3D, 3U);
  EXPECT_EQ(Rtk::FIX_RTCM_CODE_DIFFERENTIAL, 4U);
  EXPECT_EQ(Rtk::FIX_EXTRAPOLATED, 8U);
  EXPECT_EQ(Rtk::FIX_RTK_FLOAT, 5U);
  EXPECT_EQ(Rtk::FIX_RTK_FIXED, 6U);
  const Rtk rtk{};
  ExpectZeroTime(rtk.stamp);
  EXPECT_EQ(rtk.fix_type, Rtk::FIX_UNKNOWN);
  EXPECT_FALSE(rtk.corrections_fresh);
  EXPECT_FLOAT_EQ(rtk.correction_age_s, 0.0F);
  EXPECT_FLOAT_EQ(rtk.horizontal_accuracy_m, 0.0F);
  EXPECT_EQ(rtk.satellites_used, 0U);

  using Spray = dyx3_interfaces::msg::SprayState;
  EXPECT_EQ(Spray::STATE_OFF, 0U);
  EXPECT_EQ(Spray::STATE_ON, 1U);
  EXPECT_EQ(Spray::STATE_FAULT, 2U);
  const Spray spray{};
  ExpectZeroTime(spray.stamp);
  EXPECT_EQ(spray.state, Spray::STATE_OFF);
  EXPECT_FALSE(spray.enabled);
  EXPECT_FALSE(spray.watchdog_healthy);
  EXPECT_FLOAT_EQ(spray.commanded_flow_normalized, 0.0F);

  using Recorder = dyx3_interfaces::msg::RecorderStatus;
  EXPECT_EQ(Recorder::STATE_IDLE, 0U);
  EXPECT_EQ(Recorder::STATE_RECORDING, 1U);
  EXPECT_EQ(Recorder::STATE_FINALIZING, 2U);
  EXPECT_EQ(Recorder::STATE_ERROR, 3U);
  const Recorder recorder{};
  ExpectZeroTime(recorder.stamp);
  EXPECT_EQ(recorder.state, Recorder::STATE_IDLE);
  EXPECT_FALSE(recorder.bag_healthy);
  EXPECT_EQ(recorder.bytes_written, 0U);
  EXPECT_EQ(recorder.free_bytes, 0U);
}

TEST(ServiceAndActionAbi, ConstantsAndSafeDefaults) {
  using Start = dyx3_interfaces::srv::StartMission;
  EXPECT_EQ(Start::Response::REASON_OK, 0U);
  EXPECT_EQ(Start::Response::REASON_INVALID_ARTIFACT, 1U);
  EXPECT_EQ(Start::Response::REASON_BUSY, 2U);
  EXPECT_EQ(Start::Response::REASON_SAFETY_GATE, 3U);
  EXPECT_TRUE(Start::Request{}.path_artifact_sha256.empty());
  EXPECT_FALSE(Start::Response{}.accepted);
  EXPECT_EQ(Start::Response{}.reason_code, 0U);
  EXPECT_EQ(Start::Response{}.mission_id, 0U);

  using Pause = dyx3_interfaces::srv::PauseMission;
  EXPECT_EQ(Pause::Response::REASON_OK, 0U);
  EXPECT_EQ(Pause::Response::REASON_NOT_RUNNING, 1U);
  EXPECT_EQ(Pause::Response::REASON_SAFETY_GATE, 2U);
  EXPECT_FALSE(Pause::Response{}.accepted);
  EXPECT_EQ(Pause::Response{}.reason_code, 0U);
  using Resume = dyx3_interfaces::srv::ResumeMission;
  EXPECT_EQ(Resume::Response::REASON_OK, 0U);
  EXPECT_EQ(Resume::Response::REASON_NOT_PAUSED, 1U);
  EXPECT_EQ(Resume::Response::REASON_SAFETY_GATE, 2U);
  EXPECT_FALSE(Resume::Response{}.accepted);
  EXPECT_EQ(Resume::Response{}.reason_code, 0U);
  using Abort = dyx3_interfaces::srv::AbortMission;
  EXPECT_EQ(Abort::Request::REASON_UNSPECIFIED, 0U);
  EXPECT_EQ(Abort::Request::REASON_OPERATOR, 1U);
  EXPECT_EQ(Abort::Request::REASON_SAFETY, 2U);
  EXPECT_EQ(Abort::Response::REASON_OK, 0U);
  EXPECT_EQ(Abort::Response::REASON_NOT_ACTIVE, 1U);
  EXPECT_EQ(Abort::Request{}.reason_code, Abort::Request::REASON_UNSPECIFIED);
  EXPECT_FALSE(Abort::Response{}.accepted);
  EXPECT_EQ(Abort::Response{}.reason_code, 0U);
  using Skip = dyx3_interfaces::srv::SkipPoint;
  EXPECT_EQ(Skip::Response::REASON_OK, 0U);
  EXPECT_EQ(Skip::Response::REASON_NO_ACTIVE_POINT, 1U);
  EXPECT_EQ(Skip::Response::REASON_NOT_RUNNING, 2U);
  EXPECT_FALSE(Skip::Response{}.accepted);
  EXPECT_EQ(Skip::Response{}.reason_code, 0U);
  EXPECT_EQ(Skip::Response{}.skipped_point_index, 0U);
  using Estop = dyx3_interfaces::srv::SetEmergencyStop;
  EXPECT_EQ(Estop::Response::REASON_OK, 0U);
  EXPECT_EQ(Estop::Response::REASON_INVALID_SOURCE, 1U);
  EXPECT_FALSE(Estop::Request{}.asserted);
  EXPECT_TRUE(Estop::Request{}.source.empty());
  EXPECT_FALSE(Estop::Response{}.accepted);
  EXPECT_EQ(Estop::Response{}.reason_code, 0U);

  using Action = dyx3_interfaces::action::ExecuteMission;
  EXPECT_EQ(Action::Result::RESULT_COMPLETED, 0U);
  EXPECT_EQ(Action::Result::RESULT_ABORTED, 1U);
  EXPECT_EQ(Action::Result::RESULT_ERROR, 2U);
  EXPECT_TRUE(Action::Goal{}.path_artifact_sha256.empty());
  EXPECT_EQ(Action::Result{}.result_code, 0U);
  EXPECT_EQ(Action::Result{}.completed_runs, 0U);
  EXPECT_EQ(Action::Feedback{}.run_index, 0U);
  EXPECT_EQ(Action::Feedback{}.point_index, 0U);
  EXPECT_FLOAT_EQ(Action::Feedback{}.progress_fraction, 0.0F);
  EXPECT_EQ(Action::Feedback{}.mission_state, 0U);
}

}  // namespace

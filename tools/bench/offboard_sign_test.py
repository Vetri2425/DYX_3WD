#!/usr/bin/env python3
"""Step 5 bench test: PX4 Offboard sign convention for the 3WD differential rover.

Sends exactly what dyx3_px4_link sends (src/px4_link_node.cpp publish_setpoint_set): OffboardControlMode with
velocity=true, a NaN TrajectorySetpoint, RoverSpeedSetpoint, RoverAttitudeSetpoint and RoverRateSetpoint at 50 Hz,
plus VEHICLE_CMD_DO_SET_MODE(1, 6) for OFFBOARD. It does NOT go through RPP or motion_guard: that chain is step 7.

Preconditions (the script checks the ones it can):
  * the control graph is stopped (sudo systemctl stop dyx3-ros), so nothing else writes /fmu/in;
  * the operator has ARMED the rover (RC/QGC, Manual) with the RC kill switch in hand;
  * spray hardware off.

Sequence (each motion step is capped in size and time; STOP between steps):
  prestream STOP -> OFFBOARD -> STOP -> +speed -> STOP -> +yaw rate -> STOP -> -yaw rate -> STOP
  [--heading: turn to current yaw + 30 deg]  [--loss-test: stop streaming while moving, expect disarm]
  -> STOP -> DISARM.

wheel_encoders and vehicle_angular_velocity are NOT exported over DDS by our firmware (dds_topics.yaml), so the
script judges rotation from /fmu/out/vehicle_attitude (yaw rate, positive = clockwise, NED) wheels down, and prints
each step's UTC window: the wheel-encoder signs per step are then checked in the PX4 ULog (wheel_speed[0] = right,
[1] = left, forward positive, proven 2026-10-10). Wheels up, the yaw does not move: judge by eye + the ULog.

Run on the rover as the service user:
  sudo -u dyx3 env ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 ROS_HOME=/tmp/rh bash -c \
    '. /opt/ros/humble/setup.bash; . /opt/dyx3/current/ros2_ws/install/setup.bash; \
     python3 /opt/dyx3/current/tools/bench/offboard_sign_test.py --wheels-up'
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
import px4_msgs.msg as P

NAV_OFFBOARD = 14
ARMED = 2
CMD_DO_SET_MODE = 176
CMD_ARM_DISARM = 400
NAN = float("nan")
MAX_SPEED = 0.5      # m/s hard cap, whatever the arguments say
MAX_RATE = 0.6       # rad/s hard cap
MAX_STEP_S = 4.0     # s hard cap per motion step


def now_us():
    return int(time.time() * 1e6)


class Bench(Node):
    def __init__(self):
        super().__init__("offboard_sign_test")
        rel1 = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        rel10 = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.p_ocm = self.create_publisher(P.OffboardControlMode, "/fmu/in/offboard_control_mode", rel1)
        self.p_traj = self.create_publisher(P.TrajectorySetpoint, "/fmu/in/trajectory_setpoint", rel1)
        self.p_speed = self.create_publisher(P.RoverSpeedSetpoint, "/fmu/in/rover_speed_setpoint", rel1)
        self.p_att = self.create_publisher(P.RoverAttitudeSetpoint, "/fmu/in/rover_attitude_setpoint", rel1)
        self.p_rate = self.create_publisher(P.RoverRateSetpoint, "/fmu/in/rover_rate_setpoint", rel1)
        self.p_cmd = self.create_publisher(P.VehicleCommand, "/fmu/in/vehicle_command", rel10)
        self.status = None
        self.yaw = None
        self.samples = []  # (monotonic t, yaw rad)
        sd = qos_profile_sensor_data
        self.create_subscription(P.VehicleStatus, "/fmu/out/vehicle_status_v1", self._on_status, sd)
        self.create_subscription(P.VehicleAttitude, "/fmu/out/vehicle_attitude", self._on_att, sd)

    def _on_status(self, m):
        self.status = m

    def _on_att(self, m):
        w, x, y, z = m.q
        self.yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def spin_for(self, s):
        end = time.monotonic() + s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def send_setpoint(self, speed, yaw, rate):
        t = now_us()
        ocm = P.OffboardControlMode()
        ocm.timestamp = t
        ocm.velocity = True  # same as px4_link: the stock flag enabling speed + attitude + rate together
        self.p_ocm.publish(ocm)
        ts = P.TrajectorySetpoint()
        ts.timestamp = t
        ts.position = [NAN] * 3
        ts.velocity = [NAN] * 3
        ts.acceleration = [NAN] * 3
        ts.jerk = [NAN] * 3
        ts.yaw = NAN
        ts.yawspeed = NAN
        self.p_traj.publish(ts)
        rs = P.RoverSpeedSetpoint()
        rs.timestamp = t
        rs.speed_body_x = float(speed)
        rs.speed_body_y = NAN
        self.p_speed.publish(rs)
        ra = P.RoverAttitudeSetpoint()
        ra.timestamp = t
        ra.yaw_setpoint = float(yaw)
        self.p_att.publish(ra)
        rr = P.RoverRateSetpoint()
        rr.timestamp = t
        rr.yaw_rate_setpoint = float(rate)
        self.p_rate.publish(rr)

    def command(self, cmd, p1, p2=0.0):
        c = P.VehicleCommand()
        c.timestamp = now_us()
        c.command = cmd
        c.param1 = float(p1)
        c.param2 = float(p2)
        c.target_system = 1
        c.target_component = 1
        c.source_system = 1
        c.source_component = 1
        c.from_external = True
        self.p_cmd.publish(c)

    def stream(self, s, speed=0.0, yaw=NAN, rate=0.0, record=False, require_offboard=True):
        """Stream one setpoint at 50 Hz for s seconds. Returns False if PX4 left OFFBOARD or disarmed."""
        end = time.monotonic() + s
        nxt = time.monotonic()
        while time.monotonic() < end:
            if time.monotonic() >= nxt:
                self.send_setpoint(speed, yaw, rate)
                nxt += 0.02
                if record:
                    self.samples.append((time.monotonic(), self.yaw))
            rclpy.spin_once(self, timeout_sec=0.002)
            st = self.status
            if require_offboard and st is not None and (st.nav_state != NAV_OFFBOARD or st.arming_state != ARMED):
                return False
        return True


def yaw_rate(samples, skip_s=0.6):
    """Mean yaw rate (rad/s, + = clockwise) over the step, skipping the first skip_s of spin-up."""
    use = [s for s in samples if s[1] is not None and s[0] - samples[0][0] >= skip_s] if samples else []
    if len(use) < 2:
        return None
    d = math.atan2(math.sin(use[-1][1] - use[0][1]), math.cos(use[-1][1] - use[0][1]))
    return d / (use[-1][0] - use[0][0])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speed", type=float, default=0.2, help="forward speed for the speed step, m/s (cap 0.5)")
    ap.add_argument("--rate", type=float, default=0.3, help="yaw rate for the rate steps, rad/s (cap 0.6)")
    ap.add_argument("--step", type=float, default=2.0, help="seconds per motion step (cap 4)")
    ap.add_argument("--wheels-up", action="store_true", help="judge by encoders only (gyro does not move)")
    ap.add_argument("--heading", action="store_true", help="also turn to current yaw + 30 deg (wheels down)")
    ap.add_argument("--loss-test", action="store_true",
                    help="end by driving +speed and STOPPING the stream: expect PX4 to disarm (COM_OF_LOSS_T)")
    a = ap.parse_args()
    speed = max(-MAX_SPEED, min(MAX_SPEED, a.speed))
    rate = max(-MAX_RATE, min(MAX_RATE, a.rate))
    step = max(0.5, min(MAX_STEP_S, a.step))

    rclpy.init()
    n = Bench()
    results = []
    try:
        n.spin_for(2.0)
        others = n.count_publishers("/fmu/in/offboard_control_mode") - 1
        if others > 0:
            print("ABORT: %d other publisher(s) on /fmu/in/offboard_control_mode. Stop dyx3-ros first." % others)
            return 2
        st = n.status
        if st is None or n.yaw is None:
            print("ABORT: no vehicle_status / attitude from PX4 (DDS session down?)")
            return 2
        if st.arming_state != ARMED:
            print("ABORT: not armed. Arm in Manual with RC/QGC (kill switch in hand), then run again.")
            return 2
        print("armed, nav_state=%d, yaw=%.1f deg. Prestreaming STOP..." % (st.nav_state, math.degrees(n.yaw)))
        n.stream(1.0, require_offboard=False)
        n.command(CMD_DO_SET_MODE, 1.0, 6.0)
        t = time.monotonic()
        while time.monotonic() - t < 2.5 and (n.status.nav_state != NAV_OFFBOARD):
            n.stream(0.1, require_offboard=False)
        if n.status.nav_state != NAV_OFFBOARD:
            print("ABORT: PX4 did not enter OFFBOARD (nav_state=%d)" % n.status.nav_state)
            n.stream(0.5, require_offboard=False)
            return 3
        print("OFFBOARD entered.")

        def motion(name, spd, yaw, rt, check):
            if not n.stream(1.0):
                return False
            n.samples = []
            utc0 = time.strftime("%H:%M:%S", time.gmtime())
            ok = n.stream(step, spd, yaw, rt, record=True)
            utc1 = time.strftime("%H:%M:%S", time.gmtime())
            r = yaw_rate(n.samples)
            if a.wheels_up:
                verdict = None  # judged by eye + ULog wheel_encoders
            else:
                verdict = bool(ok and r is not None and check(r))
            results.append((name, r, verdict, ok))
            print("%-28s UTC %s-%s  yaw rate %s  -> %s" % (
                name, utc0, utc1, "%+.3f rad/s" % r if r is not None else "n/a",
                "CHECK LOG/EYE" if verdict is None else ("PASS" if verdict else "FAIL")))
            return ok

        steps = [
            ("+speed %.2f m/s (forward)" % speed, speed, NAN, 0.0, lambda r: abs(r) < 0.1),
            ("+yaw rate %.2f rad/s (CW)" % rate, 0.0, NAN, rate, lambda r: r > 0.5 * rate),
            ("-yaw rate %.2f rad/s (CCW)" % rate, 0.0, NAN, -rate, lambda r: r < -0.5 * rate),
        ]
        for name, spd, yaw, rt, chk in steps:
            if not motion(name, spd, yaw, rt, chk):
                print("STOPPED: PX4 left OFFBOARD or disarmed (operator / failsafe). Ending.")
                break
        else:
            if a.heading and n.yaw is not None:
                target = math.atan2(math.sin(n.yaw + math.radians(30)), math.cos(n.yaw + math.radians(30)))
                y0 = n.yaw
                n.stream(1.0)
                n.samples = []
                n.stream(min(MAX_STEP_S, 4.0), 0.0, target, NAN, record=True)
                err = math.degrees(math.atan2(math.sin(target - n.yaw), math.cos(target - n.yaw)))
                turned = math.degrees(math.atan2(math.sin(n.yaw - y0), math.cos(n.yaw - y0)))
                v = abs(err) < 5.0 and turned > 0
                results.append(("heading +30 deg", None, v, True))
                print("heading +30 deg: turned %+.1f deg, final error %+.1f deg -> %s" % (turned, err, "PASS" if v else "FAIL"))
            if a.loss_test:
                n.stream(1.0)
                n.stream(1.0, speed, NAN, 0.0)
                print("LOSS TEST: stopping all setpoints now while moving at %.2f m/s" % speed)
                t_cut = time.monotonic()
                t_dis = None
                while time.monotonic() - t_cut < 3.0:
                    rclpy.spin_once(n, timeout_sec=0.005)
                    if n.status.arming_state != ARMED:
                        t_dis = time.monotonic() - t_cut
                        break
                v = t_dis is not None and t_dis < 1.0
                results.append(("loss -> disarm", None, v, True))
                print("loss -> disarm after %s -> %s" % ("%.2f s" % t_dis if t_dis else "NOT within 3 s", "PASS" if v else "FAIL"))
                return 0 if all(r[2] is not False for r in results) else 1
        n.stream(1.0, require_offboard=False)
    except KeyboardInterrupt:
        print("Interrupted: streaming STOP")
        n.stream(0.6, require_offboard=False)
    finally:
        if n.status is not None and n.status.arming_state == ARMED:
            n.stream(0.3, require_offboard=False)
            n.command(CMD_ARM_DISARM, 0.0)
            n.spin_for(0.5)
            print("disarm sent")
        print("SUMMARY: " + ", ".join("%s=%s" % (r[0], "CHECK" if r[2] is None else ("PASS" if r[2] else "FAIL")) for r in results))
        n.destroy_node()
        rclpy.shutdown()
    return 0 if results and all(r[2] is not False for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())

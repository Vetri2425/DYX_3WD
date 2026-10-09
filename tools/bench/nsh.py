import sys, time
from pymavlink import mavutil
m = mavutil.mavlink_connection(sys.argv[1], baud=57600)
m.wait_heartbeat(timeout=15)
def run(cmd, wait=2.0):
    data = (cmd + "\n").encode()
    m.mav.serial_control_send(10, 2 | 4, 0, 0, len(data), list(data) + [0] * (70 - len(data)))
    out, end = b"", time.time() + wait
    while time.time() < end:
        msg = m.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.3)
        if msg and msg.count:
            out += bytes(msg.data[:msg.count])
        m.mav.serial_control_send(10, 2 | 4, 0, 0, 0, [0] * 70)
    return out.decode(errors="replace")
for c in sys.argv[2:]:
    print(f"===== {c}"); print(run(c, 3.0))

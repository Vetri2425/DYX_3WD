import socket, json
s = socket.socket(socket.AF_UNIX); s.settimeout(3); s.connect("/run/dyx3/rtk-control.sock")
s.sendall(b'{"v":1,"cmd":"GET_STATUS"}\n'); d = b""
while not d.endswith(b"\n"): d += s.recv(65536)
r = json.loads(d)["data"]
t = r["transport"]
print(r["worker_state"], "delivered", t["frames_delivered"], "failures", t["failures"], "valid", r["source"]["valid_frames"])

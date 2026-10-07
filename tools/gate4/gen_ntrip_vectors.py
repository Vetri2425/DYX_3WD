#!/usr/bin/env python3
"""Equivalence vectors for dyx3_gnss_rtk from the prototype's own NTRIP code (PX4_DXP/ntrip_rtcm_node.py).

    tools/gate4/gen_ntrip_vectors.py --dxp <PX4_DXP checkout root>

The prototype file is NOT carried into this repository (read-only evidence); the generator records its sha256
in the fixture so the provenance is checkable. Run in the Humble container (the module imports rclpy,
mavros_msgs and sensor_msgs). Output: ros2_ws/src/dyx3_gnss_rtk/test/fixtures/ntrip_vectors.txt

Format
  SRC <sha256 of ntrip_rtcm_node.py>
  STREAM <nchunks> <nframes>          then nchunks `C <hex>` (delivery order) and nframes `F <hex>` (expected), END
  CRC <hex data> -> <crc24q decimal>
  CHK <body> -> <2 hex digits>
  GGA <lat> <lon> <alt> <quality-status 0|1|2> <cov0> <stamp_sec> -> <sentence | NONE>
"""
import argparse
import hashlib
import os
import random
import sys
import time
from types import SimpleNamespace

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
OUT = os.path.join(REPO, "ros2_ws", "src", "dyx3_gnss_rtk", "test", "fixtures", "ntrip_vectors.txt")
SEED = 20261009


def r(x):
    return repr(float(x))


def make_frame(rng, payload_len, reserved=0):
    import ntrip_rtcm_node as m

    f = bytearray([0xD3, ((payload_len >> 8) & 3) | reserved, payload_len & 0xFF])
    f += bytes(rng.randrange(256) for _ in range(payload_len))
    crc = m._rtcm3_crc(bytes(f), len(f))
    f += bytes([(crc >> 16) & 0xFF, (crc >> 8) & 0xFF, crc & 0xFF])
    return bytes(f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dxp", required=True)
    a = ap.parse_args()
    src = os.path.join(a.dxp, "ntrip_rtcm_node.py")
    sys.path.insert(0, a.dxp)
    import ntrip_rtcm_node as m

    rng = random.Random(SEED)
    L = ["NTRIPVEC 1", "# from the prototype ntrip_rtcm_node.py; see tools/gate4/gen_ntrip_vectors.py",
         "SRC " + hashlib.sha256(open(src, "rb").read()).hexdigest()]

    for _ in range(400):
        n = rng.randrange(0, 1100)
        data = bytes(rng.randrange(256) for _ in range(n))
        L.append(f"CRC {data.hex() or '-'} -> {m._rtcm3_crc(data, len(data))}")
    for _ in range(200):
        body = "".join(chr(rng.randrange(32, 127)) for _ in range(rng.randrange(1, 80)))
        L.append(f"CHK {body.replace(' ', '~')} -> {m._nmea_checksum(body)}")
    # CHK bodies with a space are encoded with '~' above; make that explicit: spaces are not generated at all.
    L = [ln for ln in L if not (ln.startswith("CHK") and "~" in ln)]

    for _ in range(300):
        stream = bytearray()
        for _k in range(rng.randrange(1, 8)):
            kind = rng.random()
            if kind < 0.55:
                stream += make_frame(rng, rng.choice([0, 1, 5, 40, 120, 300, 800, 1023, rng.randrange(0, 1024)]))
            elif kind < 0.7:
                stream += make_frame(rng, rng.randrange(1, 200), reserved=rng.choice([0x04, 0x40, 0xFC]))
            elif kind < 0.85:
                f = bytearray(make_frame(rng, rng.randrange(8, 200)))
                f[rng.randrange(3, len(f))] ^= 1 << rng.randrange(8)
                stream += f
            else:
                stream += bytes(rng.randrange(256) for _ in range(rng.randrange(1, 40)))
        if rng.random() < 0.3:
            stream += make_frame(rng, 200)[: rng.randrange(1, 100)]  # truncated tail
        # deliver in random chunks exactly as the prototype loop does
        chunks = []
        i = 0
        while i < len(stream):
            k = rng.choice([1, 2, 7, 64, 256, 4096])
            chunks.append(bytes(stream[i:i + k]))
            i += k
        buf = b""
        expected = []
        for ch in chunks:
            buf += ch
            frames, buf = m.NtripNode._parse_rtcm_frames(buf, None)
            expected += frames
        L.append(f"STREAM {len(chunks)} {len(expected)}")
        L += ["C " + (c.hex() or "-") for c in chunks]
        L += ["F " + e.hex() for e in expected]
        L.append("END")

    node = object.__new__(m.NtripNode)
    import threading

    node._gps_lock = threading.Lock()
    for _ in range(600):
        lat = rng.choice([rng.uniform(-90, 90), 0.0, -0.0001, 89.9999, 12.97166667, -33.8688])
        lon = rng.choice([rng.uniform(-180, 180), 0.0, -0.5, 179.9999, 77.59666, 151.2093])
        alt = rng.uniform(-100, 3000)
        st = rng.choice([0, 1, 2, -1])
        cov = rng.choice([0.0001, 0.0099, 0.01, 0.5])
        stamp = rng.choice([0, 1700000000, 1700000059, 1234567890, rng.randrange(1, 2_000_000_000)])
        fix = SimpleNamespace(latitude=lat, longitude=lon, altitude=alt, status=SimpleNamespace(status=st),
                              position_covariance=[cov] + [0.0] * 8, header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp)))
        node._gps_fix = fix
        node._gps_fix_recv_monotonic = time.monotonic()
        s = node._format_gga()
        L.append(f"GGA {r(lat)} {r(lon)} {r(alt)} {st} {r(cov)} {stamp} -> {'NONE' if s is None else s.strip().replace(chr(13), '')}")
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as f:
        f.write("\n".join(L) + "\n")
    print(f"wrote {OUT}: {len(L)} lines")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Identify what the UM982 sends on a USB serial port. Bench tool; stdlib only (termios).

Usage (on the Jetson, as root; the RTK worker must NOT have the port open):
  sudo python3 um982_usb_probe.py /dev/serial/by-path/<...>            passive listen at each baud
  sudo python3 um982_usb_probe.py /dev/serial/by-path/<...> --query 115200
        send the read-only queries CONFIG and UNILOGLIST at that baud and print the replies

Never sends anything that changes receiver state: no CONFIG <args>, no LOG/UNLOG, no SAVECONFIG.
"""
import os
import select
import sys
import termios
import time

BAUDS = [115200, 230400, 460800, 921600, 9600, 19200, 38400, 57600]
READ_ONLY_QUERIES = [b"CONFIG\r\n", b"UNILOGLIST\r\n"]
SPEED = {b: getattr(termios, f"B{b}") for b in BAUDS}


def open_port(path, baud):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0                                            # iflag: raw
    attrs[1] = 0                                            # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag: 8N1
    attrs[3] = 0                                            # lflag
    attrs[4] = attrs[5] = SPEED[baud]
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def read_for(fd, seconds):
    data, end = b"", time.time() + seconds
    while time.time() < end:
        r, _, _ = select.select([fd], [], [], 0.1)
        if r:
            try:
                data += os.read(fd, 4096)
            except BlockingIOError:
                pass
    return data


def nmea_ok(line):
    if not line.startswith("$") or "*" not in line:
        return False
    body, _, cs = line[1:].partition("*")
    calc = 0
    for ch in body.encode(errors="replace"):
        calc ^= ch
    return cs[:2].upper() == f"{calc:02X}"


def summarize(data):
    lines = [l.strip() for l in data.decode(errors="replace").splitlines() if l.strip()]
    valid = [l for l in lines if nmea_ok(l)]
    kinds = {}
    for l in valid:
        k = l[1:].split(",", 1)[0]
        kinds[k] = kinds.get(k, 0) + 1
    return len(data), len(valid), kinds, lines


def main():
    path = sys.argv[1]
    if len(sys.argv) >= 4 and sys.argv[2] == "--query":
        baud = int(sys.argv[3])
        fd = open_port(path, baud)
        for q in READ_ONLY_QUERIES:
            os.write(fd, q)
            nbytes, nvalid, kinds, lines = summarize(read_for(fd, 2.5))
            print(f"=== {q.decode().strip()} @ {baud}: {nbytes} B, {nvalid} valid sentences")
            for l in lines:
                if not l.startswith(("$GN", "$GP", "$GB", "$GA", "$GL")):  # skip periodic NMEA noise
                    print("   ", l[:200])
        os.close(fd)
        return
    for baud in BAUDS:
        fd = open_port(path, baud)
        nbytes, nvalid, kinds, lines = summarize(read_for(fd, 2.5))
        os.close(fd)
        print(f"{baud:>7}: {nbytes:6d} B, {nvalid:4d} valid NMEA  {kinds}")
        for l in [l for l in lines if nmea_ok(l)][:2]:
            print("          ", l[:120])


if __name__ == "__main__":
    main()

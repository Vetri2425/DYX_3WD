#!/usr/bin/env python3
"""Write message-hash vectors by running the FIRMWARE's own hash code.

  gen_hash_vectors.py --helper <fw>/Tools/msg/px_generate_uorb_topic_helper.py \
      --msg-dir <px4_msgs>/msg [--name Foo ...] > vectors.txt

The firmware helper imports ROS1 genmsg for parsing only; genmsg_shim/ provides that parsing and the
helper's hash functions run unmodified. Output: one `<MessageName> <hash decimal>` per line, sorted.
"""
import argparse
import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "genmsg_shim"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--helper", required=True)
    ap.add_argument("--msg-dir", required=True)
    ap.add_argument("--name", action="append", default=[])
    a = ap.parse_args()

    spec = importlib.util.spec_from_file_location("px_helper", a.helper)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    from genmsg import msg_loader

    search = {"px4": [a.msg_dir]}
    names = a.name or sorted(f[:-4] for f in os.listdir(a.msg_dir) if f.endswith(".msg"))
    for n in names:
        with open(os.path.join(a.msg_dir, n + ".msg")) as f:
            fields = msg_loader.parse(f.read(), "px4")
        print(n, helper.get_message_hash(fields, search))


if __name__ == "__main__":
    main()

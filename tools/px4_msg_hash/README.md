# px4_msg_hash — test vectors from the firmware's own hash code

`dyx3_px4_link` refuses to publish setpoints until the firmware's message-format hash for every
topic it uses equals the hash computed from the installed `px4_msgs` definitions (architecture 4.6).
The C++ implementation is `message_hash.cpp`; this directory proves it against the **firmware's own
Python function**, not against a re-derivation of the algorithm.

`gen_hash_vectors.py` imports `Tools/msg/px_generate_uorb_topic_helper.py` from the pinned firmware
unmodified. That file needs ROS1 `genmsg` for parsing only; `genmsg_shim/` supplies just the parsing
(fields in file order, constants excluded, bare nested types resolved to `<pkg>/<Type>`). The hash
functions that run are the firmware's.

```bash
# firmware at the pinned SHA (installer/pins/firmware.pin), px4_msgs msg dir from the same SHA
python3 tools/px4_msg_hash/gen_hash_vectors.py \
  --helper <firmware>/Tools/msg/px_generate_uorb_topic_helper.py \
  --msg-dir <px4_msgs>/msg > ros2_ws/src/dyx3_px4_link/test/fixtures/px4_msg_hash_vectors.txt
```

Regenerate when `FIRMWARE_SHA` changes. The unit test compares the C++ hash against the vectors for
the messages carried in `test/fixtures/msgdefs/` (including nested `EscStatus`,
`PositionSetpointTriplet`, `ArmingCheckReply`), and, when `DYX3_PX4_MSGS_SRC=<px4_msgs>/msg` is set,
against all 235 messages of the pinned set.

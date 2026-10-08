# Proposal: RTK status interface for ROS safety and recorder consumers

**Not implemented in this branch.** `dyx3_interfaces` is Claude-owned and frozen. The
RTK worker exposes the full model through its local v1 control socket and backend REST API.
The existing `RtkStatus`, `NtripStatus` and `GnssReport` messages remain compatible.

At the next coordinated interface version, add these fields to `RtkStatus.msg` so recorder,
motion guard and spray can distinguish selected source and delivered corrections without
consulting FastAPI:

```msg
uint8 SOURCE_NTRIP=1
uint8 SOURCE_LORA=2
uint8 source
uint8 TRANSPORT_USB_DIRECT=1
uint8 TRANSPORT_PX4_DDS=2
uint8 transport
uint8 STATE_STOPPED=0
uint8 STATE_STARTING=1
uint8 STATE_WAIT_SOURCE=2
uint8 STATE_WAIT_RTCM=3
uint8 STATE_WAIT_TRANSPORT=4
uint8 STATE_INJECTING=5
uint8 STATE_DEGRADED=6
uint8 STATE_RECONFIGURING=7
uint8 STATE_ERROR=8
uint8 worker_state
uint64 config_revision
bool source_valid_rtcm_age_valid
float32 source_valid_rtcm_age_s
bool transport_delivery_age_valid
float32 transport_delivery_age_s
bool receiver_correction_age_valid
float32 receiver_correction_age_s
uint64 source_valid_frames
uint64 transport_delivered_frames
```

`GnssReport.msg` also needs an authoritative correction-age field if PX4_DDS mode is to
report receiver age without USB GGA readback:

```msg
bool receiver_correction_age_valid
float32 receiver_correction_age_s
```

No receiver age is inferred from source or delivery age. The existing px4_link contract
and frozen interface version must be reviewed before these fields are landed.

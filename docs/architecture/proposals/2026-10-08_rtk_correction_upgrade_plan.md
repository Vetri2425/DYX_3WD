# Consolidated RTK correction (injection) upgrade plan

**Status:** PLAN ONLY — no production-code, configuration, receiver-setting, or test change is
authorized by this document.  This plan is to be executed after the already-merged
`claude/cloud-phases` work.  It is deliberately a 3WD plan: the 4WD implementation is evidence
and design reference, never a drop-in port.

**Decision baseline.** The 3WD production topology is Jetson LTE WAN for NTRIP and Ethernet
uXRCE-DDS to the Pixhawk; it has no router/site LAN in that path
([V1 §4.3–4.4](../DYX_3WD_Production_Stack_Architecture_V1.md#L348-L415)).  3WD uses a UM982;
the 4WD reference uses MAVROS and a Septentrio mosaic-H, including a USB2-direct correction path
([4WD low-satellite report](../../../../../4WD_Proto/rover_ws/docs/2026-10-03_RTK_Low_Satellite_Base_Rover_Report.md#L5-L14)).
Consequently, nothing below authorizes MAVROS, Septentrio commands, or 4WD receiver configuration
on 3WD.

The human decision of 2026-10-06 supplied with this task is binding: source (NTRIP or LoRa) is
independent from transport; the preferred transport is Jetson-to-UM982 direct USB, with PX4/DDS
as fallback, and never both concurrently.  Its approximately five-second failover and ten-second
failback values are **pending human confirmation**, not newly invented acceptance limits.

## Evidence and current position

The target has the *structural* foundations, but not an implemented correction worker:

- `dyx3-rtk` is already a systemd sibling of the backend, restarts itself, and is ordered after
  platform service ([deployment/systemd/dyx3-rtk.service:1-16](../../../deployment/systemd/dyx3-rtk.service#L1-L16));
  its launcher is explicitly a skeleton that exits 1
  ([deployment/scripts/start-rtk.sh:1-5](../../../deployment/scripts/start-rtk.sh#L1-L5)).
- `dyx3_gnss_rtk` declares parser, transport, NTRIP, GGA, health, and node modules, each still a
  no-op stub ([ros2_ws/src/dyx3_gnss_rtk/src/rtk_node.cpp:1-4](../../../ros2_ws/src/dyx3_gnss_rtk/src/rtk_node.cpp#L1-L4);
  [rtcm_parser.cpp:1-4](../../../ros2_ws/src/dyx3_gnss_rtk/src/rtcm_parser.cpp#L1-L4)).
  The current motion RTK gate is likewise a stub
  ([ros2_ws/src/dyx3_motion_guard/src/rtk_gate.cpp:1-4](../../../ros2_ws/src/dyx3_motion_guard/src/rtk_gate.cpp#L1-L4)).
- The first frozen `RtkStatus` has only receiver fix/freshness/age/accuracy/satellites fields
  ([ros2_ws/src/dyx3_interfaces/msg/RtkStatus.msg:1-13](../../../ros2_ws/src/dyx3_interfaces/msg/RtkStatus.msg#L1-L13));
  its ABI test establishes zero/unknown as the safe default
  ([interface_abi_test.cpp:159-172](../../../ros2_ws/src/dyx3_interfaces/test/interface_abi_test.cpp#L159-L172)).
- Firmware commit `07741c2f26` exposes multi-instance `/fmu/in/gps_inject_data` and documents
  caller-side fragmentation over 300 bytes
  ([dds_topics.yaml:228-233](../../../../PX4-Autopilot-3WD-Prod/src/modules/uxrce_dds_client/dds_topics.yaml#L228-L233)).
  `GpsInjectData` is a 300-byte payload, fragment-LSB message
  ([GpsInjectData.msg:5-9](../../../../PX4-Autopilot-3WD-Prod/msg/GpsInjectData.msg#L5-L9)); the GPS
  driver writes injected bytes to its GPS UART ([gps.cpp:618-635](../../../../PX4-Autopilot-3WD-Prod/src/drivers/gps/gps.cpp#L618-L635)).
- UM982 configuration is receiver-persistent and must never be performed by PX4/companion;
  correction bytes are explicitly data, not configuration
  ([GNSS_receiver_configuration.md:6-19](../../contracts/GNSS_receiver_configuration.md#L6-L19)).
  The required receiver-produced messages/rates are GPGGA, UNIAGRICA, UNIHEADINGA at 5 Hz and
  GPGST, GPGSA, GPRMC at 1 Hz ([GNSS_receiver_configuration.md:34-48](../../contracts/GNSS_receiver_configuration.md#L34-L48)).

## Comparison matrix and decision record

“Quality” below is an assessment, not a claim of field validation.  “Field failure” only records
evidence found in the reviewed sources.  `NEW` means a 3WD-specific composition or contract is
required.  For every row: **3WD-prod** is “architectural/skeleton” unless implementation is cited;
**3WD-proto** is field-proven vehicle/protocol lineage except where this plan explicitly labels a
commit “never field-run”; **4WD-proto** is the most complete implementation reference but has
receiver/MAVROS portability risk.  “No row-specific field failure located” means exactly that —
not that the behaviour is field-safe.  The two demonstrated failures that apply across the matrix
are the 3WD deploy-to-FLOAT process coupling ([V1:505-526](../DYX_3WD_Production_Stack_Architecture_V1.md#L505-L526))
and 4WD’s stable-but-wrong re-fix despite apparently good receiver metrics
([2026-09-09 handoff:19-33](../../../../../4WD_Proto/rover_ws/docs/2026-09-09_Handoff_RTK_Wrong_Ambiguity_Fix_Root_Cause.md#L19-L33)).

| Concern | 3WD production today | 3WD prototype | 4WD prototype reference | Decision / reason |
|---|---|---|---|---|
| Sources | P8 names NTRIP only; no implementation ([Phase_plan.md:376-400](../../agents/Phase_plan.md#L376-L400)). | NTRIP and LoRa workers exist; LoRa reads serial RTCM and reconnects ([lora_rtcm_node.py:176-250](../../../../../3WD_Proto/PX4_DXP/lora_rtcm_node.py#L176-L250)). | Profile schema supports `NTRIP` and `LORA` ([rtk_profile_store.py:729-751](../../../../../4WD_Proto/rover_ws/src/rover_backend/rover_backend/rtk_profile_store.py#L729-L751)). | **NEW:** retain both source adapters but make either feed one common validated-frame pipeline.  Whether LoRa ships is open. |
| NTRIP client | Stubs only. | Basic auth, ICY/HTTP 200 check, initial + periodic GGA, socket-timeout reconnect ([ntrip_rtcm_node.py:350-411](../../../../../3WD_Proto/PX4_DXP/ntrip_rtcm_node.py#L350-L411); [ntrip_protocol.py:9-34](../../../../../3WD_Proto/PX4_DXP/ntrip_protocol.py#L9-L34)).  Quality: useful field lineage, but failures are broadly `ConnectionError`. | Separates permanent authentication/mountpoint rejections from retryable failures ([ntrip_failures.py:6-76](../../../../../4WD_Proto/rover_ws/src/rtk_correction_bridge/rtk_correction_bridge/ntrip_failures.py#L6-L76)); pure GGA generation is separately tested. | **ADOPT-FROM-4WD**, preserving 3WD-proto’s immediate/periodic GGA behaviour.  Classify DNS/connect/TLS/read/GGA/stream-stall as retryable; auth/mountpoint/profile validation as non-retrying configuration faults. |
| RTCM framing/validation | No implementation. Firmware demands 300-byte fragmentation only on the DDS leg. | CRC-24Q and scan-after-bad-preamble are implemented, but reserved header bits are accepted ([ntrip_rtcm_node.py:424-478](../../../../../3WD_Proto/PX4_DXP/ntrip_rtcm_node.py#L424-L478)). | Incremental RTCM3 parser validates 0xD3, reserved bits, 10-bit length, CRC-24Q and resynchronizes without discarding the whole buffer ([rtcm3_parser.py:1-16](../../../../../4WD_Proto/rover_ws/src/rtk_correction_bridge/rtk_correction_bridge/rtcm3_parser.py#L1-L16); [rtcm3_parser.py:163-174](../../../../../4WD_Proto/rover_ws/src/rtk_correction_bridge/rtk_correction_bridge/rtcm3_parser.py#L163-L174)). | **ADOPT-FROM-4WD:** receiver-independent parser, counter model, partial-frame timeout and resync.  **NEW:** 3WD DDS fragmenter must preserve each validated frame and set the PX4 fragment LSB exactly as required. |
| Transport | Ethernet/DDS is production companion-to-PX4; direct UM982 was an option in V1 ([V1:546-574](../DYX_3WD_Production_Stack_Architecture_V1.md#L546-L574); [V1:612-627](../DYX_3WD_Production_Stack_Architecture_V1.md#L612-L627)). | MAVROS-only publishing, so not portable. | Has MAVROS/PX4 and direct serial modes ([status_snapshot.py:24-40](../../../../../4WD_Proto/rover_ws/src/rtk_correction_bridge/rtk_correction_bridge/status_snapshot.py#L24-L40)). | **NEW:** USB direct is PRIMARY; `/fmu/in/gps_inject_data` through `dyx3_px4_link` is FALLBACK.  **NOT** MAVROS. |
| Single-source / single-transport ownership | No worker. | Manager serializes one NTRIP/LoRa child at a time (documented in its class contract) ([rtk_manager.py:1-18](../../../../../3WD_Proto/PX4_DXP/server/rtk_manager.py#L1-L18)). | Explicit injection lock module and dedicated direct-routing tests are present ([injection_lock.py:1-4](../../../../../4WD_Proto/rover_ws/src/rtk_correction_bridge/rtk_correction_bridge/injection_lock.py#L1-L4); [test_direct_injection_routing.py](../../../../../4WD_Proto/rover_ws/src/rtk_correction_bridge/test/test_direct_injection_routing.py)). | **ADOPT-FROM-4WD concept, NEW implementation:** one source lease and one transport lease, owned only by `dyx3-rtk`; prohibit parallel USB/PX4 writes. |
| Failover/failback | None. | Source reconnect only; LoRa backs off `min(5*2^attempt,60)` ([lora_rtcm_node.py:245-250](../../../../../3WD_Proto/PX4_DXP/lora_rtcm_node.py#L245-L250)). | Direct serial is selectable but the reviewed reference is profile/policy oriented, not the 3WD autonomous transport failover specified by the human decision. | **NEW:** explicit primary/fallback state machine below.  Use the supplied approximately 5 s / 10 s values as pending decisions, never as “proven” values. |
| Process ownership | Correct sibling unit, but launcher does not exist. | Backend owns a child; V1 records deploy → silent FLOAT/no autostart ([V1:505-526](../DYX_3WD_Production_Stack_Architecture_V1.md#L505-L526)). | Backend process orchestration is comprehensive, but still a backend ownership design. | **KEEP-3WD-PROD:** systemd sibling owns worker lifetime; backend may request desired state only. |
| Profiles / credentials | V1 requires `/etc/dyx3`, `root:dyx3 0640`, never Git ([V1:1143-1173](../DYX_3WD_Production_Stack_Architecture_V1.md#L1143-L1173)). | Named profiles are 0600, password write-only, autostart on backend restart ([RTK_NTRIP_OPERATIONS.md:3-37](../../../../../3WD_Proto/PX4_DXP/docs/RTK_NTRIP_OPERATIONS.md#L3-L37)). | SQLite schema contains a stored `password_secret` and transport parameters ([rtk_profile_store.py:501-557](../../../../../4WD_Proto/rover_ws/src/rover_backend/rover_backend/rtk_profile_store.py#L501-L557)). | **KEEP-3WD-PROD secret location + ADOPT 4WD public/redacted profile shape.** Do not put NTRIP passwords into backend DB, Socket.IO, ROS params, logs, or tablet payloads. |
| Autostart / restart | Unit has `Restart=always`/5 s but current executable exits ([dyx3-rtk.service:10-16](../../../deployment/systemd/dyx3-rtk.service#L10-L16); [start-rtk.sh:1-5](../../../deployment/scripts/start-rtk.sh#L1-L5)). | Autostarts from `rover-server`; bounded retry only after child exit ([RTK_NTRIP_OPERATIONS.md:34-37](../../../../../3WD_Proto/PX4_DXP/docs/RTK_NTRIP_OPERATIONS.md#L34-L37)). | Lifecycle/orchestrator tests cover backend managed process behaviour (test inventory under `rover_backend/test/test_rtk_*`). | **KEEP systemd restart, NEW worker retry policy:** avoid restart storms and surface terminal configuration faults. |
| Health domains | `RtkStatus` mixes receiver freshness/solution fields only. | Status records child link/frame age, while GPSRAW gate is separate ([ntrip_rtcm_node.py:198-221](../../../../../3WD_Proto/PX4_DXP/ntrip_rtcm_node.py#L198-L221); [rtk_quality.py:30-100](../../../../../3WD_Proto/PX4_DXP/src/rtk_quality.py#L30-L100)). | Status snapshot separates source, transport and direct serial data ([status_snapshot.py:16-40](../../../../../4WD_Proto/rover_ws/src/rtk_correction_bridge/rtk_correction_bridge/status_snapshot.py#L16-L40)). | **ADOPT-FROM-4WD shape, NEW 3WD receiver inputs:** never equate socket bytes, topic presence, RTK FLOAT/FIXED, or receiver correction age. |
| Drive / spray gates | Required but unimplemented (P6 motion guard, P9 spray RTK gate) ([Phase_plan.md:408-427](../../agents/Phase_plan.md#L408-L427)). | 42d8d4b implements GPSRAW freshness ≤0.5 s, only fix 6, known h_acc ≤0.10 m and 1 s recovery; unknown h_acc blocks spray ([rtk_quality.py:30-100](../../../../../3WD_Proto/PX4_DXP/src/rtk_quality.py#L30-L100); [test_spray_rtk_gate.py:173-225](../../../../../3WD_Proto/PX4_DXP/src/test_spray_rtk_gate.py#L173-L225)).  Per task direction these commits were never field-run. | 4WD field analysis finds that a re-fix may be decimetres wrong despite `eph`; it recommends invalidating marking reference on FIX re-acquisition ([2026-09-09 handoff:135-153](../../../../../4WD_Proto/rover_ws/docs/2026-09-09_Handoff_RTK_Wrong_Ambiguity_Fix_Root_Cause.md#L135-L153)). | **ADOPT-FROM-3WD-PROTO as bench-only initial policy; ADOPT-FROM-4WD the re-fix warning/reference-validation concept; NEW 3WD contracts.** |
| Observability / recorder | V1 requires link/fix/age published, recorded and gated ([V1:523-526](../DYX_3WD_Production_Stack_Architecture_V1.md#L523-L526)). | JSON status has mode/state/frames/bytes/age/error ([ntrip_rtcm_node.py:198-221](../../../../../3WD_Proto/PX4_DXP/ntrip_rtcm_node.py#L198-L221)). | Parser/transport counters distinguish valid, CRC-invalid, resync, oversize, publish errors ([rtcm_transport.py:60-87](../../../../../4WD_Proto/rover_ws/src/rtk_correction_bridge/rtk_correction_bridge/rtcm_transport.py#L60-L87)). | **ADOPT-FROM-4WD counters + NEW recorder evidence event schema.** |
| Tablet/backend contract | Backend owns no ROS/motion authority ([Phase_plan.md:551-570](../../agents/Phase_plan.md#L551-L570)). | REST status/profile routes redact password; operations says tablet cannot persist it ([RTK_NTRIP_OPERATIONS.md:3-18](../../../../../3WD_Proto/PX4_DXP/docs/RTK_NTRIP_OPERATIONS.md#L3-L18)). | Frontend type files and backend contract tests exist ([DYX_GCS_Frontend/src/types/rtk.ts](../../../../../4WD_Proto/DYX_GCS_Frontend/src/types/rtk.ts); [test_rtk_gga_profile_contract.py](../../../../../4WD_Proto/rover_ws/src/rover_backend/test/test_rtk_gga_profile_contract.py)). | **ADOPT-FROM-4WD shape; NEW service-control boundary.** Backend is observer/configurer, never a safety or process parent. |
| Tests | ABI only; P8 remains stubs. | Protocol, quality, profile, manager and route tests exist; the task says 42d8d4b/98e951a were never field-run. | Strongest unit/contract suite: parser, NTRIP, GGA, serial, lock, route, process and logging tests (listed under `rtk_correction_bridge/test` and `rover_backend/test`). | **ADOPT-FROM-4WD test decomposition, ADOPT 3WD proto gate fixtures, NEW replay/capture tests and 3WD hardware gates.** |

Known field evidence must constrain the plan: 4WD proves that healthy correction transport cannot
alone prove a correct solution (one report recorded all message types, zero CRC errors and
0.5–0.8 s latency while diagnosing a separate satellite problem
[2026-10-03 report:22-29](../../../../../4WD_Proto/rover_ws/docs/2026-10-03_RTK_Low_Satellite_Base_Rover_Report.md#L22-L29)).
It also records a solution that can be stable-but-wrong across a re-fix despite small `eph`
([2026-09-09 handoff:19-33](../../../../../4WD_Proto/rover_ws/docs/2026-09-09_Handoff_RTK_Wrong_Ambiguity_Fix_Root_Cause.md#L19-L33)).
Thus no gate in 3WD may claim that correction bytes or h_acc proves marking truth.

## Consolidated 3WD target architecture

### Authority and graph

```text
NTRIP over LTE WAN ─┐
                    ├─ source lease ─> RTCM3 validator ─> transport lease ─┬─ USB PRIMARY ─> UM982 chosen COM
LoRa serial ────────┘                                                        │      └─ same-port GGA readback
                                                                             └─ DDS FALLBACK ─> dyx3_px4_link
                                                                                     └─ /fmu/in/gps_inject_data ─> PX4 GPS UART ─> UM982

UM982 GGA/readback ─┐                         ┌─ PX4 SensorGps fix_type/h_acc/satellites
stream/transport ───┼─> dyx3_gnss_rtk ──────┼─> RtkStatus + RTK event stream ─> motion_guard, spray, recorder
PX4 receiver state ─┘                         └─ backend REST/Socket.IO projection ─> tablet
```

`dyx3-rtk` is the systemd sibling worker required by V1, not a child process of backend
([V1:1143-1157](../DYX_3WD_Production_Stack_Architecture_V1.md#L1143-L1157)).  It alone owns source
and transport leases, serial handles, retransmission/retry policy, and publication of factual
RTK status.  `dyx3_motion_guard` and `dyx3_spray` are the safety authorities for their own stop/
spray decisions.  Backend can persist a *desired* selected source/profile and send a versioned
control request, but can neither start an unowned child nor override a failed gate.

**Source layer.** NTRIP uses LTE only; it authenticates, validates status response, sends GGA from
the selected receiver-readback source when required by the profile, preserves handshake bytes,
and reports classified failures.  LoRa, if approved, is a local serial source using the same
RTCM3 parser.  One source lease makes switching a stop-then-start transaction; counters preserve
lifetime totals and separately identify the current connection/session.

**Frame layer.** Accept only CRC-24Q-valid RTCM3 frames after the 0xD3/10-bit-length parser.
Record preamble/noise discard, bad-header, CRC, partial-timeout, valid frame and byte counters.
Do not impose the PX4 300-byte limit on USB: it is a *DDS* message payload constraint, whereas
RTCM3 itself can be up to 1029 bytes in the 4WD parser
([rtcm3_parser.py:7-16](../../../../../4WD_Proto/rover_ws/src/rtk_correction_bridge/rtk_correction_bridge/rtcm3_parser.py#L7-L16)).
For DDS, fragment a validated frame over 300-byte `GpsInjectData` messages, preserve ordering,
and set the LSB fragment flag.  The firmware contract explicitly assigns that responsibility to
the sender ([F-tasks.md:242-251](../../Firmware/F-tasks.md#L242-L251)).

**Transport layer.** USB direct writes only to a stable `/dev/serial/by-id/...` symlink for the
UM982 COM port designated for Jetson correction ingress and reads GGA from that *same* port.
The Pixhawk UART and USB transport must be different UM982 COM ports: the supplied C-RTK 2HP
wiring warning says USB/COM2/UART2 may starve PX4.  This is a hardware fact to prove with the
`gps status` USB-plug test, not a software assumption.  The fallback goes through
`dyx3_px4_link` and the existing DDS injection topic, never MAVROS.

No transport configures receiver output, baud, logs, constellation, or `SAVECONFIG`.  The
production ownership contract expressly disallows it, including on reconnect/heading loss
([GNSS_receiver_configuration.md:59-75](../../contracts/GNSS_receiver_configuration.md#L59-L75)).

### Transport state machine

States are named so every change is a recorder/tablet event with monotonic timestamp, reason,
old/new source, old/new transport, and relevant age/error counters:

1. `STOPPED` — no requested source, no write handle.
2. `SOURCE_CONNECTING` — selected source lease is establishing; no correction claim.
3. `USB_PRIMARY_PROBING` — open by-id port and establish same-port GGA readback; no parallel
   PX4 injection.
4. `USB_PRIMARY_ACTIVE` — validated frames write successfully and readback is live; the only
   active transport.
5. `PX4_FALLBACK_ARMING` — close USB writer first, then enable DDS fragmentation/publish path.
6. `PX4_FALLBACK_ACTIVE` — only DDS writes; it is validated by FCU `gps status` and solution
   transition, never merely ROS topic presence ([V1:566-574](../DYX_3WD_Production_Stack_Architecture_V1.md#L566-L574)).
7. `USB_FAILBACK_PROBING` — while fallback remains the sole writer, open/read USB and accumulate
   the approved healthy dwell; do not write USB yet.
8. `SWITCHING_TO_USB` — atomically stop/flush DDS ownership, then grant USB writer lease.
9. `DEGRADED_NO_TRANSPORT` — no eligible transport; publish fail-closed status and continue
   source observation/recovery.
10. `FAULT_LATCHED` — terminal profile/auth/mountpoint/configuration fault; operator correction
    is required, but existing safety consumers remain informed.

Transition reasons include `USB_DEVICE_LOST`, `USB_WRITE_ERROR`, `SOURCE_STALLED`,
`CORRECTION_AGE_EXCEEDED`, `PX4_INJECTION_UNVERIFIED`, `AUTH_REJECTED`, `MOUNTPOINT_REJECTED`,
`OPERATOR_SOURCE_SWITCH`, `AGENT_RESTART`, and `HEALTHY_DWELL_COMPLETE`.  Per the human decision,
fail over on USB device loss/write error or correction age **greater than approximately 5 s while
the source is live**, and fail back only after **approximately 10 s healthy**.  These values must
be confirmed at the P8 design gate; put the accepted values and their decision record in the
contract, not hidden defaults.

### Three independent health domains

`RtkStatus` must represent all three; no Boolean may silently stand for all of them.

| Domain | Evidence | Healthy means | Does not prove |
|---|---|---|---|
| Stream | source state; socket/serial bytes; valid frame age/rate; CRC/header/resync counters; NTRIP/GGA error class | selected source is delivering recent CRC-valid frames | transport delivery or receiver FIX |
| Transport | active transport; USB open/write/readback/error/last-write; DDS fragments/publish result; `gps status` evidence for fallback | the *one* active path is plausibly delivering corrections | receiver solution correctness |
| Receiver solution | USB same-port GGA quality and correction age; PX4 `SensorGps` fix_type/h_acc/satellites; explicit FIX transitions | current solution inputs meet the phase’s gate and no required data is unknown | absolute surveyed truth after reacquisition |

The software must validate DDS fallback by observed `gps status` correction reception plus a
FIX-type transition, because firmware evidence warns that topic presence can be silently useless
([F-tasks.md:242-251](../../Firmware/F-tasks.md#L242-L251)).  A direct USB success likewise
requires successful writes *and* same-port GGA evidence, not a successfully opened device.

### Gate consumers

`dyx3_motion_guard` consumes an atomic `RtkGateInput` projection from `RtkStatus` plus PX4
receiver state; it outputs a fail-closed drive decision/reason. `dyx3_spray` consumes the same
projection independently and defaults OFF on any unavailable, stale, FLOAT, unknown-accuracy,
unknown-correction-age, or re-fix-unacknowledged condition.  Neither accepts a backend “healthy”
claim.  The P8 worker reports facts; P6/P9 own policy.

The initial bench contract may reproduce the 3WD-prototype values — GPSRAW age ≤0.5 s, only
fix 6, known h_acc ≤0.10 m, one continuous second recovery; h_acc=0 is unknown and fails closed
([RTK_NTRIP_OPERATIONS.md:47-57](../../../../../3WD_Proto/PX4_DXP/docs/RTK_NTRIP_OPERATIONS.md#L47-L57)).
**DERIVED — NOT FROM V1 SPEC: those 42d8d4b values are a bench-validation starting point only;
the task records that they were never field-run.** They must not be presented as a production
accuracy proof.

`FIXED → FLOAT → FIXED` is a stateful safety event: halt marking at loss; on reacquisition,
latch `reference_revalidation_required` until the approved surveyed-reference procedure completes.
This is a conservative 3WD adoption of 4WD’s measured re-fix hazard, not an assertion that UM982
behaves identically. **DERIVED — NOT FROM V1 SPEC.** Drive-resume policy needs human approval:
either use the same latch or permit controlled non-marking repositioning while spray remains
latched off.

### Contracts to write before code

Create `docs/contracts/rtk_*.md` as **DRAFT** only if implementation starts; freeze interfaces
before their implementation.  The minimum contract set is:

1. **`RtkStatus` v2 (ROS).** Retain existing fields/values for compatibility and add:
   `source`, `source_state`, `transport`, `transport_state`, `active_profile_id` (non-secret),
   `stream_healthy`, `transport_healthy`, `receiver_healthy`, `receiver_gga_quality`,
   `receiver_correction_age_s`, `px4_fix_type`, `px4_h_acc_m`, `valid_frame_age_s`, frame/byte
   totals, CRC/header/resync/fragment/write/publish error totals, `last_transition_reason`,
   `reference_revalidation_required`, and separate monotonic ages/timestamps.  Use enums, not
   unbounded error strings; a bounded sanitized diagnostic code is tablet-safe.
2. **ROS control/status surface.** Define a request/response control service for selecting
   source/profile and explicit stop, a read-only status topic, and a transition-event topic.
   The service may request only policy-compliant changes; worker returns applied/queued/rejected
   and never credentials.  Topic QoS, clock, ordering, restart sequence, safe defaults, and
   one-writer rules are part of the contract.
3. **Backend REST + Socket.IO projection.** Reuse the 4WD separation of public profile metadata,
   desired/active state and runtime status shape, but expose only redacted
   `password_configured`.  REST owns profile CRUD/desired selection; Socket.IO is a live
   read-only `rtk_status`/`rtk_transition` mirror. Tablet payload must show source, transport,
   correction age, solution fix/accuracy, alarm/reason and each switch.  It never carries NTRIP
   password, receiver command, ROS handle, or safety override.
4. **Recorder evidence.** Record raw `RtkStatus`, all transitions/reasons, selected profile ID
   + revision (not secret), source/transport counters, `SensorGps` and GGA-derived evidence,
   FCU `gps status` validation record, software/firmware hashes, physical port identity, and
   gate decisions.  This meets V1’s requirement that loss is a vehicle state and runs have
   provenance ([V1:1205-1218](../DYX_3WD_Production_Stack_Architecture_V1.md#L1205-L1218)).
5. **Installer/privilege contract.** Installer creates the `dyx3-rtk` unit, `dyx3` membership in
   `dialout`, a VID/PID/serial-specific udev rule resolving to `/dev/serial/by-id/...`, and a
   preflight that rejects absent/ambiguous device identity. `ntrip.env` remains `/etc/dyx3`,
   `root:dyx3`, 0640; UI writes are mediated and redacted.  No secret appears in Git or unit
   command line.

## Phased implementation plan and gates

Every phase is one variable at a time, has a contract-first artifact, and replays/captures evidence
before a rover run, consistent with V1 non-negotiables ([V1:1205-1227](../DYX_3WD_Production_Stack_Architecture_V1.md#L1205-L1227)).

| Phase / owner | Deliverables and contract-first work | Evidence tests and acceptance gate | Rollback |
|---|---|---|---|
| **P8 GNSS/RTK** | Draft/freeze parser, source, health, state-machine and `RtkStatus` contracts; implement source-independent parser, NTRIP/optional-LoRa adapters, USB writer/GGA reader, one-writer leases, status/event output. | Unit replay of recorded 3WD RTCM captures; receiver-independent 4WD RTCM parser captures; malformed preamble/length/CRC/partial frames; credential redaction; source switch; state-machine table tests. Bench gate: valid CRC counters and USB readback match capture expectation; no receiver configuration bytes. | Disable `dyx3-rtk`; do not enable fallback until P7 proves it. Preserve capture/evidence. |
| **P7 `dyx3_px4_link` fallback** | Freeze DDS fragmentation and fallback health contract; add an RTK transport adapter only through `/fmu/in/gps_inject_data`. | Fragment boundaries 1/300/301/max recorded frame, flags/order/error paths; agent restart; FCU observation. Acceptance: `gps status` reports corrections **and** a controlled FIX transition occurs; topic presence alone fails the gate ([F-tasks.md:280-285](../../Firmware/F-tasks.md#L280-L285)). | Revoke DDS transport lease; return to USB primary. Do not restore MAVROS. |
| **P6 / P9 gates** | Freeze shared `RtkGateInput`, drive decision and spray reason contracts; implement separately in motion guard and spray. | Replay FIX→FLOAT→FIX, stale GPS, h_acc zero, h_acc over the starting 0.10 m, source live/transport lost, correction-age alarm, service/backend restart; assert zero/valve-off and reason every time. Acceptance: no unreasoned resume; prototype thresholds are bench-validated before approval. | Keep mission disabled / valve-off policy; no threshold relaxation in field. |
| **P10 recorder + gateway** | Freeze event/evidence schema and Socket.IO projection ownership; recorder captures counters, transitions, FCU evidence and gate decisions. | Rebuild a run report from bags/captures and verify monotonic sequence, no dropped switch event, redacted secrets, hashes/profile revision present. Acceptance: a reviewer can determine source, transport, age, FIX, switch and gate reason from one run folder. | Continue local journal plus bags; do not delete raw evidence. |
| **P2 backend** | Freeze REST/Socket.IO OpenAPI/JSON schema, public profile shape, optimistic revision/desired-state control semantics. | Contract tests derived from 4WD profile/GGA/logging/process tests; tablet receives switches and alarms; password never appears in JSON, logs or WebSocket. Acceptance: backend restart/deploy neither stops worker nor changes a safe gate decision. | Backend may be down; sibling RTK/recorder remain active. Revert API only, not the worker. |
| **P11 installer / udev** | Unit/env-file, udev by-id, dialout, health/preflight, release/rollback instructions. | Fresh-install and upgrade rehearsal; unplug/replug identity; incorrect/duplicate device refusal; permission audit; service restart; release rollback. Acceptance: `dyx3-health` distinguishes stream/transport/receiver and reports missing device/secret safely. | `dyx3-rollback` to known-good release; leave UM982 persistent configuration untouched. |

P8 is intentionally first in the named implementation stream, while the production Phase plan
orders P7 → P6 → P8 → P9 → P10 → P2 → P11
([Phase_plan.md:551-570](../../agents/Phase_plan.md#L551-L570)).  **DERIVED — NOT FROM V1 SPEC:**
perform P8’s contracts/parser/capture work first, but do not make the fallback authoritative until
P7, nor claim drive/spray readiness until P6/P9.  This respects both dependency directions without
silently changing authority.

## Bench and field validation protocol

No test below authorizes a production mission until its acceptance evidence is attached to the
run and reviewed.

1. **COM-port map / non-interference.** Record UM982 saved configuration, all physical cable
   endpoints, `by-id` identifier, port baud, and PX4-facing COM. Plug/unplug USB while checking
   FCU `gps status`; verify PX4 GNSS remains alive, correction path is observed only on the
   selected port, and the Jetson’s GGA readback is from that port. Serial-capture PX4→UM982 after
   boot/heading-loss: it may contain RTCM but no receiver configuration commands
   ([GNSS_receiver_configuration.md:70-75](../../contracts/GNSS_receiver_configuration.md#L70-L75)).
2. **Nominal acquisition and re-fix.** From cold/non-correction → FIXED, deliberately interrupt
   correction → FLOAT/non-fixed, restore → FIXED. Record time to loss/recovery, GGA quality,
   receiver correction age, PX4 fix/h_acc/satellites, source/transport counters and every gate
   transition. At each re-fix test the reference-revalidation latch; do not infer position truth
   from h_acc alone.
3. **Transport failover/failback.** With source live, pull USB; then force write error; restore
   USB. Confirm exactly one writer at all times, emitted reason, fallback FCU `gps status` plus
   FIX transition, approved failover/failback dwell, and no receiver reconfiguration. Record
   source frame age at each state change and all data gaps.
4. **Source/network faults.** Stop NTRIP/caster, remove LTE WAN, simulate DNS/connect/auth/
   mountpoint failure, then restore. Confirm failure classification, bounded retry behaviour,
   terminal-fault handling, alarm, and fail-closed gate. If LoRa is approved, repeat for LoRa
   disconnect/noise and source switching.
5. **Data-integrity faults.** Inject bad CRC, reserved-header/length corruption, noise before a
   valid frame, split frame, stale partial frame, and oversize/fragment-boundary captures. Record
   CRC/resync/partial/fragment counters and prove the next valid frame is accepted.
6. **Lifecycle faults.** Restart `dyx3-rtk`, backend, recorder and XRCE agent independently;
   simulate deployment/backend loss; power-cycle UM982/PX4 as approved. Confirm RTK worker is not
   killed by backend restart, no duplicate writer exists, correct post-restart state is visible,
   and correction continuity/reacquisition is explicitly classified.
7. **Field soak and accuracy evidence.** Record a parked surveyed-point run before/after every
   transport-variable change, with raw/fused GNSS, correction stream, satellite/constellation
   data where available, weather/antenna environment, base/caster identity and exact topology.
   A 4WD incident shows a nearby person can cause FIXED→FLOAT through antenna carrier loss
   ([2026-10-03 report:16-29](../../../../../4WD_Proto/rover_ws/docs/2026-10-03_RTK_Low_Satellite_Base_Rover_Report.md#L16-L29)); repeat obstructed/open-sky observations on 3WD rather than transfer a threshold.

Numbers to record rather than invent: source byte/frame rate and age; valid/CRC/header/resync/
partial/fragment/write/publish errors; GGA/PX4 correction age; time spent per state; switch count
and reason; `gps status` receipt evidence; FIX/h_acc/satellite/GGA quality sequences; gate stop/
resume and spray transitions; position at fixed surveyed point across re-fixes; serial device/
port identity; release/firmware/parameter/profile revisions.

## Risks and questions requiring a human decision

1. Which exact UM982 COM is physically wired to Pixhawk on this 3WD module, and which different
   COM can USB/direct transport use without starving it? Approve the COM-map acceptance record.
2. Confirm LTE dongle model, stable Linux/by-id behaviour, SIM/APN/reconnect ownership, and
   whether NTRIP is permitted only over that WAN. V1 assigns LTE bring-up/link health to platform
   ([V1:379-381](../DYX_3WD_Production_Stack_Architecture_V1.md#L379-L381)).
3. Is LoRa a production source or a bench/site option? If production, identify radio, port,
   source-selection UX and its credential/configuration owner.
4. Confirm the supplied approximately 5 s failover and 10 s failback criteria, and separately
   the correction-age/gate thresholds. Do not inherit the prototype 0.5 s/0.10 m/1 s values
   without their bench evidence and explicit production decision.
5. Define the surveyed reference-revalidation procedure and whether non-marking repositioning may
   proceed after re-fix. This changes marking safety behaviour materially.
6. Decide the production profile store: immutable system-managed `/etc/dyx3/ntrip.env` only, or
   mediated editable profiles whose secrets are written only to root-owned files. Tablet-side
   credential persistence is forbidden either way.
7. Resolve the outstanding GPS-driver submodule violation: today’s `request_unicore_messages()`
   can configure UM982 on heading loss ([GNSS_receiver_configuration.md:21-32](../../contracts/GNSS_receiver_configuration.md#L21-L32)).
   No correction deployment should mask that independent production blocker.

## Deliberately not ported

- **MAVROS and `/mavros/gps_rtk/send_rtcm`.** They are 3WD-incompatible; the production link is
  Ethernet uXRCE-DDS and firmware already exposes the dedicated injection topic.
- **Septentrio/mosaic-H commands, SBF status assumptions and its COM3/USB2 mapping.** These are
  receiver-specific 4WD implementation details, not UM982 evidence.
- **4WD’s backend-as-worker-parent process topology.** It conflicts with the 3WD sibling-service
  non-negotiable and reintroduces the known deploy-to-FLOAT failure.
- **Direct copy of 4WD numeric defaults** such as its development MAVROS frame gate.  The 4WD
  transport explicitly distinguishes a 720-byte development gate from the 1029-byte protocol
  ceiling ([rtcm_transport.py:1-13](../../../../../4WD_Proto/rover_ws/src/rtk_correction_bridge/rtk_correction_bridge/rtcm_transport.py#L1-L13)); 3WD uses firmware’s 300-byte fragment payload instead.
- **3WD prototype’s backend child supervisor, raw JSON status file as the authority, and
  `rover-server` autostart coupling.** Preserve its protocol/quality lessons, not its lifetime
  structure.
- **Automatic receiver configuration/recovery.** It violates the explicit UM982 ownership rule.
- **Any claim that FIXED/h_acc or fresh RTCM proves correct absolute placement.** 4WD field
  evidence refutes that general inference; 3WD must record and revalidate its own solution.

## Completion definition

The plan is complete when the above contracts are reviewed and frozen, all listed bench evidence
is recorded, USB primary and DDS fallback are separately validated using `gps status` plus a FIX
transition, gates fail closed with auditable reasons, a backend deploy/restart cannot stop RTK,
and the human has answered the seven questions.  Until then, “RTK topic publishes” is not an
acceptance criterion.

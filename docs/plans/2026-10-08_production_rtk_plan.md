> **Repository copy, 2026-10-08.** Plan authored by the human in chat (reviewed by Sonnet); committed verbatim below as the
> input for fix-plan item E4. Frozen human decision recorded the same day (`docs/reviews/2026-10-08_cloud_review_verification_and_fix_plan.md` section 5):
> NTRIP profile security is explicit, `security = PLAINTEXT | TLS`, required for every new profile; PLAINTEXT is supported (port-2101 casters)
> with a visible security warning in UI/status; TLS verifies certificates by default and never silently downgrades; security is never
> inferred from the port number. This refines the "TLS policy" line of section 6.1.

# DYX 3WD Production RTK Correction Upgrade Plan

**Status:** Planned — implement only after `claude/cloud-phases` is complete, reviewed, and merged to `master`  
**Primary package:** `ros2_ws/src/dyx3_gnss_rtk`  
**Related packages:** `dyx3_px4_link`, `dyx3_motion_guard`, `dyx3_spray`, `dyx3_recorder`, `dyx3_system_gateway`, `backend/`, `installer/`, tablet frontend  
**Production principle:** RTK correction delivery is an independent vehicle service. Backend and frontend configure and observe it; they do not keep it alive.

---

## 1. Objective

Build a production RTK correction subsystem for the DYX 3WD rover with these properties:

- correction source is selectable and persistent:
  - `NTRIP`
  - `LORA`
- correction transport is selectable and persistent:
  - `USB_DIRECT`
  - `PX4_DDS`
- source selection and transport selection are independent
- exactly one correction source is active
- exactly one correction transport is active
- RTK correction delivery survives unrelated process failures
- `dyx3-rtk` runs as its own systemd service
- backend/FastAPI/Socket.IO are control and telemetry surfaces only
- the tablet exposes full RTK state, counters, source, transport, receiver solution, and faults
- PX4/companion never configures the UM982 receiver
- receiver configuration remains in UM982 persistent memory
- no automatic dual injection
- no stale RTCM replay
- no hidden dependency on mission, RPP, spray, backend, or tablet connectivity

This subsystem becomes the reference implementation for later porting the RTK control/status/frontend model into 4WD production.

---

## 2. Starting point

After the cloud session finishes:

1. Freeze and record the final `claude/cloud-phases` head.
2. Review and merge the branch into `master`.
3. Treat the merged tree as authoritative.
4. Do not start RTK implementation against the current pre-merge `master`.
5. Preserve useful Phase 8 work already implemented:
   - NTRIP client
   - RTCM3 framing
   - CRC-24Q validation
   - stream resynchronization
   - RTCM chunking
   - GGA generation/back-feed
   - correction health
   - status counters
   - existing tests
6. Replace the current DDS-only ownership/transport architecture with the production design in this document.

Current cloud Phase 8 is useful implementation evidence, but its correction path is not yet the final 3WD production architecture.

---

## 3. Hard architecture rules

### 3.1 RTK execution authority

`dyx3-rtk.service` is the only runtime owner of RTK correction execution.

It owns:

- persisted desired RTK state
- active correction source
- active transport
- source reconnects
- RTCM parsing and validation
- injection ownership
- transport retry/recovery
- receiver-solution verification
- correction health
- RTK runtime state
- RTK counters and events

The backend does **not**:

- spawn the RTK worker
- reap the RTK worker
- restart the RTK worker on backend lifecycle events
- own RTCM injection
- own source sockets
- own receiver USB
- own correction freshness
- decide whether RTK stays alive

Backend shutdown or redeploy must not interrupt a running RTK stream.

### 3.2 Receiver ownership

The UM982 owns its production configuration in persistent memory.

The companion/PX4 may:

- consume GNSS data
- consume heading data
- inject RTCM

They must not:

- reconfigure output message rates
- reconfigure constellations
- overwrite persistent receiver settings
- send automatic configuration sequences

### 3.3 Source and transport are independent

Valid combinations:

| Source | Transport |
|---|---|
| NTRIP | USB_DIRECT |
| NTRIP | PX4_DDS |
| LORA | USB_DIRECT |
| LORA | PX4_DDS |

Do not encode source-specific transport fields such as "LoRa direct inject" as the long-term architecture.

### 3.4 Single active injection authority

At all times:

`active_transport ∈ {NONE, USB_DIRECT, PX4_DDS}`

Never:

`USB_DIRECT + PX4_DDS`

Transport switch sequence:

1. stop accepting new frames for the old sink
2. close/release old sink
3. prove old sink inactive
4. acquire new sink
5. begin forwarding only fresh RTCM frames

No overlapping injection window is allowed.

### 3.5 No stale RTCM replay

RTCM is time-sensitive.

If:

- source reconnects
- transport is unavailable
- USB is unplugged
- DDS disappears
- service restarts

do not buffer seconds of RTCM and inject it later.

Old correction frames are discarded.

Injection resumes only with fresh incoming RTCM.

---

## 4. Target architecture

```text
                         PERSISTENT CONFIG
                               |
                  +------------+------------+
                  |                         |
             source=NTRIP              source=LORA
                  |                         |
          NTRIP client/socket         LoRa serial reader
                  |                         |
                  +------------+------------+
                               |
                     raw RTCM byte stream
                               |
                    RTCM3 parser / CRC24Q
                   framing / resync / stats
                               |
                       CRC-valid RTCM3
                               |
                   SINGLE INJECTION AUTHORITY
                               |
                     selected transport
                     /                 \
                    /                   \
          USB_DIRECT                     PX4_DDS
              |                             |
      UM982 serial port             /fmu/in/gps_inject_data
              |                             |
              +----------- UM982 -----------+
                               |
                   receiver solution/status
                               |
          FIX / FLOAT / h_acc / sats / correction age
                               |
             status + safety + recorder + frontend
```

---

## 5. Startup behavior

### 5.1 Boot sequence

On rover boot:

1. systemd starts `dyx3-rtk`
2. worker loads persisted RTK configuration
3. validate schema and revision
4. read persisted `desired_state`
5. if `STOPPED`:
   - remain idle
   - publish status
6. if `RUNNING`:
   - validate selected source config
   - validate selected transport config
   - initialize source
   - wait for valid RTCM
   - initialize selected transport
   - begin injection
   - monitor receiver response
   - continue recovery/reconnect autonomously

### 5.2 Do not wait for RTK FIX before injection

Injection readiness must not depend on receiver already being RTK fixed.

Expected progression may be:

`3D -> RTK FLOAT -> RTK FIXED`

A degraded receiver may also become:

`RTK FIXED -> RTK FLOAT`

RTCM injection must continue during FLOAT.

For DDS transport, injection readiness depends on DDS/PX4 transport availability, not RTK fix type.

---

## 6. Correction source layer

All correction sources normalize into the same internal interface:

```text
Source
  -> raw bytes
  -> RTCM3 parser
  -> RTCM3Frame
```

### 6.1 NTRIP

Reuse the strongest behavior from current 3WD Phase 8 and 4WD prototype:

- persistent profile
- host
- port
- mountpoint
- username
- write-only password
- TLS policy
- connect timeout
- read timeout
- reconnect backoff
- caster response validation
- immediate GGA when required
- periodic GGA
- GGA freshness validation
- reconnect after half-open/stale stream
- counters
- no credentials in logs/status

### 6.2 LoRa

Reference 4WD `feat/rtk-lora-source` behavior:

- stable serial device
- baud
- 8N1
- exclusive open
- bounded read timeout
- read-error accounting
- close/reopen on error
- reconnect delay
- raw RTCM bytes unchanged
- feed the same RTCM parser as NTRIP

LoRa does not get its own RTCM interpretation path.

---

## 7. RTCM parser and validation

Use one ROS-free parser shared by all sources.

Required behavior:

- preamble `0xD3`
- 10-bit payload length
- maximum RTCM3 protocol frame length
- CRC-24Q
- incremental parsing
- split-frame reconstruction
- multiple frames per read
- invalid-header detection
- invalid-CRC detection
- one-byte resync from rejected preamble
- bounded residual buffer
- partial-frame timeout
- message type extraction
- deterministic counters

Required counters:

- bytes received
- valid frames
- valid bytes
- CRC failures
- invalid headers
- resync bytes
- partial-frame timeouts

Protocol validity and transport validity remain separate concepts.

---

## 8. Transport layer

Internal transport abstraction:

```text
TransportSink:
    readiness()
    open()
    deliver(frame)
    close()
    status()
```

### 8.1 USB_DIRECT

USB direct is fully independent from PX4 and DDS.

Use stable `/dev/serial/by-id/...` paths only.

Do not use:

- `/dev/ttyACM0`
- enumeration-order assumptions

Required behavior:

- validate configured device identity
- exclusive open
- byte-exact frame write
- partial write continuation
- no replay from byte zero after failed partial frame
- write timeout
- close on hard error
- reopen delay
- self-heal after unplug/replug
- frames-written counter
- bytes-written counter
- write failure counter
- open failure counter
- reopen counter
- last successful write age

The same owner may read receiver output when the UM982 port supports duplex operation.

### 8.2 PX4_DDS

DDS transport injects RTCM through:

`/fmu/in/gps_inject_data`

Production goal:

- RTK correction injection must not depend on mission
- must not depend on RPP
- must not depend on spray
- must not depend on backend
- must not depend on tablet
- should not be interrupted by unrelated PX4-link application logic

Preferred ownership:

`dyx3-rtk` owns only the RTCM DDS input endpoint.

`dyx3_px4_link` retains ownership of:

- motion setpoints
- arm/disarm
- OFFBOARD
- vehicle state
- normal PX4 command/state interfaces

If the current "only px4_link may touch /fmu/**" rule remains, this point must be explicitly revised before implementation because otherwise a `dyx3_px4_link` crash interrupts DDS correction injection.

DDS transport readiness must validate:

- XRCE/DDS availability
- compatible `px4_msgs`
- `GpsInjectData` path availability
- publisher state
- delivery/publish failures

Do not use RTK fix type as transport readiness.

---

## 9. Initial transport policy

First production implementation:

- configured transport only
- no automatic USB<->DDS failover

Reason:

- deterministic field behavior
- simpler safety argument
- easier validation
- no accidental dual injection
- no silent route switching

Example:

If configured `USB_DIRECT` and USB disappears:

- status -> DEGRADED
- stop successful-delivery freshness
- retry USB
- do not switch to DDS automatically

If configured `PX4_DDS` and DDS disappears:

- status -> DEGRADED
- retry DDS
- do not switch to USB automatically

An `AUTO` failover mode may be added later only after independent validation.

---

## 10. RTK worker state machine

Recommended top-level states:

- `STOPPED`
- `STARTING`
- `WAIT_SOURCE`
- `WAIT_RTCM`
- `WAIT_TRANSPORT`
- `INJECTING`
- `DEGRADED`
- `RECONFIGURING`
- `ERROR`

State must be visible in:

- worker status
- recorder
- backend REST
- WebSocket telemetry
- tablet RTK screen

Every state transition should carry:

- timestamp
- previous state
- new state
- reason code
- human-readable reason
- configuration revision

---

## 11. Source health, transport health, receiver health

Do not collapse RTK into one boolean.

### 11.1 Source health

Examples:

- NTRIP connected
- LoRa port open
- bytes arriving
- CRC-valid RTCM arriving
- valid-frame age
- RTCM rate
- source reconnect count

### 11.2 Transport health

Examples:

- USB device resolved
- USB port open
- USB frames/bytes written
- DDS available
- DDS chunks published
- delivery errors
- last successful delivery age

### 11.3 Receiver solution health

Examples:

- fix type
- RTK FLOAT
- RTK FIXED
- h_acc
- v_acc
- satellites used
- HDOP
- receiver correction age where authoritative
- FIX transition count
- last FIX transition

Hard distinction:

`source connected`
!=
`valid RTCM received`
!=
`RTCM delivered`
!=
`receiver using corrections`
!=
`RTK FIXED`

---

## 12. Correction age semantics

Do not use one ambiguous `correction_age`.

Use explicit concepts.

Recommended fields:

- `source_valid_rtcm_age_s`
- `transport_delivery_age_s`
- `receiver_correction_age_s`

Meaning:

### `source_valid_rtcm_age_s`

Time since last CRC-valid RTCM frame arrived from NTRIP/LoRa.

### `transport_delivery_age_s`

Time since last RTCM frame was successfully delivered to the selected transport.

### `receiver_correction_age_s`

Age reported or inferred from an authoritative receiver/FCU source.

If receiver correction age is unavailable, report:

- `receiver_correction_age_valid = false`

Do not substitute source age.

---

## 13. Persistent configuration

Recommended high-level schema:

```text
desired_state = RUNNING | STOPPED

source = NTRIP | LORA
transport = USB_DIRECT | PX4_DDS

ntrip:
    active_profile_id
    profiles[]

lora:
    serial_device
    baud

usb:
    receiver_device
    baud
    write_timeout
    reopen_delay

dds:
    expected_firmware_sha
    expected_message_contract

revision
updated_at
```

Secrets:

- live under `/etc/dyx3`
- owner `root:dyx3`
- mode `0640`
- never stored in Git
- never emitted to tablet
- never emitted to logs
- never emitted to WebSocket
- never returned by profile APIs

---

## 14. Transactional configuration update

Configuration is applied by the RTK service, not by editing runtime state from FastAPI.

Flow:

```text
Tablet
 -> FastAPI
 -> local RTK control interface
 -> dyx3-rtk validates candidate
```

If invalid:

- reject
- current injection remains untouched

If valid:

1. validate complete candidate
2. prepare new source/transport
3. quiesce current forwarding
4. close/release old sink
5. atomically persist new configuration
6. activate new source/transport
7. resume injection using fresh RTCM
8. publish new configuration revision

If activation fails:

- report failure
- preserve known-good persistent config where possible
- never allow two active sinks

---

## 15. Backend control boundary

Backend is an authenticated client of `dyx3-rtk`.

Preferred local interface:

`/run/dyx3/rtk-control.sock`

Potential commands:

- `GET_STATUS`
- `GET_CONFIG`
- `SET_CONFIG`
- `START`
- `STOP`
- `LIST_PROFILES`
- `CREATE_PROFILE`
- `UPDATE_PROFILE`
- `DELETE_PROFILE`
- `TEST_SOURCE`
- `LIST_SERIAL_PORTS`

Backend failure only removes operator access temporarily.

The RTK worker continues.

---

## 16. Systemd ownership

Dedicated unit:

`dyx3-rtk.service`

Requirements:

- `Restart=always`
- independent from backend
- no `Requires=dyx3-backend.service`
- no backend child process
- survives backend restart/redeploy
- survives tablet disconnect
- retries missing LTE/network
- retries missing LoRa device
- retries missing USB receiver
- retries DDS loss
- persists desired RUNNING/STOPPED state

Systemd supervises the process.

The process supervises RTK state.

FastAPI does neither.

---

## 17. Backend REST API

One canonical RTK API surface.

Recommended endpoints:

- `GET /api/rtk/status`
- `GET /api/rtk/config`
- `PUT /api/rtk/config`
- `POST /api/rtk/start`
- `POST /api/rtk/stop`
- `GET /api/rtk/source`
- `PUT /api/rtk/source`
- `GET /api/rtk/transport`
- `PUT /api/rtk/transport`
- `GET /api/rtk/profiles`
- `POST /api/rtk/profiles`
- `PATCH /api/rtk/profiles/{id}`
- `DELETE /api/rtk/profiles/{id}`
- `GET /api/rtk/serial-ports`

Do not create a second simplified `/api/rtk/status`.

The fully assembled FastAPI application must have an integration test proving that exactly one RTK status route is registered.

---

## 18. WebSocket telemetry

WebSocket is for live status delivery.

It must not be needed for correction delivery.

Recommended RTK telemetry object:

```text
rtk:
    overall
    source
    transport
    receiver
    counters
    events
    config_revision
```

The exact shape must be versioned and shared by:

- backend serializer
- WebSocket payload
- frontend TypeScript types
- tests

---

## 19. Tablet RTK tab

The tablet RTK tab is a first-class observability surface.

It must answer immediately:

1. Which source is selected?
2. Is valid RTCM arriving?
3. Which transport is selected?
4. Is RTCM actually being delivered?
5. What solution is the receiver producing?

### 19.1 Overall section

Show:

- worker state
- overall healthy/unhealthy
- desired RUNNING/STOPPED
- configuration revision
- worker uptime
- last state change
- last state-change reason
- last error

Example:

`RTK FIXED · NTRIP -> USB · Healthy`

### 19.2 Source section

Show:

- selected source `NTRIP|LORA`
- connected
- source state
- source age
- bytes received
- RTCM frames received
- CRC-valid frames
- CRC failures
- invalid headers
- resync bytes
- partial timeouts
- RTCM rate
- last valid-frame age
- reconnect count

NTRIP also shows:

- profile name
- caster host
- port
- mountpoint
- TLS mode
- GGA state
- last GGA age
- GGA count

Never show:

- password
- Authorization header

LoRa also shows:

- radio device
- baud
- port open
- bytes received
- read errors
- reopen count

### 19.3 Transport section

Show:

- selected transport `USB_DIRECT|PX4_DDS`
- state
- ready
- frames delivered
- bytes delivered
- last successful delivery age
- delivery failures
- reconnect/reopen count

USB additionally:

- stable `/dev/serial/by-id/...`
- device detected
- port open
- baud
- receiver readback state

DDS additionally:

- XRCE/DDS state
- message-contract compatibility
- `GpsInjectData` ready
- frames/chunks sent
- publish errors

### 19.4 Receiver solution section

Show:

- fix type name
- fix type numeric value
- `NO FIX`
- `2D`
- `3D`
- `DGPS`
- `RTK FLOAT`
- `RTK FIXED`
- horizontal accuracy
- vertical accuracy
- satellites used
- HDOP
- receiver correction age
- receiver status freshness
- FIX transition count
- last transition

### 19.5 Event history

Show recent events such as:

- service started
- service restarted
- source connected
- source disconnected
- NTRIP reconnect
- NTRIP auth failed
- mountpoint rejected
- LoRa lost
- LoRa recovered
- USB opened
- USB lost
- USB recovered
- DDS lost
- DDS recovered
- source changed
- transport changed
- RTCM stale
- RTCM recovered
- FLOAT -> FIXED
- FIXED -> FLOAT
- configuration changed

### 19.6 Stale frontend state

If tablet/backend connection is lost:

- do not keep the last green RTK state displayed as live
- mark UI `DISCONNECTED` or `STALE`
- retain historical values only if clearly marked stale
- actual RTK injection continues independently

---

## 20. Example tablet summary lines

Healthy:

`RTK FIXED · 0.018 m · 21 sats · NTRIP -> USB · RTCM 0.3 s · 18.4 MB RX · 18.1 MB injected · Healthy`

Degraded transport:

`RTK FLOAT · NTRIP -> USB · Source healthy · USB unavailable · delivery age 6.2 s · DEGRADED`

Degraded source:

`RTK FIXED · NTRIP -> DDS · NTRIP reconnecting · last valid RTCM 4.8 s · DEGRADED`

Disconnected tablet:

`ROVER TELEMETRY STALE · last RTK state RTK FIXED · not live`

---

## 21. Safety consumers

RTK status flows outward to safety.

```text
dyx3-rtk
   |
   +-> motion_guard
   +-> spray
   +-> recorder
   +-> system_gateway
   +-> backend
   +-> frontend
```

No consumer controls correction delivery.

### Motion guard

May stop rover motion if RTK policy fails.

It must not stop the RTK worker.

### Spray

May block spray if RTK quality fails.

It must not stop the RTK worker.

### Mission

Must not own correction injection.

Do not automatically port the 4WD "FIXED returned -> auto resume mission" behavior without an explicit 3WD decision.

---

## 22. Recorder requirements

Record:

- RTK configuration revision
- active source
- active transport
- worker state
- source state
- transport state
- valid RTCM frame age
- delivery age
- receiver correction age
- source byte/frame counters
- transport byte/frame counters
- CRC failures
- reconnects
- write/publish failures
- fix type
- h_acc
- satellites
- FIX transitions
- RTK state transitions
- source transitions
- transport transitions
- errors

This must make field failures diagnosable without relying on the tablet.

---

## 23. Failure-domain acceptance matrix

| Failure | USB_DIRECT selected | PX4_DDS selected |
|---|---|---|
| FastAPI crash | no interruption | no interruption |
| backend redeploy | no interruption | no interruption |
| WebSocket loss | no interruption | no interruption |
| tablet disconnect | no interruption | no interruption |
| RPP crash | no interruption | no interruption |
| mission crash | no interruption | no interruption |
| spray crash | no interruption | no interruption |
| system gateway crash | no interruption | no interruption |
| `dyx3_px4_link` crash | no interruption | no interruption if RTK owns its DDS sink |
| PX4/XRCE unavailable | USB continues | DDS degrades/retries |
| USB receiver unplugged | USB degrades/retries | irrelevant |
| LTE loss | NTRIP reconnects | NTRIP reconnects |
| NTRIP caster loss | NTRIP reconnects | NTRIP reconnects |
| LoRa unplugged | LoRa retries | LoRa retries |
| bad RTCM CRC | reject/count/resync | reject/count/resync |
| `dyx3-rtk` crash | systemd restarts + restores persisted config | same |

---

## 24. Reference behavior from 4WD prototype

Reuse concepts, not architecture mistakes.

Strong reference lineage:

- validated RTCM3 parser
- RTCM transport gating
- deterministic manager/supervisor concepts
- persisted NTRIP profiles
- RTK lifecycle hardening
- production serial RTCM sink
- direct injection independent of PX4/MAVROS
- stable USB `/dev/serial/by-id/...`
- persisted NTRIP/LoRa source selection
- correction-stream vs GNSS-solution telemetry
- single injection ownership lock
- source/device listing
- frontend RTK status contract

Do not port:

- backend-owned RTK worker process
- FastAPI lifecycle as RTK process authority
- MAVROS transport assumptions
- duplicate RTK status routes
- source/transport coupling
- automatic mission resume policy
- ttyACM enumeration paths

---

## 25. Phased implementation

### Phase A — Contract freeze

Before code:

- freeze RTK production architecture
- freeze source enum
- freeze transport enum
- freeze worker state enum
- freeze reason/error codes
- freeze correction-age semantics
- freeze REST contract
- freeze WS contract
- freeze frontend TypeScript contract
- decide `/fmu/in/gps_inject_data` ownership
- document current Phase 8 migration

Acceptance:

- no runtime code yet
- contracts reviewed
- no ambiguous age field
- no dual-injection design path

### Phase B — RTK core refactor

Deliver:

- source abstraction
- transport abstraction
- injection authority
- top-level state machine
- persistent config reader
- status model
- event model

Preserve:

- current NTRIP
- current RTCM parser
- current GGA
- current counters

Tests:

- source/transport combinations
- state transitions
- config validation
- stale frame rejection
- single authority

### Phase C — USB direct transport

Deliver:

- stable by-id resolver
- exclusive serial sink
- write/reopen logic
- counters
- readback support
- device loss/recovery

Tests:

- partial write
- zero-byte write
- write exception
- unplug/replug simulation
- stale path
- wrong device
- no ttyACM dependency

### Phase D — DDS transport

Deliver:

- direct `GpsInjectData` writer
- compatibility checks
- delivery counters
- DDS availability handling

Tests:

- DDS absent
- DDS appears
- PX4 restart
- agent restart
- publisher mismatch
- message mismatch
- stale data not replayed

### Phase E — systemd independence

Deliver:

- production `dyx3-rtk.service`
- persistent desired state
- startup restore
- restart recovery
- watchdog/health
- secrets permissions
- logs

Tests:

- backend killed
- gateway killed
- worker killed
- reboot
- missing network
- missing source
- missing transport

### Phase F — backend control API

Deliver:

- Unix control socket client
- canonical REST API
- config validation projection
- profile management
- serial-port list
- one status endpoint

Tests:

- full FastAPI route integration
- backend restart
- unavailable worker -> 503
- secret redaction
- invalid config leaves runtime unchanged

### Phase G — WebSocket telemetry

Deliver:

- canonical RTK telemetry envelope
- change-driven event updates
- periodic freshness/status update
- stale handling

Tests:

- schema
- reconnect
- no password leakage
- stale UI behavior

### Phase H — tablet RTK tab

Deliver:

- overall card
- source card
- transport card
- receiver solution card
- counters
- event history
- profile/config controls
- source selector
- transport selector
- clear stale/disconnected state

Tests:

- NTRIP + USB
- NTRIP + DDS
- LoRa + USB
- LoRa + DDS
- source fault
- transport fault
- receiver FLOAT
- receiver FIXED
- backend disconnect

### Phase I — safety integration

Deliver:

- motion guard RTK gate
- spray RTK gate
- recorder integration

Do not alter correction worker lifecycle.

### Phase J — installer / production deployment

Deliver:

- stable udev/by-id validation
- `dialout`/device permissions
- `/etc/dyx3` secrets
- `dyx3-rtk.service`
- health command
- upgrade/rollback behavior

### Phase K — bench validation

Required tests:

1. boot with NTRIP + USB
2. boot with NTRIP + DDS
3. boot with LoRa + USB
4. boot with LoRa + DDS
5. verify `3D -> FLOAT -> FIXED`
6. verify `FIXED -> FLOAT -> FIXED`
7. unplug USB
8. reconnect USB
9. stop XRCE agent
10. restart XRCE agent
11. reboot PX4
12. drop LTE
13. restore LTE
14. reject NTRIP mountpoint
15. use wrong NTRIP password
16. inject bad CRC stream
17. kill backend
18. redeploy backend
19. kill system gateway
20. kill RPP
21. kill RTK worker
22. reboot Jetson
23. change source while running
24. change transport while running
25. prove no simultaneous USB+DDS injection

Record all transition times and counters.

### Phase L — field validation

Measure:

- source connection time
- first valid RTCM time
- first successful delivery time
- FLOAT acquisition time
- FIXED acquisition time
- fix stability
- source age
- delivery age
- receiver correction age
- h_acc
- satellites
- transport failures
- correction recovery
- mission/spray gate behavior

No production threshold is accepted solely from desk assumptions.

---

## 26. Open decisions before implementation

These require explicit confirmation:

1. exact UM982 port used for PX4 GNSS input
2. exact UM982 USB interface used for direct RTCM
3. whether direct USB port can simultaneously provide readback/GGA
4. final USB baud
5. final LoRa hardware/baud
6. LTE dongle model
7. whether `dyx3-rtk` directly owns `/fmu/in/gps_inject_data`
8. final control-socket schema
9. exact receiver correction-age source
10. whether automatic USB<->DDS failover is postponed as recommended

---

## 27. Non-goals for the first release

Do not include initially:

- automatic source failover NTRIP<->LoRa
- automatic transport failover USB<->DDS
- simultaneous injection
- receiver configuration
- backend-owned worker process
- mission-driven RTK lifecycle
- tablet-driven RTK lifecycle dependency
- hidden stale data fallback
- inferred receiver correction age from source age
- auto mission resume after FIX recovery

---

## 28. Production acceptance criteria

The RTK upgrade is accepted only when all are true:

1. `dyx3-rtk` survives backend restart without correction interruption.
2. `dyx3-rtk` survives tablet disconnect without correction interruption.
3. USB mode works without PX4/DDS.
4. DDS mode works without backend/FastAPI.
5. NTRIP and LoRa feed the same RTCM validation layer.
6. USB and DDS are never simultaneously active.
7. stale correction frames are never replayed.
8. bad CRC frames are never injected.
9. stable device identities are used.
10. source health, transport health, and receiver solution health are separate.
11. correction ages have unambiguous semantics.
12. the tablet shows source, transport, state, bytes, frames, ages, solution and faults.
13. tablet stale/disconnected state is visible and never masquerades as live RTK.
14. credentials never reach status/WS/logs.
15. recorder contains sufficient evidence to reconstruct failures.
16. motion/spray safety consumes RTK status without owning the worker.
17. reboot restores persisted desired state and configuration.
18. FIX transitions are verified from receiver/FCU evidence, not merely topic presence.
19. field validation proves the chosen configuration.
20. the design is suitable to reuse as the RTK API/frontend model for 4WD production.

---

## 29. Final production principle

`dyx3-rtk` must behave like a self-contained correction appliance inside the Jetson:

- it boots from persisted configuration
- owns one correction source
- validates RTCM
- owns exactly one selected injection transport
- verifies receiver response
- self-recovers
- records evidence
- publishes status

FastAPI, Socket.IO and the tablet may configure and observe it.

They must never be required to keep RTK alive.

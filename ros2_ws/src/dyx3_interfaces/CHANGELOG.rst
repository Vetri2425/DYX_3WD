^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dyx3_interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The full interface history from 0.1.0 is in ``docs/interfaces/CHANGELOG.md`` (CI checks that
every field change bumps ``package.xml`` and adds an entry there). This file records the
releases from 0.13.1 on, in the ROS package changelog format.

0.15.0 (2026-10-10)
-------------------
Mission contract v2 (``docs/plans/2026-10-10_mission_contract_v2.md`` section 3).

* ``MissionState.msg``: new states ``STATE_PLACING=8``, ``STATE_ARMING=9``, ``STATE_ENGAGING=10``;
  new reasons 6..17 (``EKF_RESET``, ``EKF_REFERENCE_INVALID``, ``PLACEMENT_OUT_OF_BOUNDS``,
  ``NO_PLACEMENT_FRAME``, ``ARM_REFUSED``, ``ARM_TIMEOUT``, ``OFFBOARD_REFUSED``, ``OFFBOARD_TIMEOUT``,
  ``RPP_ACK_TIMEOUT``, ``ESTOP``, ``RPP_ERROR``, ``RPP_STALE``); appended ``source_artifact_sha256``,
  ``request_id``, ``reason_detail``, ``gate_reason_code``, ``waiting_on`` (``WAIT_*``) and
  ``state_entered``. ``path_artifact_sha256`` now names the placed execution artifact.
* ``StartMission.srv``: asynchronous admission; request appends ``request_id``; response appends
  ``REASON_INVALID_REQUEST=4``, ``duplicate`` and ``gate_reason_code``.
* ``SafetyGateStatus.msg``: appended ``pre_arm_ok`` and ``pre_arm_reason_code``.
* ``MotionSetpointStatus.msg``: new ``REASON_GLOBAL_REFERENCE_INVALID=13`` (pre-arm reason only).
* ``ResumeMission.srv``: new ``REASON_NOT_ARMED_OR_OFFBOARD=3`` and ``REASON_EKF_REFERENCE_CHANGED=4``.

0.14.0 (2026-10-10)
-------------------
* ``VehicleState.msg``: append ``vertical_position_valid`` and ``vertical_velocity_valid`` from
  PX4 ``z_valid`` / ``v_z_valid`` (IF-004).
* ``MotionSetpoint.msg``: append ``source_pose_sample_stamp`` (PX4 sample time of the pose behind
  the command; RPP fills it, the guard preserves it or stamps its own STOP) (IF-003).
* ``Px4LinkStatus.msg``: append ``pose_to_write_age_valid``, ``pose_to_write_age_s``,
  ``pose_to_write_age_max_s`` (IF-003).

0.13.1 (2026-10-10)
-------------------
* ``SetEmergencyStop.srv``: document the accepted ``source`` values (``tablet``, ``backend``,
  ``ble``, ``physical``; exact, lowercase), that any accepted source may clear, and that an
  accepted assert is applied inside the service call (IF-007). Comment only; no field changed.

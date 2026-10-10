^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dyx3_interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The full interface history from 0.1.0 is in ``docs/interfaces/CHANGELOG.md`` (CI checks that
every field change bumps ``package.xml`` and adds an entry there). This file records the
releases from 0.13.1 on, in the ROS package changelog format.

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

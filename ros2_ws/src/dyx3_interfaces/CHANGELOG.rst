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

0.13.1 (2026-10-10)
-------------------
* ``SetEmergencyStop.srv``: document the accepted ``source`` values (``tablet``, ``backend``,
  ``ble``, ``physical``; exact, lowercase), that any accepted source may clear, and that an
  accepted assert is applied inside the service call (IF-007). Comment only; no field changed.

.. py:currentmodule:: lsst.ts.ATDome

.. _lsst.ts.ATDome.revision_history:

##########################
ts_ATDome Revision History
##########################

v1.1.2
======

Changes:

* Add a test that the code is formatted with ``black``.
  This test uses a function that was added to ts_salobj 5.11.
* Use mock_port=0 with the `ATDomeCsc` and port=0 with the `MockDomeController` constructor to mean "pick an available port".
  This eliminates the risk that a unit test can fail due to trying to use a TCP/IP port that is already in use.

Requires:

* ts_salobj 5.11
* ts_simactuators 0.1
* ts_idl
* IDL file for ATDome from ts_xml 4.8

v1.1.1
======

Major changes:

* Fix determination of "azimuth in position" by using a tolerance a bit larger than that used by the low-level controller.
  This margin is controlled by attribute `az_tolerance_margin`.
* Report ``azimuthEncoderPosition=0`` in the ``position`` telemetry topic, if the value is too large for the schema.

Requires:

* ts_salobj 5.4
* ts_simactuators 0.1
* ts_idl
* IDL file for ATDome from ts_xml 4.8

v1.1.0
======

Major changes:

* Output additional information, as new fields in the ``settingsAppliedController`` event and ``position`` telemetry, plus new events ``doorEncoderExtremes`` and ``lastAzimuthGoTo``.
  This requires ts_xml 4.8.
* Improve error handling by rejecting commands if the low level controller returns unexpected data.
* Rewrite test_csc to use `lsst.ts.salobj.BaseCscTestCase`.
  This requires ts_salobj 5.4.
* Code formatted by ``black``, with a pre-commit hook to enforce this. See the README file for configuration instructions.

Requires:

* ts_salobj 5.4
* ts_simactuators 0.1
* ts_idl
* IDL file for ATDome from ts_xml 4.8

v1.0.0
======

Update for a change to the low-level controller (a minor change to full status output).

Requires:

* ts_salobj 5.2
* ts_simactuators 0.1
* ts_idl
* IDL file for ATDome from ts_xml 4.1

v0.10.0
=======

Update to use ts_simactuators.

Requires:

* ts_salobj 5.2
* ts_simactuators 0.1
* ts_idl
* IDL file for ATDome from ts_xml 4.1

v0.9.0
======

Update for ts_salobj 5.2: rename initial_simulation_mode to simulation_mode.

Requires:

* ts_salobj 5.2
* ts_idl
* IDL file for ATDome from ts_xml 4.1

v0.8.0
======

Change the shutter motion commands to report done only after the shutter motion finishes.
Change the behavior when going from ENABLED to DISABLED state to stop the azimuth and close the shutters.

Note that the stop command and any valid shutter move command will cancel and supersede any existing shutter move command.

Updated the unit tests to use the ``asynctest`` package.

Requires:

* ts_salobj 5
* ts_idl
* IDL file for ATDome from ts_xml 4.1

v0.7.0
======

Make ATDome a non-indexed SAL component.

Requires:

* ts_salobj 4.3
* ts_idl
* IDL file for ATDome from ts_xml 4.1

v0.6.1
======

Add a dependency on ``ts_config_attcs`` to the ups table file.

v0.6.0
======

Use OpenSplice dds instead of SALPY libraries.

Requires:

* ts_salobj 4
* ts_idl
* IDL file for ATDome from ts_xml 3.9

v0.5.0
======

Make configurable in the standard way.
The configuration files are in package ``ts_config_attcs``.

Requires:

* ts_sal 3.9
* ts_salobj 3.12
* ts_xml 3.9

v0.4.0
======

Add commanded state events.
Fixed several issues with the real ATDome TCP/IP interface.

Requires:

* ts_xml develop rev 865c63d
* ts_sal 3.8.41
* ts_salobj 3.9

v0.3.0
======

Allow ``run_atdome.py`` to start in simulation mode.

Requires:

* ts_sal 3.8.41
* ts_salobj 3.8
* ts_xml  develop cf6280b through 3.9


v0.2.1
======

Fix line width warnings for documentation and comments.

v0.2.0
======

First release of the real ATDome CSC, not just a simulator.

Updated for a major change to the ATDome XML.

Requires:

* ts_sal 3.8.41
* ts_salobj 3.8
* ts_xml develop cf6280b through 3.9

v0.1.0
======

First release of the ATDome simulator.

Requires:

* ts_sal 3.8.41
* ts_salobj 3.6
* ts_xml 3.8

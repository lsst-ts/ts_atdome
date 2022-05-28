.. py:currentmodule:: lsst.ts.ATDome

.. _lsst.ts.ATDome.version_history:

###############
Version History
###############

v1.7.1
------

* Build using pyproject.toml

Requires:

* ts_salobj 6.3
* ts_simactuators 2
* ts_utils
* ts_idl
* IDL file for ATDome built from ts_xml 11

v1.7.0
------

* Update for ts_salobj v7, which is required.
  This also requires ts_xml 11.
* Use ts_utils and pytest-black.

Requires:

* ts_salobj 6.3
* ts_simactuators 2
* ts_utils
* ts_idl
* IDL file for ATDome built from ts_xml 11

v1.6.1
------

* Remove ``asynctest`` from setup.py

Requires:

* ts_salobj 6.3
* ts_simactuators 2
* ts_idl
* IDL file for ATDome from ts_xml 9

v1.6.0
------

* `DomeCsc`: overhaul azimuth homing:

    * Output new azimuthState event fields ``homed`` and ``homeSwitch``.
      This requires ts_xml 9.
    * Make the ``homeAzimuth`` command report wait until homing is finished,
      and provide an CMD_INPROGRESS acknowledgement with a pessimistic time estimate when it starts.
    * Make ``moveAzimuth`` and ``homeAzimuth`` fail while homing.
    * Log a warning if you call ``moveAzimuth`` when the dome is not homed
      (as far as the CSC knows; this information is not yet available from the low-level controller).
* Update the User Guide to fix outdated information about ATDomeTrajectory
  and add links to the ts_xml documentation for all commands and events mentioned in the guide.
* Update tests to use `unittest.IsolatedAsyncioTestCase` instead of the abandoned asynctest package.

Requires:

* ts_salobj 6.3
* ts_simactuators 2
* ts_idl
* IDL file for ATDome from ts_xml 9

v1.5.2
------

* Reformat the code with black 20.8b1

Requires:

* ts_salobj 6.3
* ts_simactuators 2
* ts_idl
* IDL file for ATDome from ts_xml 8

v1.5.1
------

* Make test_csc.py more robust by changing assert_angle_in_range to test angle <= max_angle instead of <.
  This avoids a race condition.

Requires:

* ts_salobj 6.3
* ts_simactuators 2
* ts_idl
* IDL file for ATDome from ts_xml 8

v1.5.0
------

* Store the CSC configuration schema in code.
  This requires ts_salobj 6.3.

Requires:

* ts_salobj 6.3
* ts_simactuators 2
* ts_idl
* IDL file for ATDome from ts_xml 8

v1.4.0
------

* `ATDomeCsc`: publish the new moveCode event. This requires ts_xml 8.
* `ATDomeCsc`: set class variable ``version`` and test that this properly sets the ``cscVersion`` field of the ``softwareVersions`` event.
* `ATDomeCsc`: make the ``standby`` command more reliable by eliminating a race condition that could cause a fault.
* `MoveCode`: make this a public class and update `MockDomeController` to use it.
* `MockDomeController`: change the initial azimuth and specify it in a new constant ``INITIAL_AZIMUTH``.
* Fixed a race condition in test_stop in test_csc.py.
* Update doc/conf.py for documenteer 0.6.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_idl
* IDL file for ATDome from ts_xml 8

v1.3.4
------

* `ATDomeCsc`: update the moveAzimuth command to support azimuth angles outside the range [0, 360)

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_idl
* IDL file for ATDome from ts_xml 4.8

v1.3.3
------

* Fix an exception in the status loop caused by comparing an astropy unit to a scalar (DM-27885).
* Update the unit tests to check the three inPosition events after a move (which catches DM-27885).
* Add intersphinx mapping to ts_salobj and ts_xml to the documentation.
* Use ``pre-commit`` instead of a custom git pre-commit hook.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_idl
* IDL file for ATDome from ts_xml 4.8

v1.3.2
------

* Update Jenkinsfile.conda to Jenkins Shared Library
* Pin the ts-idl and ts-salobj versions in the conda recipe

v1.3.1
------

Changes:

* Fix conda build.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_idl
* IDL file for ATDome from ts_xml 4.8

v1.3.0
------

Changes:

* Update `ATDomeCsc` to use ts_salobj 6 simulation mode support.

Requires:

* ts_salobj 6
* ts_simactuators 2
* ts_idl
* IDL file for ATDome from ts_xml 4.8

v1.2.1
------

Changes:

* Overhaul the documentation.

Requires:

* ts_salobj 5.15
* ts_simactuators 2
* ts_idl
* IDL file for ATDome from ts_xml 4.8

v1.2.0
------

Changes:

* Updated for ts_simactuators 2
* Removed all use of astropy Angle.

Requires:

* ts_salobj 5.15
* ts_simactuators 2
* ts_idl
* IDL file for ATDome from ts_xml 4.8

v1.1.3
------

Changes:

* Remove ``sudo: false`` from ``.travis.yml``.

Requires:

* ts_salobj 5.11
* ts_simactuators 0.1
* ts_idl
* IDL file for ATDome from ts_xml 4.8

v1.1.2
------

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
------

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
------

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
------

Update for a change to the low-level controller (a minor change to full status output).

Requires:

* ts_salobj 5.2
* ts_simactuators 0.1
* ts_idl
* IDL file for ATDome from ts_xml 4.1

v0.10.0
------=

Update to use ts_simactuators.

Requires:

* ts_salobj 5.2
* ts_simactuators 0.1
* ts_idl
* IDL file for ATDome from ts_xml 4.1

v0.9.0
------

Update for ts_salobj 5.2: rename initial_simulation_mode to simulation_mode.

Requires:

* ts_salobj 5.2
* ts_idl
* IDL file for ATDome from ts_xml 4.1

v0.8.0
------

Change the shutter motion commands to report done only after the shutter motion finishes.
Change the behavior when going from ENABLED to DISABLED state to stop the azimuth and close the shutters.

Note that the stop command and any valid shutter move command will cancel and supersede any existing shutter move command.

Updated the unit tests to use the ``asynctest`` package.

Requires:

* ts_salobj 5
* ts_idl
* IDL file for ATDome from ts_xml 4.1

v0.7.0
------

Make ATDome a non-indexed SAL component.

Requires:

* ts_salobj 4.3
* ts_idl
* IDL file for ATDome from ts_xml 4.1

v0.6.1
------

Add a dependency on ``ts_config_attcs`` to the ups table file.

v0.6.0
------

Use OpenSplice dds instead of SALPY libraries.

Requires:

* ts_salobj 4
* ts_idl
* IDL file for ATDome from ts_xml 3.9

v0.5.0
------

Make configurable in the standard way.
The configuration files are in package ``ts_config_attcs``.

Requires:

* ts_sal 3.9
* ts_salobj 3.12
* ts_xml 3.9

v0.4.0
------

Add commanded state events.
Fixed several issues with the real ATDome TCP/IP interface.

Requires:

* ts_xml develop rev 865c63d
* ts_sal 3.8.41
* ts_salobj 3.9

v0.3.0
------

Allow ``run_atdome.py`` to start in simulation mode.

Requires:

* ts_sal 3.8.41
* ts_salobj 3.8
* ts_xml  develop cf6280b through 3.9


v0.2.1
------

Fix line width warnings for documentation and comments.

v0.2.0
------

First release of the real ATDome CSC, not just a simulator.

Updated for a major change to the ATDome XML.

Requires:

* ts_sal 3.8.41
* ts_salobj 3.8
* ts_xml develop cf6280b through 3.9

v0.1.0
------

First release of the ATDome simulator.

Requires:

* ts_sal 3.8.41
* ts_salobj 3.6
* ts_xml 3.8

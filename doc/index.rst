.. py:currentmodule:: lsst.ts.atdome

.. _lsst.ts.atdome:

##############
lsst.ts.atdome
##############

.. image:: https://img.shields.io/badge/Project Metadata-gray.svg
    :target: https://ts-xml.lsst.io/index.html#index-csc-table-atdome
.. image:: https://img.shields.io/badge/SAL\ Interface-gray.svg
    :target: https://ts-xml.lsst.io/sal_interfaces/ATDome.html
.. image:: https://img.shields.io/badge/GitHub-gray.svg
    :target: https://github.com/lsst-ts/ts_atdome
.. image:: https://img.shields.io/badge/Jira-gray.svg
    :target: https://jira.lsstcorp.org/issues/?jql=project%3DDM%20AND%20labels%3Dts_atdome

.. _lsst.ts.atdome.overview:

Overview
========

ATDome controls the Vera C. Rubin Observatory Auxiliary Telescope Dome.

The Auxiliary Telescope Dome is a 30 foot diameter Ash Dome with azimuth rotation and two shutter doors.
The azimuth axis has no rotation limits.
The top shutter door gives visibility from 30° elevation to zenith (90°), whereas the bottom shutter gives visibility from the horizon to 30° elevation.

.. _lsst.ts.atdome.user_guide:

User Guide
==========

Start the ATDome CSC as follows:

.. prompt:: bash

    run_atdome

Stop the CSC by sending it to the OFFLINE state.

See ATDome `SAL communication interface <https://ts-xml.lsst.io/sal_interfaces/ATDome.html>`_ for commands, events and telemetry.

For on-sky observing: enable ATDomeTrajectory azimuth following by issuing the ATDomeTrajectory `setEnabledMode`_ command command with ``enabled=True``.
ATDomeTrajectory will automatically command ATDome to follow the telescope azimuth.

To move the dome manually in azimuth: first disable ATDomeTrajectory azimuth following by issuing the ATDomeTrajectory `setEnabledMode`_ command command with ``enabled=False``.
Then issue the ATDome `moveAzimuth`_ command to move the dome.

The `moveAzimuth`_ command is reported as done as soon as the command is received;
use the `azimuthState`_ event to track when the move actually finishes.

The door commands are reported done when the doors have finished moving.

The `homeAzimuth`_ command is reported done when homing has finished.
The low-level controller does not report whether the azimuth axis has been homed,
so the CSC only knows the dome has been homed if you run the `homeAzimuth`_ command.
Thus it will log a warning if issue the `moveAzimuth`_ command before the CSC has homed the dome, but still allow the motion.

New commands may be sent at any time, though `moveAzimuth`_ and `homeAzimuth`_ will be rejected while homing azimuth.
If a new door command arrives while the door is moving, the door immediately starts moving to the new position and the old door command is reported as superseded.

.. _homeAzimuth: https://ts-xml.lsst.io/sal_interfaces/ATDome.html#homeazimuth
.. _moveAzimuth: https://ts-xml.lsst.io/sal_interfaces/ATDome.html#moveazimuth
.. _azimuthState: https://ts-xml.lsst.io/sal_interfaces/ATDome.html#azimuthstate
.. _setEnabledMode: https://ts-xml.lsst.io/sal_interfaces/ATDomeTrajectory.html#setenabledmode

.. _lsst.ts.atdome.configuration:

Configuration
-------------

It is unlikely that a user will have to modify the default configuration for ATDome, as it only contains parameters for the connection to the low-level controller.

Configuration is defined by `this schema <https://github.com/lsst-ts/ts_atdome/blob/main/python/lsst/ts/atdome/schema_config.py>`_.

Configuration files live in `ts_config_attcs/ATDome <https://github.com/lsst-ts/ts_config_attcs/tree/develop/ATDome>`_.

.. _lsst.ts.atdome.simulation:

Simulator
---------

The CSC includes a simulation mode. To run using simulation:

.. prompt:: bash

    run_atdome --simulate

The simulated azimuth axis and shutter doors move at constant speed, with infinite acceleration.

Developer Guide
===============

.. toctree::
    developer_guide
    :maxdepth: 1

Version History
===============

.. toctree::
    version_history
    :maxdepth: 1

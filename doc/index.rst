.. py:currentmodule:: lsst.ts.ATDome

.. _lsst.ts.ATDome:

##############
lsst.ts.ATDome
##############

.. image:: https://img.shields.io/badge/SAL\ Interface-gray.svg
    :target: https://ts-xml.lsst.io/sal_interfaces/ATDome.html
.. image:: https://img.shields.io/badge/GitHub-gray.svg
    :target: https://github.com/lsst-ts/ts_ATDome
.. image:: https://img.shields.io/badge/Jira-gray.svg
    :target: https://jira.lsstcorp.org/issues/?jql=labels+%3D+ts_ATDome

.. _lsst.ts.ATDome.overview:

Overview
========

ATDome controls the Vera C. Rubin Observatory Auxiliary Telescope Dome.

The Auxiliary Telescope Dome is a 30 foot diameter Ash Dome with azimuth rotation and two shutter doors.
The azimuth axis has no rotation limits.
The top shutter door gives visibility from 30° elevation to zenith (90°), whereas the bottom shutter gives visibility from the horizon to 30° elevation.

.. _lsst.ts.ATDome.user_guide:

User Guide
==========

Start the ATDome CSC as follows:

.. prompt:: bash

    run_atdome.py

Stop the CSC by sending it to the OFFLINE state.

See ATDome `SAL communication interface <https://ts-xml.lsst.io/sal_interfaces/ATDome.html>`_ for commands, events and telemetry.

For on-sky observing, send the `ATDomeTrajectory CSC <https://ts-atdometrajectory.lsst.io/>`_ to the ENABLED state. Then ``ATDomeTrajectory`` will automatically command ``ATDome`` azimuth to follow the telescope.
When you want to command ``ATDome`` azimuth manually, send ``ATDomeTrajectory`` to the DISABLED state to prevent it from commanding ``ATDome``.

The ``moveAzimuth`` command is reported as done as soon as the command is received; use the ``azimuthState`` event to track when the move finishes.
The door commands are reported done when the doors have finished moving.

New commands may be sent at any time.
If a new door command arrives while the door is moving, the door immediately starts moving to the new position and the old door command is reported as superseded.

.. _lsst.ts.ATDome.configuration:

Configuration
-------------

It is unlikely that a user will have to modify the default configuration for ATDome, as it only contains parameters for the connection to the low-level controller.

Configuration is defined by `this schema <https://github.com/lsst-ts/ts_ATDome/blob/develop/schema/ATDome.yaml>`_.

Configuration files live in `ts_config_attcs/ATDome <https://github.com/lsst-ts/ts_config_attcs/tree/develop/ATDome>`_.

.. _lsst.ts.ATDome.simulation:

Simulator
---------

The CSC includes a simulation mode. To run using simulation:

.. prompt:: bash

    run_atdome.py --simulate

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

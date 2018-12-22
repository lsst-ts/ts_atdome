.. py:currentmodule:: lsst.ts.ATDome

.. _lsst.ts.ATDome:

##############
lsst.ts.ATDome
##############

A simulator for the LSST ATDome CSC.

.. _lsst.ts.ATDome-using:

Using lsst.ts.ATDome
====================

This package has the following requirements:

* ts_salobj
* The SALPY_ATDome SAL library

The package is compatible with LSST DM's `scons` build system and `eups` package management system.
To build the package:

* ``setup -r .`` to setup the package and dependencies.
* ``scons`` to build the package and run unit tests.
* ``scons install declare`` to install the package and declare it to eups.
* `package-docs build` to build the documentation.
  This requires `documenteer`; see [building single package docs](https://developer.lsst.io/stack/building-single-package-docs.html) for installation instructions.

Run the simulator using ``bin/run_atdome_simulator.py`` (which only exists after you build the package).

.. toctree linking to topics related to using the module's APIs.

.. .. toctree::
..    :maxdepth: 1

.. _lsst.ts.ATDome-contributing:

Contributing
============

``lsst.ts.ATDome`` is developed at https://github.com/lsst-ts/ts_ATDomeSimulator.

.. .. toctree::
..    :maxdepth: 1

.. _lsst.ts.ATDome-pyapi:

Python API reference
====================

.. automodapi:: lsst.ts.ATDome
   :no-main-docstr:
   :no-inheritance-diagram:

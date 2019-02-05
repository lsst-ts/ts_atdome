.. py:currentmodule:: lsst.ts.ATDome

.. _lsst.ts.ATDome:

##############
lsst.ts.ATDome
##############

Controller for the LSST auxiliary telescope dome (ATDome).

.. _lsst.ts.ATDome-using:

Using lsst.ts.ATDome
====================

Build and Test
--------------

This package has the following requirements:

* ts_salobj
* The SALPY_ATDome SAL library

The package is compatible with LSST DM's ``scons`` build system and ``eups`` package management system.
Assuming you have the basic LSST DM stack installed you can do the following, from within the package directory:

* ``setup -r .`` to setup the package and dependencies.
* ``scons`` to build the package and run unit tests.
* ``scons install declare`` to install the package and declare it to eups.
* `package-docs build` to build the documentation.
  This requires ``documenteer``; see `building single package docs`_ for installation instructions.

Usage
-----

The primary classes are:

* `ATDomeCsc`: a simulator for the ATDome CSC.
* `MockDomeController`: a simulator for the ATDome TCP/IP interface.

Run the ``ATDome`` controller  using ``bin/run_atdome.py`` (which only exists after you build the package).

.. _building single package docs: https://developer.lsst.io/stack/building-single-package-docs.html

.. _lsst.ts.ATDome-contributing:

Contributing
============

``lsst.ts.ATDome`` is developed at https://github.com/lsst-ts/ts_ATDomeSimulator.
You can find Jira issues for this module under the `ts_ATDome <https://jira.lsstcorp.org/issues/?jql=project%20%3D%20DM%20AND%20component%20%3D%20ts_ATDome>`_ component.

.. _lsst.ts.ATDome-pyapi:

Python API reference
====================

.. automodapi:: lsst.ts.ATDome
   :no-main-docstr:
   :no-inheritance-diagram:

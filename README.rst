#########
ts_ATDome
#########

``ts_ATDome`` is an LSST Telescope and Site package that implements the auxiliary telescope dome (ATDome) controller.

The primary classes are:

* ``ATDomeCsc``: controller for the auxiliary telescope dome.
* ``MockDomeController``:  simulator for the auxiliary telescope dome TCP/IP interface.

The package is compatible with LSST DM's ``scons`` build system and ``eups`` package management system.
Assuming you have the basic LSST DM stack installed you can do the following, from within the package directory:

* ``setup -r .`` to setup the package and dependencies.
* ``scons`` to build the package and run unit tests.
* ``scons install declare`` to install the package and declare it to eups.
* ``package-docs build`` to build the documentation.
  This requires ``documenteer``; see `building single package docs`_ for installation instructions.

.. _building single package docs: https://developer.lsst.io/stack/building-single-package-docs.html

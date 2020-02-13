#########
ts_ATDome
#########

``ts_ATDome`` is a Commandable SAL Component (CSC) to control the dome for the auxiliary telescope at the Vera C. Rubin Observatory.

`Documentation <https://ts-atdome.lsst.io>`_

The package is compatible with Vera Rubin LSST DM's ``scons`` build system, and the `eups <https://github.com/RobertLuptonTheGood/eups>`_ package management system.
Assuming you have the basic DM stack installed you can do the following, from within the package directory:

* ``setup -r .`` to setup the package and dependencies.
* ``scons`` to build the package and run unit tests.
* ``scons install declare`` to install the package and declare it to eups.
* ``package-docs build`` to build the documentation.
  This requires ``documenteer``; see `building single package docs <https://developer.lsst.io/stack/building-single-package-docs.html>`_ for installation instructions.

This code is automatically formatted by ``black`` using a git pre-commit hook.
To enable this:

* Install the ``black`` Python package.
* Run ``git config core.hooksPath .githooks`` once in this repository.

#########
ts_ATDome
#########

``ts_ATDome`` is a package in the `LSST Science Pipelines <https://pipelines.lsst.io>`_.

The primary classes are:

* ``ATDomeCsc``: a simulator for the ATDome CSC.
* ``MockDomeController``: a simulator for the ATDome TCP/IP interface.

The package is compatible with LSST DM's `scons` build system and `eups` package management system.
Assuming you have the basic LSST DM stack installed you can do the following, from within the package directory:

* ``setup -r .`` to setup the package and dependencies.
* ``scons`` to build the package and run unit tests.
* ``scons install declare`` to install the package and declare it to eups.
* `package-docs build` to build the documentation.
  This requires `documenteer`; see [building single package docs](https://developer.lsst.io/stack/building-single-package-docs.html) for installation instructions.

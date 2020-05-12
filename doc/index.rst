.. py:currentmodule:: lsst.ts.ATDome

.. _lsst.ts.ATDome:

##############
lsst.ts.ATDome
##############

Controller for the LSST auxiliary telescope dome.

.. _lsst.ts.ATDome-using:

Using lsst.ts.ATDome
====================

The primary classes are:

* `ATDomeCsc`: controller for the auxiliary telescope dome.
* `MockDomeController`: simulator for the auxiliary telescope dome TCP/IP interface.

Run the ``ATDome`` controller  using ``bin/run_atdome.py`` (which only exists after you build the package).

.. _building single package docs: https://developer.lsst.io/stack/building-single-package-docs.html

.. _lsst.ts.ATDome-contributing:

Contributing
============

``lsst.ts.ATDome`` is developed at https://github.com/lsst-ts/ts_ATDome.
You can find Jira issues for this module using `labels=ts_ATDome <https://jira.lsstcorp.org/issues/?jql=project%20%3D%20DM%20AND%20labels%20%20%3D%20ts_ATDome>`_.

.. _lsst.ts.ATDome-pyapi:

Python API reference
====================

.. automodapi:: lsst.ts.ATDome
   :no-main-docstr:
   :no-inheritance-diagram:

Revision History
================

.. toctree::
    revision_history
    :maxdepth: 1

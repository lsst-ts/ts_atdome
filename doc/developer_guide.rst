.. py:currentmodule:: lsst.ts.atdome

.. _lsst.ts.atdome.developer_guide:

###############
Developer Guide
###############

The ATDome CSC is implemented using `ts_salobj <https://github.com/lsst-ts/ts_salobj>`_.

The CSC controls the dome using a TCP/IP connection to a low-level controller provided by Astronomical Consulting Equipment. Here is the `software manual <https://docushare.lsst.org/docushare/dsweb/Get/Document-27878/LSST%20AT%20SmartDome.pdf>`_ and `code 
<https://github.com/lsst-ts/ts_auxtel_dome>`_ for that low-level controller, as well as an `electrical diagram <https://docushare.lsst.org/docushare/dsweb/Get/Document-27879/LSST_AT_SMARTDOME%20V2%20FINAL.pdf>`_.

.. _lsst.ts.atdome.api:

API
===

The primary classes are:

* `ATDomeCsc`: controller for the auxiliary telescope dome.
* `MockDomeController`: simulator for the auxiliary telescope dome TCP/IP interface.

.. automodapi:: lsst.ts.atdome
    :no-main-docstr:

.. _lsst.ts.atdome.build:

Build and Test
==============

This is a pure python package. There is nothing to build except the documentation.

.. code-block:: bash

    make_idl_files.py ATDome
    setup -r .
    pytest -v  # to run tests
    package-docs clean; package-docs build  # to build the documentation

.. _lsst.ts.atdome.contributing:

Contributing
============

``lsst.ts.atdome`` is developed at https://github.com/lsst-ts/ts_atdome.
Bug reports and feature requests use `Jira with project=DM labels=ts_atdome <https://jira.lsstcorp.org/issues/?jql=project%3DDM%20AND%20labels%3Dts_atdome>`_.

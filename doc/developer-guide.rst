.. py:currentmodule:: lsst.ts.ATDome

.. _lsst.ts.ATDome.developer_guide:

###############
Developer Guide
###############

The ATDome CSC is implemented using `ts_salobj <https://github.com/lsst-ts/ts_salobj>`_.

The CSC controls the dome using a TCP/IP connection to a low-level controller provided by Astronomical Consulting Equipment. Here is the `software manual <https://docushare.lsst.org/docushare/dsweb/Get/Document-27878/LSST%20AT%20SmartDome.pdf>`_ and `code 
<https://github.com/lsst-ts/ts_auxtel_dome>`_ for that low-level controller, as well as an `electrical diagram <https://docushare.lsst.org/docushare/dsweb/Get/Document-27879/LSST_AT_SMARTDOME%20V2%20FINAL.pdf>`_.

.. _lsst.ts.ATDome.api:

API
===

The primary classes are:

* `ATDomeCsc`: controller for the auxiliary telescope dome.
* `MockDomeController`: simulator for the auxiliary telescope dome TCP/IP interface.

.. automodapi:: lsst.ts.ATDome
    :no-main-docstr:

.. _lsst.ts.ATDome.build:

Build and Test
==============

This is a pure python package. There is nothing to build except the documentation.

.. code-block:: bash

    make_idl_files.py ATDome
    setup -r .
    pytest -v  # to run tests
    package-docs clean; package-docs build  # to build the documentation

.. _lsst.ts.ATDome.contributing:

Contributing
============

``lsst.ts.ATDome`` is developed at https://github.com/lsst-ts/ts_ATDome.
Bug reports and feature requests use `Jira with labels=ts_ATDome <https://jira.lsstcorp.org/issues/?jql=project%20%3D%20DM%20AND%20labels%20%20%3D%20ts_ATDome>`_.

# This file is part of ts_ATDome.
#
# Developed for the LSST Data Management System.
# This product includes software developed by the LSST Project
# (https://www.lsst.org).
# See the COPYRIGHT file at the top-level directory of this distribution
# for details of code ownership.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License

import asyncio
import math
import shutil
import unittest

from astropy.coordinates import Angle
import astropy.units as u

from lsst.ts import salobj
from lsst.ts import ATDome

import SALPY_ATDome

port_generator = salobj.index_generator(imin=3200)


class Harness:
    def __init__(self, initial_state):
        salobj.test_utils.set_random_lsst_dds_domain()
        self.index = 1
        self.remote = salobj.Remote(SALPY_ATDome, index=self.index)
        self.csc = ATDome.ATDomeCsc(
            index=self.index,
            initial_state=initial_state, initial_simulation_mode=1)
        self.csc.port = next(port_generator)

    async def stop(self):
        await asyncio.wait_for(self.csc.stop(), timeout=2)


class CscTestCase(unittest.TestCase):
    def test_initial_info(self):

        async def doit():
            harness = Harness(initial_state=salobj.State.ENABLED)
            state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
            self.assertEqual(state.summaryState, salobj.State.ENABLED)

            az_state = await harness.remote.evt_azimuthState.next(flush=False, timeout=2)
            self.assertEqual(az_state.state, SALPY_ATDome.ATDome_shared_AzimuthState_NotInMotion)
            self.assertFalse(az_state.homing)

            main_door_state = await harness.remote.evt_mainDoorState.next(flush=False, timeout=2)
            self.assertEqual(main_door_state.state, SALPY_ATDome.ATDome_shared_ShutterDoorState_Closed)
            dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False, timeout=2)
            self.assertEqual(dropout_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Closed)
            emergency_stop = await harness.remote.evt_emergencyStop.next(flush=False, timeout=2)
            self.assertFalse(emergency_stop.active)

            position = await harness.remote.tel_position.next(flush=False, timeout=2)
            self.assertEqual(position.dropoutDoorOpeningPercentage, 0)
            self.assertEqual(position.mainDoorOpeningPercentage, 0)
            self.assertAlmostEqual(position.azimuthPosition, 0)
            self.assertTrue(math.isnan(position.dropoutDoorOpeningPercentageSet))
            self.assertTrue(math.isnan(position.mainDoorOpeningPercentageSet))
            self.assertTrue(math.isnan(position.azimuthPositionSet))

            tcp_settings = await harness.remote.evt_settingsAppliedDomeTcp.next(flush=False, timeout=2)
            self.assertEqual(tcp_settings.host, "127.0.0.1")
            # the port will not match, because we set it in an underhanded way
            # for the unit test
            self.assertEqual(tcp_settings.connectionTimeout, harness.csc.connection_timeout)
            self.assertEqual(tcp_settings.readTimeout, harness.csc.read_timeout)

            await harness.stop()

        asyncio.get_event_loop().run_until_complete(doit())

    def test_home(self):
        async def doit():
            harness = Harness(initial_state=salobj.State.ENABLED)

            state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
            self.assertEqual(state.summaryState, salobj.State.ENABLED)
            az_state = await harness.remote.evt_azimuthState.next(flush=False, timeout=2)
            self.assertEqual(az_state.state, SALPY_ATDome.ATDome_shared_AzimuthState_NotInMotion)
            self.assertFalse(az_state.homing)

            # set home azimuth near current position so homing goes quickly
            curr_az = harness.csc.mock_ctrl.az_actuator.curr_pos
            home_azimuth = (curr_az - 2*u.deg).wrap_at(Angle(360, u.deg))
            harness.csc.mock_ctrl.home_az = home_azimuth

            await harness.remote.cmd_homeAzimuth.start(timeout=2)

            # wait for homing to begin and check status
            az_state = await harness.remote.evt_azimuthState.next(flush=False, timeout=1)
            self.assertEqual(az_state.state, SALPY_ATDome.ATDome_shared_AzimuthState_MovingCCW)
            self.assertTrue(az_state.homing)
            position = harness.remote.tel_position.get()
            self.assertGreater(position.azimuthPosition, home_azimuth.deg)
            self.assertTrue(math.isnan(position.azimuthPositionSet))

            # check that moveAzimuth is disallowed while homing
            harness.remote.cmd_moveAzimuth.set(azimuth=0)  # arbitrary
            with salobj.assertRaisesAckError():
                await harness.remote.cmd_moveAzimuth.start(timeout=2)

            # check that homing is disallowed while homing
            harness.remote.cmd_moveAzimuth.set(azimuth=0)  # arbitrary
            with salobj.assertRaisesAckError():
                await harness.remote.cmd_homeAzimuth.start(timeout=2)

            # wait for the initial CCW homing move to finish
            az_state = await harness.remote.evt_azimuthState.next(flush=False, timeout=1)
            self.assertTrue(az_state.homing)
            self.assertEqual(az_state.state, SALPY_ATDome.ATDome_shared_AzimuthState_MovingCW)
            self.assertAlmostEqual(harness.csc.mock_ctrl.az_actuator.speed.deg,
                                   harness.csc.mock_ctrl.home_az_vel.deg)

            # wait for the slow CW homing move to finish
            az_state = await harness.remote.evt_azimuthState.next(flush=False, timeout=1)
            self.assertFalse(az_state.homing)
            self.assertEqual(az_state.state, SALPY_ATDome.ATDome_shared_AzimuthState_NotInMotion)
            self.assertAlmostEqual(harness.csc.mock_ctrl.az_actuator.speed.deg,
                                   harness.csc.mock_ctrl.az_vel.deg)
            position = harness.remote.tel_position.get()
            self.assertAlmostEqual(position.azimuthPosition, home_azimuth.deg)
            self.assertTrue(math.isnan(position.azimuthPositionSet))

        asyncio.get_event_loop().run_until_complete(doit())

    def test_move_az(self):
        async def doit():
            harness = Harness(initial_state=salobj.State.ENABLED)
            state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
            self.assertEqual(state.summaryState, salobj.State.ENABLED)
            az_state = await harness.remote.evt_azimuthState.next(flush=False, timeout=2)
            self.assertEqual(az_state.state, SALPY_ATDome.ATDome_shared_AzimuthState_NotInMotion)
            self.assertFalse(az_state.homing)

            desired_azimuth = 354
            harness.remote.cmd_moveAzimuth.set(azimuth=desired_azimuth)
            await harness.remote.cmd_moveAzimuth.start(timeout=2)

            # wait for the move to begin and check status
            az_state = await harness.remote.evt_azimuthState.next(flush=False, timeout=1)
            self.assertEqual(az_state.state, SALPY_ATDome.ATDome_shared_AzimuthState_MovingCCW)
            self.assertFalse(az_state.homing)
            position = harness.remote.tel_position.get()
            self.assertAlmostEqual(position.azimuthPositionSet, desired_azimuth)
            self.assertGreater(position.azimuthPosition, desired_azimuth)
            self.assertLess(position.azimuthPosition, 360)

            # wait for the move to end and check status
            az_state = await harness.remote.evt_azimuthState.next(flush=False, timeout=1)
            self.assertEqual(az_state.state, SALPY_ATDome.ATDome_shared_AzimuthState_NotInMotion)
            self.assertFalse(az_state.homing)
            position = harness.remote.tel_position.get()
            self.assertAlmostEqual(position.azimuthPositionSet, desired_azimuth)
            self.assertAlmostEqual(position.azimuthPosition, desired_azimuth)

            # try several invalid values for azimuth
            for bad_az in (-0.001, 360.001):
                harness.remote.cmd_moveAzimuth.set(azimuth=bad_az)
                with salobj.test_utils.assertRaisesAckError():
                    await harness.remote.cmd_moveAzimuth.start(timeout=2)

            await harness.stop()

        asyncio.get_event_loop().run_until_complete(doit())

    def test_move_shutter(self):
        async def doit():
            harness = Harness(initial_state=salobj.State.ENABLED)
            state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
            self.assertEqual(state.summaryState, salobj.State.ENABLED)
            main_door_state = await harness.remote.evt_mainDoorState.next(flush=False, timeout=2)
            self.assertEqual(main_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Closed)
            dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False, timeout=2)
            self.assertEqual(dropout_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Closed)

            # open both doors
            await harness.remote.cmd_openShutter.start(timeout=2)

            # wait for the move to begin and check status
            self.assertEqual(state.summaryState, salobj.State.ENABLED)
            main_door_state = await harness.remote.evt_mainDoorState.next(flush=False, timeout=1)
            self.assertEqual(main_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Opening)
            dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False, timeout=1)
            self.assertEqual(dropout_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Opening)
            shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False, timeout=1)
            self.assertFalse(shutter_in_pos.inPosition)

            # wait for the move to end and check status
            main_door_state = await harness.remote.evt_mainDoorState.next(flush=False, timeout=2)
            self.assertEqual(main_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Opened)
            dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False, timeout=2)
            self.assertEqual(dropout_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Opened)
            shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False, timeout=2)
            self.assertTrue(shutter_in_pos.inPosition)

            # close both doors
            await harness.remote.cmd_closeShutter.start(timeout=2)

            # wait for the move to begin and check status
            self.assertEqual(state.summaryState, salobj.State.ENABLED)
            main_door_state = await harness.remote.evt_mainDoorState.next(flush=False, timeout=1)
            self.assertEqual(main_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Closing)
            dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False, timeout=1)
            self.assertEqual(dropout_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Closing)
            shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False, timeout=1)
            self.assertFalse(shutter_in_pos.inPosition)

            # wait for the move to end and check status
            main_door_state = await harness.remote.evt_mainDoorState.next(flush=False, timeout=2)
            self.assertEqual(main_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Closed)
            dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False, timeout=2)
            self.assertEqual(dropout_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Closed)
            shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False, timeout=2)
            self.assertTrue(shutter_in_pos.inPosition)

            await harness.stop()

        asyncio.get_event_loop().run_until_complete(doit())

    def test_move_individual_doors(self):
        """Test the commands that move individual shutter doors.

        Test the following restrictions:

        * The dropout door can only be moved if the main door is fully open.
        * The main door cannot be closed unless the dropout door is
          fully open or fully closed.
        """
        async def doit():
            harness = Harness(initial_state=salobj.State.ENABLED)
            state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
            self.assertEqual(state.summaryState, salobj.State.ENABLED)
            main_door_state = await harness.remote.evt_mainDoorState.next(flush=False, timeout=2)
            self.assertEqual(main_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Closed)
            dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False, timeout=2)
            self.assertEqual(dropout_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Closed)

            # check that we cannot open or close the dropout door
            # because the main door is not fully open
            harness.remote.cmd_moveShutterDropoutDoor.set(open=True)
            with salobj.assertRaisesAckError():
                await harness.remote.cmd_moveShutterDropoutDoor.start(timeout=2)
            harness.remote.cmd_moveShutterDropoutDoor.set(open=False)
            with salobj.assertRaisesAckError():
                await harness.remote.cmd_moveShutterDropoutDoor.start(timeout=2)

            # open main door
            harness.remote.cmd_moveShutterMainDoor.set(open=True)
            await harness.remote.cmd_moveShutterMainDoor.start(timeout=2)

            # wait for the move to begin and check status
            main_door_state = await harness.remote.evt_mainDoorState.next(flush=False, timeout=1)
            self.assertEqual(main_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Opening)
            shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False, timeout=1)
            self.assertFalse(shutter_in_pos.inPosition)

            # check that we cannot open or close the dropout door
            # because the main door is not fully open
            harness.remote.cmd_moveShutterDropoutDoor.set(open=True)
            with salobj.assertRaisesAckError():
                await harness.remote.cmd_moveShutterDropoutDoor.start(timeout=2)
            harness.remote.cmd_moveShutterDropoutDoor.set(open=False)
            with salobj.assertRaisesAckError():
                await harness.remote.cmd_moveShutterDropoutDoor.start(timeout=2)

            # wait for the move to end and check status
            main_door_state = await harness.remote.evt_mainDoorState.next(flush=False, timeout=2)
            self.assertEqual(main_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Opened)

            # open the dropout door
            harness.remote.cmd_moveShutterDropoutDoor.set(open=True)
            await harness.remote.cmd_moveShutterDropoutDoor.start(timeout=2)
            dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False, timeout=1)
            self.assertEqual(dropout_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Opening)

            # make sure we can't close the main door
            # while the dropout door is moving
            harness.remote.cmd_moveShutterMainDoor.set(open=False)
            with salobj.assertRaisesAckError():
                await harness.remote.cmd_moveShutterMainDoor.start(timeout=2)

            # open the main door again (though we are already there)
            harness.remote.cmd_moveShutterMainDoor.set(open=True)
            await harness.remote.cmd_moveShutterMainDoor.start(timeout=2)

            # wait for the dropout door move to finish
            dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False, timeout=1)
            self.assertEqual(dropout_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Opened)
            # both doors are in their commanded position so shutter is in pos.
            shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False, timeout=1)
            self.assertTrue(shutter_in_pos.inPosition)

            # close the main door
            harness.remote.cmd_moveShutterMainDoor.set(open=False)
            await harness.remote.cmd_moveShutterMainDoor.start(timeout=2)

            main_door_state = await harness.remote.evt_mainDoorState.next(flush=False, timeout=1)
            self.assertEqual(main_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Closing)
            shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False, timeout=1)
            self.assertFalse(shutter_in_pos.inPosition)

            # check that we cannot open or close the dropout door
            # because the main door is not fully open
            harness.remote.cmd_moveShutterDropoutDoor.set(open=True)
            with salobj.assertRaisesAckError():
                await harness.remote.cmd_moveShutterDropoutDoor.start(timeout=2)
            harness.remote.cmd_moveShutterDropoutDoor.set(open=False)
            with salobj.assertRaisesAckError():
                await harness.remote.cmd_moveShutterDropoutDoor.start(timeout=2)

            # wait for the main door to finish closing
            main_door_state = await harness.remote.evt_mainDoorState.next(flush=False, timeout=2)
            self.assertEqual(main_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Closed)
            shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False, timeout=1)
            self.assertTrue(shutter_in_pos.inPosition)

            # open the main door again and this time
            # close the dropout door once we are there
            harness.remote.cmd_moveShutterMainDoor.set(open=True)
            await harness.remote.cmd_moveShutterMainDoor.start(timeout=2)
            main_door_state = await harness.remote.evt_mainDoorState.next(flush=False, timeout=1)
            self.assertEqual(main_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Opening)
            shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False, timeout=1)
            self.assertFalse(shutter_in_pos.inPosition)
            main_door_state = await harness.remote.evt_mainDoorState.next(flush=False, timeout=1)
            self.assertEqual(main_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Opened)
            shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False, timeout=1)
            self.assertTrue(shutter_in_pos.inPosition)

            # close the dropout door
            harness.remote.cmd_moveShutterDropoutDoor.set(open=False)
            await harness.remote.cmd_moveShutterDropoutDoor.start(timeout=2)
            dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False, timeout=1)
            self.assertEqual(dropout_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Closing)
            shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False, timeout=1)
            self.assertFalse(shutter_in_pos.inPosition)

            # make sure we can't close the main door
            # while the dropout door is moving
            harness.remote.cmd_moveShutterMainDoor.set(open=False)
            with salobj.assertRaisesAckError():
                await harness.remote.cmd_moveShutterMainDoor.start(timeout=2)

            # wait for the dropout door to finish closing
            dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False, timeout=1)
            self.assertEqual(dropout_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Closed)
            shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False, timeout=1)
            self.assertTrue(shutter_in_pos.inPosition)

            # close the main door
            harness.remote.cmd_moveShutterMainDoor.set(open=False)
            await harness.remote.cmd_moveShutterMainDoor.start(timeout=2)

            main_door_state = await harness.remote.evt_mainDoorState.next(flush=False, timeout=1)
            self.assertEqual(main_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Closing)
            shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False, timeout=1)
            self.assertFalse(shutter_in_pos.inPosition)

            # check that we cannot open or close the dropout door
            # because the main door is not fully open
            harness.remote.cmd_moveShutterDropoutDoor.set(open=True)
            with salobj.assertRaisesAckError():
                await harness.remote.cmd_moveShutterDropoutDoor.start(timeout=2)
            harness.remote.cmd_moveShutterDropoutDoor.set(open=False)
            with salobj.assertRaisesAckError():
                await harness.remote.cmd_moveShutterDropoutDoor.start(timeout=2)

            # wait for the main door to finish closing
            main_door_state = await harness.remote.evt_mainDoorState.next(flush=False, timeout=2)
            self.assertEqual(main_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Closed)
            shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False, timeout=1)
            self.assertTrue(shutter_in_pos.inPosition)

            await harness.stop()

        asyncio.get_event_loop().run_until_complete(doit())

    def test_stop(self):
        async def doit():
            harness = Harness(initial_state=salobj.State.ENABLED)
            state = await harness.remote.evt_summaryState.next(flush=False, timeout=5)
            self.assertEqual(state.summaryState, salobj.State.ENABLED)
            az_state = await harness.remote.evt_azimuthState.next(flush=False, timeout=1)
            self.assertEqual(az_state.state, SALPY_ATDome.ATDome_shared_AzimuthState_NotInMotion)
            self.assertFalse(az_state.homing)
            main_door_state = await harness.remote.evt_mainDoorState.next(flush=False, timeout=2)
            self.assertEqual(main_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Closed)
            dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False, timeout=2)
            self.assertEqual(dropout_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Closed)

            # move azimuth and open both doors
            harness.remote.cmd_moveAzimuth.set(azimuth=354)
            await harness.remote.cmd_moveAzimuth.start(timeout=2)
            await harness.remote.cmd_openShutter.start(timeout=2)

            # wait for the moves to start
            az_state = await harness.remote.evt_azimuthState.next(flush=False, timeout=1)
            self.assertEqual(az_state.state, SALPY_ATDome.ATDome_shared_AzimuthState_MovingCCW)
            self.assertFalse(az_state.homing)
            az_in_position = await harness.remote.evt_azimuthInPosition.next(flush=False, timeout=1)
            self.assertFalse(az_in_position.inPosition)
            main_door_state = await harness.remote.evt_mainDoorState.next(flush=False, timeout=2)
            self.assertEqual(main_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Opening)
            dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False, timeout=2)
            self.assertEqual(dropout_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_Opening)
            shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False, timeout=1)
            self.assertFalse(shutter_in_pos.inPosition)

            # stop all motion
            # this should not produce new "inPosition" events, because
            # motion is stopped while the axes are still not in position
            await harness.remote.cmd_stopMotion.start(timeout=2)

            az_state = await harness.remote.evt_azimuthState.next(flush=False, timeout=1)
            self.assertEqual(az_state.state, SALPY_ATDome.ATDome_shared_AzimuthState_NotInMotion)
            self.assertFalse(az_state.homing)
            with self.assertRaises(asyncio.TimeoutError):
                await harness.remote.evt_azimuthInPosition.next(flush=False, timeout=0.1)
            main_door_state = await harness.remote.evt_mainDoorState.next(flush=False, timeout=2)
            self.assertEqual(main_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_PartiallyOpened)
            dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False, timeout=2)
            self.assertEqual(dropout_door_state.state,
                             SALPY_ATDome.ATDome_shared_ShutterDoorState_PartiallyOpened)
            with self.assertRaises(asyncio.TimeoutError):
                await harness.remote.evt_shutterInPosition.next(flush=False, timeout=0.1)

            await harness.stop()

        asyncio.get_event_loop().run_until_complete(doit())

    def test_run(self):
        salobj.test_utils.set_random_lsst_dds_domain()
        exe_name = "run_atdome.py"
        exe_path = shutil.which(exe_name)
        if exe_path is None:
            self.fail(f"Could not find bin script {exe_name}; did you setup and scons this package?")

        async def doit():
            process = await asyncio.create_subprocess_exec(exe_name)
            try:
                remote = salobj.Remote(SALPY_ATDome, index=1)
                summaryState_data = await remote.evt_summaryState.next(flush=False, timeout=10)
                self.assertEqual(summaryState_data.summaryState, salobj.State.STANDBY)

                id_ack = await remote.cmd_exitControl.start(timeout=2)
                self.assertEqual(id_ack.ack.ack, remote.salinfo.lib.SAL__CMD_COMPLETE)
                summaryState_data = await remote.evt_summaryState.next(flush=False, timeout=10)
                self.assertEqual(summaryState_data.summaryState, salobj.State.OFFLINE)

                await asyncio.wait_for(process.wait(), 5)
            except Exception:
                if process.returncode is None:
                    process.terminate()
                raise

        asyncio.get_event_loop().run_until_complete(doit())


if __name__ == "__main__":
    unittest.main()

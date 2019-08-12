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
import glob
import math
import os
import pathlib
import shutil
import unittest
import yaml

from astropy.coordinates import Angle
import astropy.units as u

from lsst.ts import salobj
from lsst.ts.idl.enums.ATDome import AzimuthCommandedState, AzimuthState, \
    ShutterDoorCommandedState, ShutterDoorState
from lsst.ts import ATDome

STD_TIMEOUT = 2  # standard command timeout (sec)
LONG_TIMEOUT = 20  # timeout for starting SAL components (sec)
TEST_CONFIG_DIR = pathlib.Path(__file__).parents[1].joinpath("tests", "data", "config")

port_generator = salobj.index_generator(imin=3200)


class Harness:
    def __init__(self, initial_state, config_dir=None):
        salobj.test_utils.set_random_lsst_dds_domain()
        self.csc = ATDome.ATDomeCsc(
            config_dir=config_dir,
            initial_state=initial_state,
            initial_simulation_mode=1,
            mock_port=next(port_generator))
        self.remote = salobj.Remote(domain=self.csc.domain, name="ATDome", index=0)

    async def __aenter__(self):
        await self.csc.start_task
        await self.remote.start_task
        return self

    async def __aexit__(self, *args):
        await self.csc.close()


class CscTestCase(unittest.TestCase):
    def setUp(self):
        print()

    def test_initial_info(self):

        async def doit():
            async with Harness(initial_state=salobj.State.ENABLED) as harness:
                state = await harness.remote.evt_summaryState.next(flush=False, timeout=LONG_TIMEOUT)
                self.assertEqual(state.summaryState, salobj.State.ENABLED)

                azimuthCmdState = await harness.remote.evt_azimuthCommandedState.next(flush=False,
                                                                                      timeout=STD_TIMEOUT)
                self.assertEqual(azimuthCmdState.commandedState, AzimuthCommandedState.UNKNOWN)
                self.assertTrue(math.isnan(azimuthCmdState.azimuth))

                dropoutCmdState = await harness.remote.evt_dropoutDoorCommandedState.next(flush=False,
                                                                                          timeout=STD_TIMEOUT)
                self.assertEqual(dropoutCmdState.commandedState, ShutterDoorCommandedState.UNKNOWN)
                mainCmdState = await harness.remote.evt_mainDoorCommandedState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(mainCmdState.commandedState, ShutterDoorCommandedState.UNKNOWN)

                az_state = await harness.remote.evt_azimuthState.next(flush=False, timeout=STD_TIMEOUT)
                self.assertEqual(az_state.state, AzimuthState.NOTINMOTION)
                self.assertFalse(az_state.homing)

                main_door_state = await harness.remote.evt_mainDoorState.next(flush=False,
                                                                              timeout=STD_TIMEOUT)
                self.assertEqual(main_door_state.state, ShutterDoorState.CLOSED)
                dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(dropout_door_state.state,
                                 ShutterDoorState.CLOSED)
                emergency_stop = await harness.remote.evt_emergencyStop.next(flush=False, timeout=STD_TIMEOUT)
                self.assertFalse(emergency_stop.active)

                position = await harness.remote.tel_position.next(flush=False, timeout=STD_TIMEOUT)
                self.assertEqual(position.dropoutDoorOpeningPercentage, 0)
                self.assertEqual(position.mainDoorOpeningPercentage, 0)
                self.assertAlmostEqual(position.azimuthPosition, 0)

                tcp_settings = await harness.remote.evt_settingsAppliedDomeTcp.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(tcp_settings.host, harness.csc.config.host)
                self.assertEqual(tcp_settings.port, harness.csc.config.port)
                self.assertEqual(tcp_settings.connectionTimeout, harness.csc.config.connection_timeout)
                self.assertEqual(tcp_settings.readTimeout, harness.csc.config.read_timeout)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_default_config_dir(self):
        async def doit():
            async with Harness(initial_state=salobj.State.STANDBY) as harness:
                self.assertEqual(harness.csc.summary_state, salobj.State.STANDBY)

                desired_config_pkg_name = "ts_config_attcs"
                desired_config_env_name = desired_config_pkg_name.upper() + "_DIR"
                desird_config_pkg_dir = os.environ[desired_config_env_name]
                desired_config_dir = pathlib.Path(desird_config_pkg_dir) / "ATDome/v1"
                self.assertEqual(harness.csc.get_config_pkg(), desired_config_pkg_name)
                self.assertEqual(harness.csc.config_dir, desired_config_dir)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_configuration(self):
        async def doit():
            async with Harness(initial_state=salobj.State.STANDBY, config_dir=TEST_CONFIG_DIR) as harness:
                self.assertEqual(harness.csc.summary_state, salobj.State.STANDBY)
                state = await harness.remote.evt_summaryState.next(flush=False, timeout=LONG_TIMEOUT)
                self.assertEqual(state.summaryState, salobj.State.STANDBY)

                invalid_files = glob.glob(os.path.join(TEST_CONFIG_DIR, "invalid_*.yaml"))
                bad_config_names = [os.path.basename(name) for name in invalid_files]
                bad_config_names.append("no_such_file.yaml")
                for bad_config_name in bad_config_names:
                    with self.subTest(bad_config_name=bad_config_name):
                        harness.remote.cmd_start.set(settingsToApply=bad_config_name)
                        with salobj.test_utils.assertRaisesAckError():
                            await harness.remote.cmd_start.start(timeout=STD_TIMEOUT)

                harness.remote.cmd_start.set(settingsToApply="all_fields")
                await harness.remote.cmd_start.start(timeout=STD_TIMEOUT)
                self.assertEqual(harness.csc.summary_state, salobj.State.DISABLED)
                state = await harness.remote.evt_summaryState.next(flush=False, timeout=STD_TIMEOUT)
                self.assertEqual(state.summaryState, salobj.State.DISABLED)
                all_fields_path = os.path.join(TEST_CONFIG_DIR, "all_fields.yaml")
                with open(all_fields_path, "r") as f:
                    all_fields_raw = f.read()
                all_fields_data = yaml.safe_load(all_fields_raw)
                for field, value in all_fields_data.items():
                    self.assertEqual(getattr(harness.csc.config, field), value)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_home(self):
        async def doit():
            async with Harness(initial_state=salobj.State.ENABLED) as harness:
                state = await harness.remote.evt_summaryState.next(flush=False, timeout=LONG_TIMEOUT)
                self.assertEqual(state.summaryState, salobj.State.ENABLED)

                cmdState = await harness.remote.evt_azimuthCommandedState.next(flush=False,
                                                                               timeout=STD_TIMEOUT)
                self.assertEqual(cmdState.commandedState, AzimuthCommandedState.UNKNOWN)
                self.assertTrue(math.isnan(cmdState.azimuth))

                az_state = await harness.remote.evt_azimuthState.next(flush=False, timeout=STD_TIMEOUT)
                self.assertEqual(az_state.state, AzimuthState.NOTINMOTION)
                self.assertFalse(az_state.homing)

                # set home azimuth near current position so homing goes quickly
                curr_az = harness.csc.mock_ctrl.az_actuator.curr_pos
                home_azimuth = (curr_az - 2*u.deg).wrap_at(Angle(360, u.deg))
                harness.csc.mock_ctrl.home_az = home_azimuth

                await harness.remote.cmd_homeAzimuth.start(timeout=STD_TIMEOUT)

                # wait for homing to begin and check status
                cmdState = await harness.remote.evt_azimuthCommandedState.next(flush=False,
                                                                               timeout=STD_TIMEOUT)
                self.assertEqual(cmdState.commandedState, AzimuthCommandedState.HOME)
                self.assertTrue(math.isnan(cmdState.azimuth))
                az_state = await harness.remote.evt_azimuthState.next(flush=False, timeout=STD_TIMEOUT)
                self.assertEqual(az_state.state, AzimuthState.MOVINGCCW)
                self.assertTrue(az_state.homing)
                position = harness.remote.tel_position.get()
                self.assertGreater(position.azimuthPosition, home_azimuth.deg)

                # check that moveAzimuth is disallowed while homing
                harness.remote.cmd_moveAzimuth.set(azimuth=0)  # arbitrary
                with salobj.assertRaisesAckError():
                    await harness.remote.cmd_moveAzimuth.start(timeout=STD_TIMEOUT)

                # check that homing is disallowed while homing
                harness.remote.cmd_moveAzimuth.set(azimuth=0)  # arbitrary
                with salobj.assertRaisesAckError():
                    await harness.remote.cmd_homeAzimuth.start(timeout=STD_TIMEOUT)

                # wait for the initial CCW homing move to finish
                az_state = await harness.remote.evt_azimuthState.next(flush=False, timeout=STD_TIMEOUT)
                self.assertTrue(az_state.homing)
                self.assertEqual(az_state.state, AzimuthState.MOVINGCW)
                self.assertAlmostEqual(harness.csc.mock_ctrl.az_actuator.speed.deg,
                                       harness.csc.mock_ctrl.home_az_vel.deg)

                # wait for the slow CW homing move to finish
                az_state = await harness.remote.evt_azimuthState.next(flush=False, timeout=STD_TIMEOUT)
                self.assertFalse(az_state.homing)
                self.assertEqual(az_state.state, AzimuthState.NOTINMOTION)
                self.assertAlmostEqual(harness.csc.mock_ctrl.az_actuator.speed.deg,
                                       harness.csc.mock_ctrl.az_vel.deg)
                position = harness.remote.tel_position.get()
                self.assertAlmostEqual(position.azimuthPosition, home_azimuth.deg)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_move_az(self):
        async def doit():
            async with Harness(initial_state=salobj.State.ENABLED) as harness:
                state = await harness.remote.evt_summaryState.next(flush=False, timeout=LONG_TIMEOUT)
                self.assertEqual(state.summaryState, salobj.State.ENABLED)
                cmdState = await harness.remote.evt_azimuthCommandedState.next(flush=False,
                                                                               timeout=STD_TIMEOUT)
                self.assertEqual(cmdState.commandedState, AzimuthCommandedState.UNKNOWN)
                self.assertTrue(math.isnan(cmdState.azimuth))

                az_state = await harness.remote.evt_azimuthState.next(flush=False, timeout=STD_TIMEOUT)
                self.assertEqual(az_state.state, AzimuthState.NOTINMOTION)
                self.assertFalse(az_state.homing)

                desired_azimuth = 354
                harness.remote.cmd_moveAzimuth.set(azimuth=desired_azimuth)
                await harness.remote.cmd_moveAzimuth.start(timeout=STD_TIMEOUT)

                # wait for the move to begin and check status
                cmdState = await harness.remote.evt_azimuthCommandedState.next(flush=False,
                                                                               timeout=STD_TIMEOUT)
                self.assertEqual(cmdState.commandedState, AzimuthCommandedState.GOTOPOSITION)
                self.assertAlmostEqual(cmdState.azimuth, desired_azimuth)
                az_state = await harness.remote.evt_azimuthState.next(flush=False, timeout=STD_TIMEOUT)
                self.assertEqual(az_state.state, AzimuthState.MOVINGCCW)
                self.assertFalse(az_state.homing)
                position = harness.remote.tel_position.get()
                self.assertGreater(position.azimuthPosition, desired_azimuth)
                self.assertLess(position.azimuthPosition, 360)

                # wait for the move to end and check status
                az_state = await harness.remote.evt_azimuthState.next(flush=False, timeout=STD_TIMEOUT)
                self.assertEqual(az_state.state, AzimuthState.NOTINMOTION)
                self.assertFalse(az_state.homing)
                position = harness.remote.tel_position.get()
                self.assertAlmostEqual(position.azimuthPosition, desired_azimuth)

                # try several invalid values for azimuth
                for bad_az in (-0.001, 360.001):
                    harness.remote.cmd_moveAzimuth.set(azimuth=bad_az)
                    with salobj.test_utils.assertRaisesAckError():
                        await harness.remote.cmd_moveAzimuth.start(timeout=STD_TIMEOUT)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_move_shutter(self):
        async def doit():
            async with Harness(initial_state=salobj.State.ENABLED) as harness:
                state = await harness.remote.evt_summaryState.next(flush=False, timeout=LONG_TIMEOUT)
                self.assertEqual(state.summaryState, salobj.State.ENABLED)

                dropoutCmdState = await harness.remote.evt_dropoutDoorCommandedState.next(flush=False,
                                                                                          timeout=STD_TIMEOUT)
                self.assertEqual(dropoutCmdState.commandedState, ShutterDoorCommandedState.UNKNOWN)
                mainCmdState = await harness.remote.evt_mainDoorCommandedState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(mainCmdState.commandedState, ShutterDoorCommandedState.UNKNOWN)

                main_door_state = await harness.remote.evt_mainDoorState.next(flush=False,
                                                                              timeout=STD_TIMEOUT)
                self.assertEqual(main_door_state.state,
                                 ShutterDoorState.CLOSED)
                dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(dropout_door_state.state,
                                 ShutterDoorState.CLOSED)

                # open both doors
                await harness.remote.cmd_openShutter.start(timeout=STD_TIMEOUT)

                # wait for the move to begin and check status
                dropoutCmdState = await harness.remote.evt_dropoutDoorCommandedState.next(flush=False,
                                                                                          timeout=STD_TIMEOUT)
                self.assertEqual(dropoutCmdState.commandedState, ShutterDoorCommandedState.OPENED)
                mainCmdState = await harness.remote.evt_mainDoorCommandedState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(mainCmdState.commandedState, ShutterDoorCommandedState.OPENED)

                main_door_state = await harness.remote.evt_mainDoorState.next(flush=False,
                                                                              timeout=STD_TIMEOUT)
                self.assertEqual(main_door_state.state,
                                 ShutterDoorState.OPENING)
                dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(dropout_door_state.state,
                                 ShutterDoorState.OPENING)
                shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False,
                                                                                 timeout=STD_TIMEOUT)
                self.assertFalse(shutter_in_pos.inPosition)

                # wait for the move to end and check status
                main_door_state = await harness.remote.evt_mainDoorState.next(flush=False,
                                                                              timeout=STD_TIMEOUT)
                self.assertEqual(main_door_state.state,
                                 ShutterDoorState.OPENED)
                dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(dropout_door_state.state,
                                 ShutterDoorState.OPENED)
                shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False,
                                                                                 timeout=STD_TIMEOUT)
                self.assertTrue(shutter_in_pos.inPosition)

                # close both doors
                await harness.remote.cmd_closeShutter.start(timeout=STD_TIMEOUT)

                # wait for the move to begin and check status
                dropoutCmdState = await harness.remote.evt_dropoutDoorCommandedState.next(flush=False,
                                                                                          timeout=STD_TIMEOUT)
                self.assertEqual(dropoutCmdState.commandedState, ShutterDoorCommandedState.CLOSED)
                mainCmdState = await harness.remote.evt_mainDoorCommandedState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(mainCmdState.commandedState, ShutterDoorCommandedState.CLOSED)

                self.assertEqual(state.summaryState, salobj.State.ENABLED)
                main_door_state = await harness.remote.evt_mainDoorState.next(flush=False,
                                                                              timeout=STD_TIMEOUT)
                self.assertEqual(main_door_state.state,
                                 ShutterDoorState.CLOSING)
                dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(dropout_door_state.state,
                                 ShutterDoorState.CLOSING)
                shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False,
                                                                                 timeout=STD_TIMEOUT)
                self.assertFalse(shutter_in_pos.inPosition)

                # wait for the move to end and check status
                main_door_state = await harness.remote.evt_mainDoorState.next(flush=False,
                                                                              timeout=STD_TIMEOUT)
                self.assertEqual(main_door_state.state,
                                 ShutterDoorState.CLOSED)
                dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(dropout_door_state.state,
                                 ShutterDoorState.CLOSED)
                shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False,
                                                                                 timeout=STD_TIMEOUT)
                self.assertTrue(shutter_in_pos.inPosition)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_move_individual_doors(self):
        """Test the commands that move individual shutter doors.

        Test the following restrictions:

        * The dropout door can only be moved if the main door is fully open.
        * The main door cannot be closed unless the dropout door is
          fully open or fully closed.
        """
        async def doit():
            async with Harness(initial_state=salobj.State.ENABLED) as harness:
                state = await harness.remote.evt_summaryState.next(flush=False, timeout=LONG_TIMEOUT)
                self.assertEqual(state.summaryState, salobj.State.ENABLED)

                dropoutCmdState = await harness.remote.evt_dropoutDoorCommandedState.next(flush=False,
                                                                                          timeout=STD_TIMEOUT)
                self.assertEqual(dropoutCmdState.commandedState, ShutterDoorCommandedState.UNKNOWN)
                mainCmdState = await harness.remote.evt_mainDoorCommandedState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(mainCmdState.commandedState, ShutterDoorCommandedState.UNKNOWN)

                main_door_state = await harness.remote.evt_mainDoorState.next(flush=False,
                                                                              timeout=STD_TIMEOUT)
                self.assertEqual(main_door_state.state,
                                 ShutterDoorState.CLOSED)
                dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(dropout_door_state.state,
                                 ShutterDoorState.CLOSED)

                # check that we cannot open or close the dropout door
                # because the main door is not fully open
                harness.remote.cmd_moveShutterDropoutDoor.set(open=True)
                with salobj.assertRaisesAckError():
                    await harness.remote.cmd_moveShutterDropoutDoor.start(timeout=STD_TIMEOUT)
                harness.remote.cmd_moveShutterDropoutDoor.set(open=False)
                with salobj.assertRaisesAckError():
                    await harness.remote.cmd_moveShutterDropoutDoor.start(timeout=STD_TIMEOUT)

                # open main door
                harness.remote.cmd_moveShutterMainDoor.set(open=True)
                await harness.remote.cmd_moveShutterMainDoor.start(timeout=STD_TIMEOUT)

                # wait for the move to begin and check status
                mainCmdState = await harness.remote.evt_mainDoorCommandedState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(mainCmdState.commandedState, ShutterDoorCommandedState.OPENED)

                main_door_state = await harness.remote.evt_mainDoorState.next(flush=False,
                                                                              timeout=STD_TIMEOUT)
                self.assertEqual(main_door_state.state,
                                 ShutterDoorState.OPENING)
                shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False,
                                                                                 timeout=STD_TIMEOUT)
                self.assertFalse(shutter_in_pos.inPosition)

                # check that we cannot open or close the dropout door
                # because the main door is not fully open
                harness.remote.cmd_moveShutterDropoutDoor.set(open=True)
                with salobj.assertRaisesAckError():
                    await harness.remote.cmd_moveShutterDropoutDoor.start(timeout=STD_TIMEOUT)
                harness.remote.cmd_moveShutterDropoutDoor.set(open=False)
                with salobj.assertRaisesAckError():
                    await harness.remote.cmd_moveShutterDropoutDoor.start(timeout=STD_TIMEOUT)

                # wait for the move to end and check status
                main_door_state = await harness.remote.evt_mainDoorState.next(flush=False,
                                                                              timeout=STD_TIMEOUT)
                self.assertEqual(main_door_state.state,
                                 ShutterDoorState.OPENED)

                # open the dropout door
                harness.remote.cmd_moveShutterDropoutDoor.set(open=True)
                await harness.remote.cmd_moveShutterDropoutDoor.start(timeout=STD_TIMEOUT)

                dropoutCmdState = await harness.remote.evt_dropoutDoorCommandedState.next(flush=False,
                                                                                          timeout=STD_TIMEOUT)
                self.assertEqual(dropoutCmdState.commandedState, ShutterDoorCommandedState.OPENED)

                dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(dropout_door_state.state,
                                 ShutterDoorState.OPENING)

                # make sure we can't close the main door
                # while the dropout door is moving
                harness.remote.cmd_moveShutterMainDoor.set(open=False)
                with salobj.assertRaisesAckError():
                    await harness.remote.cmd_moveShutterMainDoor.start(timeout=STD_TIMEOUT)

                # open the main door again (though we are already there)
                harness.remote.cmd_moveShutterMainDoor.set(open=True)
                await harness.remote.cmd_moveShutterMainDoor.start(timeout=STD_TIMEOUT)

                mainCmdState = await harness.remote.evt_mainDoorCommandedState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(mainCmdState.commandedState, ShutterDoorCommandedState.OPENED)

                # wait for the dropout door move to finish
                dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(dropout_door_state.state,
                                 ShutterDoorState.OPENED)
                # both doors are in their commanded position
                # so shutter is in position
                shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False,
                                                                                 timeout=STD_TIMEOUT)
                self.assertTrue(shutter_in_pos.inPosition)

                # close the main door
                harness.remote.cmd_moveShutterMainDoor.set(open=False)
                await harness.remote.cmd_moveShutterMainDoor.start(timeout=STD_TIMEOUT)

                mainCmdState = await harness.remote.evt_mainDoorCommandedState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(mainCmdState.commandedState, ShutterDoorCommandedState.CLOSED)

                main_door_state = await harness.remote.evt_mainDoorState.next(flush=False,
                                                                              timeout=STD_TIMEOUT)
                self.assertEqual(main_door_state.state,
                                 ShutterDoorState.CLOSING)
                shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False,
                                                                                 timeout=STD_TIMEOUT)
                self.assertFalse(shutter_in_pos.inPosition)

                # check that we cannot open or close the dropout door
                # because the main door is not fully open
                harness.remote.cmd_moveShutterDropoutDoor.set(open=True)
                with salobj.assertRaisesAckError():
                    await harness.remote.cmd_moveShutterDropoutDoor.start(timeout=STD_TIMEOUT)
                harness.remote.cmd_moveShutterDropoutDoor.set(open=False)
                with salobj.assertRaisesAckError():
                    await harness.remote.cmd_moveShutterDropoutDoor.start(timeout=STD_TIMEOUT)

                # wait for the main door to finish closing
                main_door_state = await harness.remote.evt_mainDoorState.next(flush=False,
                                                                              timeout=STD_TIMEOUT)
                self.assertEqual(main_door_state.state,
                                 ShutterDoorState.CLOSED)
                shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False,
                                                                                 timeout=STD_TIMEOUT)
                self.assertTrue(shutter_in_pos.inPosition)

                # open the main door again and this time
                # close the dropout door once we are there
                harness.remote.cmd_moveShutterMainDoor.set(open=True)
                await harness.remote.cmd_moveShutterMainDoor.start(timeout=STD_TIMEOUT)

                mainCmdState = await harness.remote.evt_mainDoorCommandedState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(mainCmdState.commandedState, ShutterDoorCommandedState.OPENED)

                main_door_state = await harness.remote.evt_mainDoorState.next(flush=False,
                                                                              timeout=STD_TIMEOUT)
                self.assertEqual(main_door_state.state,
                                 ShutterDoorState.OPENING)
                shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False,
                                                                                 timeout=STD_TIMEOUT)
                self.assertFalse(shutter_in_pos.inPosition)
                main_door_state = await harness.remote.evt_mainDoorState.next(flush=False,
                                                                              timeout=STD_TIMEOUT)
                self.assertEqual(main_door_state.state,
                                 ShutterDoorState.OPENED)
                shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False,
                                                                                 timeout=STD_TIMEOUT)
                self.assertTrue(shutter_in_pos.inPosition)

                # close the dropout door
                harness.remote.cmd_moveShutterDropoutDoor.set(open=False)
                await harness.remote.cmd_moveShutterDropoutDoor.start(timeout=STD_TIMEOUT)

                dropoutCmdState = await harness.remote.evt_dropoutDoorCommandedState.next(flush=False,
                                                                                          timeout=STD_TIMEOUT)
                self.assertEqual(dropoutCmdState.commandedState, ShutterDoorCommandedState.CLOSED)

                dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(dropout_door_state.state,
                                 ShutterDoorState.CLOSING)
                shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False,
                                                                                 timeout=STD_TIMEOUT)
                self.assertFalse(shutter_in_pos.inPosition)

                # make sure we can't close the main door
                # while the dropout door is moving
                harness.remote.cmd_moveShutterMainDoor.set(open=False)
                with salobj.assertRaisesAckError():
                    await harness.remote.cmd_moveShutterMainDoor.start(timeout=STD_TIMEOUT)

                # wait for the dropout door to finish closing
                dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(dropout_door_state.state,
                                 ShutterDoorState.CLOSED)
                shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False,
                                                                                 timeout=STD_TIMEOUT)
                self.assertTrue(shutter_in_pos.inPosition)

                # close the main door
                harness.remote.cmd_moveShutterMainDoor.set(open=False)
                await harness.remote.cmd_moveShutterMainDoor.start(timeout=STD_TIMEOUT)

                mainCmdState = await harness.remote.evt_mainDoorCommandedState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(mainCmdState.commandedState, ShutterDoorCommandedState.CLOSED)

                main_door_state = await harness.remote.evt_mainDoorState.next(flush=False,
                                                                              timeout=STD_TIMEOUT)
                self.assertEqual(main_door_state.state,
                                 ShutterDoorState.CLOSING)
                shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False,
                                                                                 timeout=STD_TIMEOUT)
                self.assertFalse(shutter_in_pos.inPosition)

                # check that we cannot open or close the dropout door
                # because the main door is not fully open
                harness.remote.cmd_moveShutterDropoutDoor.set(open=True)
                with salobj.assertRaisesAckError():
                    await harness.remote.cmd_moveShutterDropoutDoor.start(timeout=STD_TIMEOUT)
                harness.remote.cmd_moveShutterDropoutDoor.set(open=False)
                with salobj.assertRaisesAckError():
                    await harness.remote.cmd_moveShutterDropoutDoor.start(timeout=STD_TIMEOUT)

                # wait for the main door to finish closing
                main_door_state = await harness.remote.evt_mainDoorState.next(flush=False,
                                                                              timeout=STD_TIMEOUT)
                self.assertEqual(main_door_state.state,
                                 ShutterDoorState.CLOSED)
                shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False,
                                                                                 timeout=STD_TIMEOUT)
                self.assertTrue(shutter_in_pos.inPosition)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_stop(self):
        async def doit():
            async with Harness(initial_state=salobj.State.ENABLED) as harness:
                state = await harness.remote.evt_summaryState.next(flush=False, timeout=LONG_TIMEOUT)
                self.assertEqual(state.summaryState, salobj.State.ENABLED)
                az_state = await harness.remote.evt_azimuthState.next(flush=False, timeout=STD_TIMEOUT)
                self.assertEqual(az_state.state, AzimuthState.NOTINMOTION)
                self.assertFalse(az_state.homing)
                main_door_state = await harness.remote.evt_mainDoorState.next(flush=False,
                                                                              timeout=STD_TIMEOUT)
                self.assertEqual(main_door_state.state,
                                 ShutterDoorState.CLOSED)
                dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(dropout_door_state.state,
                                 ShutterDoorState.CLOSED)

                # move azimuth and open both doors
                harness.remote.cmd_moveAzimuth.set(azimuth=354)
                await harness.remote.cmd_moveAzimuth.start(timeout=STD_TIMEOUT)
                await harness.remote.cmd_openShutter.start(timeout=STD_TIMEOUT)

                # wait for the moves to start
                az_state = await harness.remote.evt_azimuthState.next(flush=False, timeout=STD_TIMEOUT)
                self.assertEqual(az_state.state, AzimuthState.MOVINGCCW)
                self.assertFalse(az_state.homing)
                az_in_position = await harness.remote.evt_azimuthInPosition.next(flush=False,
                                                                                 timeout=STD_TIMEOUT)
                self.assertFalse(az_in_position.inPosition)
                main_door_state = await harness.remote.evt_mainDoorState.next(flush=False,
                                                                              timeout=STD_TIMEOUT)
                self.assertEqual(main_door_state.state,
                                 ShutterDoorState.OPENING)
                dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(dropout_door_state.state,
                                 ShutterDoorState.OPENING)
                shutter_in_pos = await harness.remote.evt_shutterInPosition.next(flush=False,
                                                                                 timeout=STD_TIMEOUT)
                self.assertFalse(shutter_in_pos.inPosition)

                # stop all motion
                # this should not produce new "inPosition" events, because
                # motion is stopped while the axes are still not in position
                await harness.remote.cmd_stopMotion.start(timeout=STD_TIMEOUT)

                az_state = await harness.remote.evt_azimuthState.next(flush=False, timeout=STD_TIMEOUT)
                self.assertEqual(az_state.state, AzimuthState.NOTINMOTION)
                self.assertFalse(az_state.homing)
                with self.assertRaises(asyncio.TimeoutError):
                    await harness.remote.evt_azimuthInPosition.next(flush=False, timeout=0.1)
                main_door_state = await harness.remote.evt_mainDoorState.next(flush=False,
                                                                              timeout=STD_TIMEOUT)
                self.assertEqual(main_door_state.state,
                                 ShutterDoorState.PARTIALLYOPENED)
                dropout_door_state = await harness.remote.evt_dropoutDoorState.next(flush=False,
                                                                                    timeout=STD_TIMEOUT)
                self.assertEqual(dropout_door_state.state,
                                 ShutterDoorState.PARTIALLYOPENED)
                with self.assertRaises(asyncio.TimeoutError):
                    await harness.remote.evt_shutterInPosition.next(flush=False, timeout=0.1)

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
                async with salobj.Domain() as domain:
                    remote = salobj.Remote(domain=domain, name="ATDome", index=0)
                    summaryState_data = await remote.evt_summaryState.next(flush=False, timeout=LONG_TIMEOUT)
                    self.assertEqual(summaryState_data.summaryState, salobj.State.STANDBY)

                    ack = await remote.cmd_exitControl.start(timeout=STD_TIMEOUT)
                    self.assertEqual(ack.ack, salobj.SalRetCode.CMD_COMPLETE)
                    summaryState_data = await remote.evt_summaryState.next(flush=False, timeout=LONG_TIMEOUT)
                    self.assertEqual(summaryState_data.summaryState, salobj.State.OFFLINE)

                    await asyncio.wait_for(process.wait(), 5)
            except Exception:
                if process.returncode is None:
                    process.terminate()
                raise

        asyncio.get_event_loop().run_until_complete(doit())


if __name__ == "__main__":
    unittest.main()

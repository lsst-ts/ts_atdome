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
import unittest

import asynctest
import yaml

from astropy.coordinates import Angle
import astropy.units as u

from lsst.ts import salobj
from lsst.ts.idl.enums.ATDome import (
    AzimuthCommandedState,
    AzimuthState,
    ShutterDoorCommandedState,
    ShutterDoorState,
)
from lsst.ts import ATDome

STD_TIMEOUT = 2  # standard command timeout (sec)
DOOR_TIMEOUT = 4  # time limit for shutter door commands (sec)
LONG_TIMEOUT = 20  # timeout for starting SAL components (sec)
TEST_CONFIG_DIR = pathlib.Path(__file__).parents[1].joinpath("tests", "data", "config")

port_generator = salobj.index_generator(imin=3200)


class CscTestCase(salobj.BaseCscTestCase, asynctest.TestCase):
    def basic_make_csc(self, initial_state, config_dir, simulation_mode):
        return ATDome.ATDomeCsc(
            initial_state=initial_state,
            config_dir=config_dir,
            simulation_mode=simulation_mode,
            mock_port=next(port_generator),
        )

    async def test_initial_info(self):
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, config_dir=None, simulation_mode=1
        ):
            await self.check_initial_shutter_events()

            await self.check_initial_az_events()

            await self.assert_next_sample(
                topic=self.remote.evt_emergencyStop, active=False
            )

            position = await self.assert_next_sample(
                topic=self.remote.tel_position,
                flush=True,
                dropoutDoorOpeningPercentage=0,
                mainDoorOpeningPercentage=0,
            )
            self.assertAlmostEqual(position.azimuthPosition, 0)

            await self.assert_next_sample(
                topic=self.remote.evt_settingsAppliedDomeTcp,
                host=self.csc.config.host,
                port=self.csc.config.port,
                connectionTimeout=self.csc.config.connection_timeout,
                readTimeout=self.csc.config.read_timeout,
            )

    async def test_default_config_dir(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            self.assertEqual(self.csc.summary_state, salobj.State.STANDBY)
            await self.assert_next_summary_state(salobj.State.STANDBY)

            desired_config_pkg_name = "ts_config_attcs"
            desired_config_env_name = desired_config_pkg_name.upper() + "_DIR"
            desird_config_pkg_dir = os.environ[desired_config_env_name]
            desired_config_dir = pathlib.Path(desird_config_pkg_dir) / "ATDome/v1"
            self.assertEqual(self.csc.get_config_pkg(), desired_config_pkg_name)
            self.assertEqual(self.csc.config_dir, desired_config_dir)

    async def test_configuration(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY,
            config_dir=TEST_CONFIG_DIR,
            simulation_mode=1,
        ):
            self.assertEqual(self.csc.summary_state, salobj.State.STANDBY)
            await self.assert_next_summary_state(salobj.State.STANDBY)

            invalid_files = glob.glob(os.path.join(TEST_CONFIG_DIR, "invalid_*.yaml"))
            bad_config_names = [os.path.basename(name) for name in invalid_files]
            bad_config_names.append("no_such_file.yaml")
            for bad_config_name in bad_config_names:
                with self.subTest(bad_config_name=bad_config_name):
                    with salobj.assertRaisesAckError():
                        await self.remote.cmd_start.set_start(
                            settingsToApply=bad_config_name, timeout=STD_TIMEOUT
                        )

            await self.remote.cmd_start.set_start(
                settingsToApply="all_fields", timeout=STD_TIMEOUT
            )
            self.assertEqual(self.csc.summary_state, salobj.State.DISABLED)
            await self.assert_next_summary_state(salobj.State.DISABLED)
            all_fields_path = os.path.join(TEST_CONFIG_DIR, "all_fields.yaml")
            with open(all_fields_path, "r") as f:
                all_fields_raw = f.read()
            all_fields_data = yaml.safe_load(all_fields_raw)
            for field, value in all_fields_data.items():
                self.assertEqual(getattr(self.csc.config, field), value)

    async def test_home(self):
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, config_dir=None, simulation_mode=1
        ):
            await self.check_initial_az_events()

            # set home azimuth near current position so homing goes quickly
            curr_az = self.csc.mock_ctrl.az_actuator.current_position
            home_azimuth = (curr_az - 2 * u.deg).wrap_at(Angle(360, u.deg))
            self.csc.mock_ctrl.home_az = home_azimuth

            await self.remote.cmd_homeAzimuth.start(timeout=STD_TIMEOUT)

            # wait for homing to begin and check status
            az_cmd_state = await self.assert_next_sample(
                topic=self.remote.evt_azimuthCommandedState,
                commandedState=AzimuthCommandedState.HOME,
            )
            self.assertTrue(math.isnan(az_cmd_state.azimuth))

            await self.assert_next_sample(
                topic=self.remote.evt_azimuthState,
                state=AzimuthState.MOVINGCCW,
                homing=True,
            )

            position = await self.assert_next_sample(
                topic=self.remote.tel_position, flush=True
            )
            self.assertGreater(position.azimuthPosition, home_azimuth.deg)

            # check that moveAzimuth is disallowed while homing
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveAzimuth.set_start(
                    azimuth=0, timeout=STD_TIMEOUT
                )

            # check that homing is disallowed while homing
            with salobj.assertRaisesAckError():
                await self.remote.cmd_homeAzimuth.start(timeout=STD_TIMEOUT)

            # wait for the initial CCW homing move to finish
            await self.assert_next_sample(
                topic=self.remote.evt_azimuthState,
                state=AzimuthState.MOVINGCW,
                homing=True,
            )
            self.assertAlmostEqual(
                self.csc.mock_ctrl.az_actuator.speed.deg,
                self.csc.mock_ctrl.home_az_vel.deg,
            )

            # wait for the slow CW homing move to finish
            await self.assert_next_sample(
                topic=self.remote.evt_azimuthState,
                state=AzimuthState.NOTINMOTION,
                homing=False,
            )
            self.assertAlmostEqual(
                self.csc.mock_ctrl.az_actuator.speed.deg, self.csc.mock_ctrl.az_vel.deg
            )
            position = await self.assert_next_sample(
                topic=self.remote.tel_position, flush=True
            )
            self.assertAlmostEqual(position.azimuthPosition, home_azimuth.deg)

    async def test_move_az(self):
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, config_dir=None, simulation_mode=1
        ):
            await self.check_initial_az_events()

            desired_azimuth = 354
            await self.remote.cmd_moveAzimuth.set_start(
                azimuth=desired_azimuth, timeout=STD_TIMEOUT
            )

            # wait for the move to begin and check status
            az_cmd_state = await self.assert_next_sample(
                topic=self.remote.evt_azimuthCommandedState,
                commandedState=AzimuthCommandedState.GOTOPOSITION,
            )
            self.assertAlmostEqual(az_cmd_state.azimuth, desired_azimuth)
            await self.assert_next_sample(
                topic=self.remote.evt_azimuthState,
                state=AzimuthState.MOVINGCCW,
                homing=False,
            )
            position = self.remote.tel_position.get()
            self.assertGreater(position.azimuthPosition, desired_azimuth)
            self.assertLess(position.azimuthPosition, 360)

            # wait for the move to end and check status
            await self.assert_next_sample(
                topic=self.remote.evt_azimuthState,
                state=AzimuthState.NOTINMOTION,
                homing=False,
            )
            position = self.remote.tel_position.get()
            self.assertAlmostEqual(position.azimuthPosition, desired_azimuth)

            # try several invalid values for azimuth
            for bad_az in (-0.001, 360.001):
                with salobj.assertRaisesAckError():
                    await self.remote.cmd_moveAzimuth.set_start(
                        azimuth=bad_az, timeout=STD_TIMEOUT
                    )

    async def test_move_shutter(self):
        """Test openShutter and closeShutter commands.
        """
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, config_dir=None, simulation_mode=1
        ):
            await self.check_initial_shutter_events()

            # open both doors
            await self.remote.cmd_openShutter.start(timeout=DOOR_TIMEOUT)

            # check opening events; note that shutterInPosition was
            # initially False and is not output again
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.OPENED,
                main_door_cmd_state=ShutterDoorCommandedState.OPENED,
                dropout_door_state=ShutterDoorState.OPENING,
                main_door_state=ShutterDoorState.OPENING,
            )

            # check fully open events
            await self.check_shutter_events(
                dropout_door_state=ShutterDoorState.OPENED,
                main_door_state=ShutterDoorState.OPENED,
                shutter_in_position=True,
            )

            # close both doors
            await self.remote.cmd_closeShutter.start(timeout=DOOR_TIMEOUT)

            # check closing events
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                main_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                dropout_door_state=ShutterDoorState.CLOSING,
                main_door_state=ShutterDoorState.CLOSING,
                shutter_in_position=False,
            )

            # check fully closed events
            await self.check_shutter_events(
                dropout_door_state=ShutterDoorState.CLOSED,
                main_door_state=ShutterDoorState.CLOSED,
                shutter_in_position=True,
            )

    async def test_move_individual_doors(self):
        """Test the commands that move individual shutter doors.

        Test the following restrictions:

        * The dropout door can only be moved if the main door is fully open.
        * The main door cannot be closed unless the dropout door is
          fully open or fully closed.
        """
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, config_dir=None, simulation_mode=1
        ):
            await self.check_initial_shutter_events()

            # check that we cannot open or close the dropout door
            # because the main door is not fully open
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveShutterDropoutDoor.set_start(
                    open=True, timeout=STD_TIMEOUT
                )
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveShutterDropoutDoor.set_start(
                    open=False, timeout=STD_TIMEOUT
                )

            # start opening the main door
            main_open_task = asyncio.ensure_future(
                self.remote.cmd_moveShutterMainDoor.set_start(
                    open=True, timeout=DOOR_TIMEOUT
                )
            )

            # check main door opening status;
            # the dropout door is not moving so no dropout events output,
            # and shutter_in_position remains False so is not output
            await self.check_shutter_events(
                main_door_cmd_state=ShutterDoorCommandedState.OPENED,
                main_door_state=ShutterDoorState.OPENING,
            )

            # check that we cannot open or close the dropout door
            # because the main door is not fully open
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveShutterDropoutDoor.set_start(
                    open=True, timeout=STD_TIMEOUT
                )
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveShutterDropoutDoor.set_start(
                    open=False, timeout=STD_TIMEOUT
                )

            # wait for the move to end
            await main_open_task

            # check main fully open status
            # dropout door has no commanded position so
            # shutter_in_position remains False
            await self.check_shutter_events(main_door_state=ShutterDoorState.OPENED)

            # open the main door again; this should be quick
            await self.remote.cmd_moveShutterMainDoor.set_start(
                open=True, timeout=STD_TIMEOUT
            )

            # the only expected event is mainDoorCmdState
            await self.check_shutter_events(
                main_door_cmd_state=ShutterDoorCommandedState.OPENED
            )

            # start opening the dropout door
            dropout_open_task = asyncio.ensure_future(
                self.remote.cmd_moveShutterDropoutDoor.set_start(
                    open=True, timeout=DOOR_TIMEOUT
                )
            )

            # check opening dropout door status;
            # the main door is not moving so no main events
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.OPENED,
                dropout_door_state=ShutterDoorState.OPENING,
            )

            # make sure we can't close the main door
            # while the dropout door is moving
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveShutterMainDoor.set_start(
                    open=False, timeout=STD_TIMEOUT
                )

            # wait for the dropout door move to finish
            await dropout_open_task

            # check dropout door fully opened
            # finally shutter_in_position should be True
            await self.check_shutter_events(
                dropout_door_state=ShutterDoorState.OPENED, shutter_in_position=True
            )

            # open the dropout door again; this should be quick
            await self.remote.cmd_moveShutterDropoutDoor.set_start(
                open=True, timeout=STD_TIMEOUT
            )

            # the only expected event is dropoutDoorCmdState
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.OPENED
            )

            # start closing the main door
            main_close_task = asyncio.ensure_future(
                self.remote.cmd_moveShutterMainDoor.set_start(
                    open=False, timeout=DOOR_TIMEOUT
                )
            )

            # check main door closing status;
            # the dropout door is not moving so no dropout events output
            await self.check_shutter_events(
                main_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                main_door_state=ShutterDoorState.CLOSING,
                shutter_in_position=False,
            )

            # check that we cannot open or close the dropout door
            # because the main door is not fully open
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveShutterDropoutDoor.set_start(
                    open=True, timeout=STD_TIMEOUT
                )
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveShutterDropoutDoor.set_start(
                    open=False, timeout=STD_TIMEOUT
                )

            # wait for the main door to finish closing
            await main_close_task

            # check main fully closed status
            await self.check_shutter_events(
                main_door_state=ShutterDoorState.CLOSED, shutter_in_position=True
            )

            # close the main door again; this should be quick
            await self.remote.cmd_moveShutterMainDoor.set_start(
                open=False, timeout=STD_TIMEOUT
            )

            # the only expected event is mainDoorCmdState
            await self.check_shutter_events(
                main_door_cmd_state=ShutterDoorCommandedState.CLOSED
            )

            # open the main door
            await self.remote.cmd_moveShutterMainDoor.set_start(
                open=True, timeout=DOOR_TIMEOUT
            )

            # check main door opening status;
            # the dropout door is not moving so no dropout events output
            await self.check_shutter_events(
                main_door_cmd_state=ShutterDoorCommandedState.OPENED,
                main_door_state=ShutterDoorState.OPENING,
                shutter_in_position=False,
            )

            # check main door fully opening status;
            # the dropout door is not moving so no dropout events output
            await self.check_shutter_events(
                main_door_state=ShutterDoorState.OPENED, shutter_in_position=True
            )

            # start closing the dropout door
            dropout_close_task = asyncio.ensure_future(
                self.remote.cmd_moveShutterDropoutDoor.set_start(
                    open=False, timeout=DOOR_TIMEOUT
                )
            )

            # check dropout door closing status;
            # the main door is not moving so no main events output
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                dropout_door_state=ShutterDoorState.CLOSING,
                shutter_in_position=False,
            )

            # make sure we can't close the main door
            # while the dropout door is moving
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveShutterMainDoor.set_start(
                    open=False, timeout=STD_TIMEOUT
                )

            # wait for the dropout door to finish closing
            await dropout_close_task

            # check dropout fully closed status
            await self.check_shutter_events(
                dropout_door_state=ShutterDoorState.CLOSED, shutter_in_position=True
            )

            # close the dropout door again; this should be quick
            await self.remote.cmd_moveShutterDropoutDoor.set_start(
                open=False, timeout=STD_TIMEOUT
            )

            # the only expected event is dropoutDoorCmdState
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.CLOSED
            )

            # start closing the main door
            main_close_task = asyncio.ensure_future(
                self.remote.cmd_moveShutterMainDoor.set_start(
                    open=False, timeout=DOOR_TIMEOUT
                )
            )

            # check main door closing status;
            # the dropout door is not moving so no dropout events output
            await self.check_shutter_events(
                main_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                main_door_state=ShutterDoorState.CLOSING,
                shutter_in_position=False,
            )

            # check that we cannot open or close the dropout door
            # because the main door is not fully open
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveShutterDropoutDoor.set_start(
                    open=True, timeout=STD_TIMEOUT
                )
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveShutterDropoutDoor.set_start(
                    open=False, timeout=STD_TIMEOUT
                )

            # wait for the main door to finish closing
            await main_close_task

            # check main fully closed status
            await self.check_shutter_events(
                main_door_state=ShutterDoorState.CLOSED, shutter_in_position=True
            )

    async def test_close_shutter_supersedes_open_shutter(self):
        """Test that closing the shutter supersedes opening the shutter.
        """
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, config_dir=None, simulation_mode=1
        ):
            await self.check_initial_shutter_events()

            # start opening the both doors
            open_task = asyncio.ensure_future(
                self.remote.cmd_openShutter.start(timeout=DOOR_TIMEOUT)
            )

            # check opening status;
            # shutter_in_position is still False, so not output
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.OPENED,
                dropout_door_state=ShutterDoorState.OPENING,
                main_door_cmd_state=ShutterDoorCommandedState.OPENED,
                main_door_state=ShutterDoorState.OPENING,
            )

            # start closing the shutter
            close_task = asyncio.ensure_future(
                self.remote.cmd_closeShutter.start(timeout=DOOR_TIMEOUT)
            )

            # check that this supersedes opening the shutter
            with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_ABORTED):
                await open_task

            # check status of closing both doors
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                main_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                dropout_door_state=ShutterDoorState.CLOSING,
                main_door_state=ShutterDoorState.CLOSING,
            )

            await close_task

            # check status of both doors fully closed
            await self.check_shutter_events(
                dropout_door_state=ShutterDoorState.CLOSED,
                main_door_state=ShutterDoorState.CLOSED,
                shutter_in_position=True,
            )

    async def test_open_shutter_supersedes_move_dropout(self):
        """Test that opening the shutter supersedes moving the dropout door.
        """
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, config_dir=None, simulation_mode=1
        ):
            await self.check_initial_shutter_events()

            # open the doors, since the main door has to be fully open
            # to move the dropout door
            await self.remote.cmd_openShutter.start(timeout=DOOR_TIMEOUT)

            # check opening status
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.OPENED,
                main_door_cmd_state=ShutterDoorCommandedState.OPENED,
                dropout_door_state=ShutterDoorState.OPENING,
                main_door_state=ShutterDoorState.OPENING,
            )

            # check final open status
            await self.check_shutter_events(
                dropout_door_state=ShutterDoorState.OPENED,
                main_door_state=ShutterDoorState.OPENED,
                shutter_in_position=True,
            )

            # start closing the dropout door
            dropout_close_task = asyncio.ensure_future(
                self.remote.cmd_moveShutterDropoutDoor.set_start(
                    open=False, timeout=DOOR_TIMEOUT
                )
            )

            # check closing dropout status;
            # main door isn't moving so main events are not output
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                dropout_door_state=ShutterDoorState.CLOSING,
                shutter_in_position=False,
            )

            # start opening the shutter
            open_task = asyncio.ensure_future(
                self.remote.cmd_openShutter.start(timeout=DOOR_TIMEOUT)
            )

            # check that opening the shutter supersedes closing dropout door
            with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_ABORTED):
                await dropout_close_task

            # check status of closing both doors;
            # main door was already open but its cmd state is output
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.OPENED,
                main_door_cmd_state=ShutterDoorCommandedState.OPENED,
                dropout_door_state=ShutterDoorState.OPENING,
            )

            await open_task

            # check status of both doors fully closed;
            # the main door never moved
            await self.check_shutter_events(
                dropout_door_state=ShutterDoorState.OPENED, shutter_in_position=True
            )

    async def test_close_shutter_supersedes_move_main(self):
        """Test that closing the shutter supersedes moving the main door.
        """
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, config_dir=None, simulation_mode=1
        ):
            await self.check_initial_shutter_events()

            # start opening the main door
            main_open_task = asyncio.ensure_future(
                self.remote.cmd_moveShutterMainDoor.set_start(
                    open=True, timeout=DOOR_TIMEOUT
                )
            )

            # check opening main status;
            # dropout door isn't moving so dropout events are not output
            # shutter_in_position is still False, so not output
            await self.check_shutter_events(
                main_door_cmd_state=ShutterDoorCommandedState.OPENED,
                main_door_state=ShutterDoorState.OPENING,
            )

            # start closing the shutter;
            # this supersedes opening the main door
            close_task = asyncio.ensure_future(
                self.remote.cmd_closeShutter.start(timeout=DOOR_TIMEOUT)
            )

            with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_ABORTED):
                await main_open_task

            # check status of closing both doors;
            # dropout door was already closed but its cmd state is output
            await self.check_shutter_events(
                main_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                dropout_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                main_door_state=ShutterDoorState.CLOSING,
            )

            await close_task

            # check final closed status; the dropout door never moved
            await self.check_shutter_events(
                main_door_state=ShutterDoorState.CLOSED, shutter_in_position=True
            )

    async def test_open_shutter_supersedes_close_shutter(self):
        """Test that opening the shutter supersedes closing the shutter.
        """
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, config_dir=None, simulation_mode=1
        ):
            await self.check_initial_shutter_events()

            # open both doors
            await self.remote.cmd_openShutter.start(timeout=DOOR_TIMEOUT)

            # check move begins events
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.OPENED,
                main_door_cmd_state=ShutterDoorCommandedState.OPENED,
                dropout_door_state=ShutterDoorState.OPENING,
                main_door_state=ShutterDoorState.OPENING,
            )

            # check move ends events
            await self.check_shutter_events(
                dropout_door_state=ShutterDoorState.OPENED,
                main_door_state=ShutterDoorState.OPENED,
                shutter_in_position=True,
            )

            # start closing both doors
            close_task = asyncio.ensure_future(
                self.remote.cmd_closeShutter.start(timeout=DOOR_TIMEOUT)
            )

            # check move begins events
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                main_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                dropout_door_state=ShutterDoorState.CLOSING,
                main_door_state=ShutterDoorState.CLOSING,
                shutter_in_position=False,
            )

            # start opening both doors (again)
            open_task = asyncio.ensure_future(
                self.remote.cmd_openShutter.start(timeout=DOOR_TIMEOUT)
            )

            # check that the open command superseded the close command
            with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_ABORTED):
                await close_task

            # check opening status;
            # shutter_in_position is still False, so not output
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.OPENED,
                main_door_cmd_state=ShutterDoorCommandedState.OPENED,
                dropout_door_state=ShutterDoorState.OPENING,
                main_door_state=ShutterDoorState.OPENING,
            )

            await open_task

            # check final open status
            await self.check_shutter_events(
                dropout_door_state=ShutterDoorState.OPENED,
                main_door_state=ShutterDoorState.OPENED,
                shutter_in_position=True,
            )

    async def test_disable(self):
        """Test that disable halts az and closes the shutter.

        And that it aborts an outstanding shutter command.
        """
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, config_dir=None, simulation_mode=1
        ):
            await self.check_initial_az_events()
            await self.check_initial_shutter_events()

            # move azimuth and start opening the shutter
            await self.remote.cmd_moveAzimuth.set_start(
                azimuth=354, timeout=STD_TIMEOUT
            )
            shutter_open_task = asyncio.ensure_future(
                self.remote.cmd_openShutter.start(timeout=STD_TIMEOUT)
            )

            # wait for the moves to start
            await self.assert_next_sample(
                topic=self.remote.evt_azimuthState,
                state=AzimuthState.MOVINGCCW,
                homing=False,
            )
            await self.assert_next_sample(
                self.remote.evt_azimuthInPosition, inPosition=False
            )

            # check that the shutter was told to open;
            # shutter_in_position remains false so is not output
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.OPENED,
                main_door_cmd_state=ShutterDoorCommandedState.OPENED,
                dropout_door_state=ShutterDoorState.OPENING,
                main_door_state=ShutterDoorState.OPENING,
            )

            # disable the CSC
            # this should not produce new "inPosition" events, because
            # motion is stopped while the axes are still not in position
            await self.remote.cmd_disable.start(timeout=STD_TIMEOUT)

            # make sure the shutter command was cancelled
            with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_ABORTED):
                await shutter_open_task

            await self.assert_next_sample(
                topic=self.remote.evt_azimuthState,
                state=AzimuthState.NOTINMOTION,
                homing=False,
            )
            with self.assertRaises(asyncio.TimeoutError):
                await self.remote.evt_azimuthInPosition.next(flush=False, timeout=0.1)

            # check that the shutter was told to close
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                main_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                dropout_door_state=ShutterDoorState.CLOSING,
                main_door_state=ShutterDoorState.CLOSING,
            )

            # check that the shutter closed
            await self.check_shutter_events(
                dropout_door_state=ShutterDoorState.CLOSED,
                main_door_state=ShutterDoorState.CLOSED,
                shutter_in_position=True,
            )

    async def test_stop(self):
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, config_dir=None, simulation_mode=1
        ):
            await self.check_initial_az_events()
            await self.check_initial_shutter_events()

            # move azimuth and start opening the shutter
            await self.remote.cmd_moveAzimuth.set_start(
                azimuth=354, timeout=STD_TIMEOUT
            )
            shutter_open_task = asyncio.ensure_future(
                self.remote.cmd_openShutter.start(timeout=STD_TIMEOUT)
            )

            # wait for the moves to start
            await self.assert_next_sample(
                topic=self.remote.evt_azimuthState,
                state=AzimuthState.MOVINGCCW,
                homing=False,
            )
            await self.assert_next_sample(
                self.remote.evt_azimuthInPosition, inPosition=False
            )

            # check that the shutter was told to open;
            # shutter_in_position remains false so is not output
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.OPENED,
                main_door_cmd_state=ShutterDoorCommandedState.OPENED,
                dropout_door_state=ShutterDoorState.OPENING,
                main_door_state=ShutterDoorState.OPENING,
            )

            # stop all motion
            # this should not produce new "inPosition" events, because
            # motion is stopped while the axes are still not in position
            await self.remote.cmd_stopMotion.start(timeout=STD_TIMEOUT)

            # make sure the shutter command was cancelled
            with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_ABORTED):
                await shutter_open_task

            await self.assert_next_sample(
                topic=self.remote.evt_azimuthState,
                state=AzimuthState.NOTINMOTION,
                homing=False,
            )
            with self.assertRaises(asyncio.TimeoutError):
                await self.remote.evt_azimuthInPosition.next(flush=False, timeout=0.1)

            # check that the shutter was told to stop;
            # shutter_in_position remains false so is not output
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.STOP,
                main_door_cmd_state=ShutterDoorCommandedState.STOP,
                dropout_door_state=ShutterDoorState.PARTIALLYOPENED,
                main_door_state=ShutterDoorState.PARTIALLYOPENED,
            )
            with self.assertRaises(asyncio.TimeoutError):
                await self.remote.evt_shutterInPosition.next(flush=False, timeout=0.1)

    async def test_bin_script(self):
        await self.check_bin_script(name="ATDome", index=None, exe_name="run_atdome.py")

    async def test_standard_state_transitions(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            await self.check_standard_state_transitions(
                enabled_commands=(
                    "moveAzimuth",
                    "closeShutter",
                    "openShutter",
                    "stopMotion",
                    "homeAzimuth",
                    "moveShutterDropoutDoor",
                    "moveShutterMainDoor",
                )
            )

    async def check_initial_az_events(self):
        """Read and check initial azimuthCommandedState and azimuthState.
        """
        az_cmd_state = await self.assert_next_sample(
            topic=self.remote.evt_azimuthCommandedState,
            commandedState=AzimuthCommandedState.UNKNOWN,
        )
        self.assertTrue(math.isnan(az_cmd_state.azimuth))

        await self.assert_next_sample(
            topic=self.remote.evt_azimuthState,
            state=AzimuthState.NOTINMOTION,
            homing=False,
        )

    async def check_initial_shutter_events(self):
        """Read and check the initial state of the shutter events.

        Read and check the initial value of the following events:
        * mainDoorState,
        * dropoutDoorState
        * mainDoorCommandedState
        * dropoutDoorCommanded
        * shutterInPosition
        """
        await self.check_shutter_events(
            main_door_cmd_state=ShutterDoorCommandedState.UNKNOWN,
            dropout_door_cmd_state=ShutterDoorCommandedState.UNKNOWN,
            main_door_state=ShutterDoorState.CLOSED,
            dropout_door_state=ShutterDoorState.CLOSED,
            shutter_in_position=False,
        )

    async def check_shutter_events(
        self,
        main_door_cmd_state=None,
        dropout_door_cmd_state=None,
        main_door_state=None,
        dropout_door_state=None,
        shutter_in_position=None,
    ):
        """Check state of one or more shutter events.

        Parameters
        ----------
        main_door_cmd_state : `ShutterDoorCommandedState`, optional
            Desired state of the ``mainDoorCommandedState`` event.
            If None then this event is not read.
        dropout_door_cmd_state : `ShutterDoorCommandedState`, optional
            Desired state of the ``dropoutDoorCommandedState`` event.
            If None then this event is not read.
        main_door_state : `ShutterDoorState`, optional
            Desired state of the ``mainDoorState`` event.
            If None then this event is not read.
        dropout_door_state : `ShutterDoorState`, optional
            Desired state of the ``dropoutDoorState`` event.
            If None then this event is not read.
        shutter_in_position : `bool`, optional
            Desired state of the ``shutterInPosition`` event.
            If None then this event is not read.
        """
        if main_door_cmd_state is not None:
            data = await self.remote.evt_mainDoorCommandedState.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertEqual(data.commandedState, main_door_cmd_state)

        if dropout_door_cmd_state is not None:
            data = await self.remote.evt_dropoutDoorCommandedState.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertEqual(data.commandedState, dropout_door_cmd_state)

        if main_door_state is not None:
            data = await self.remote.evt_mainDoorState.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertEqual(data.state, main_door_state)

        if dropout_door_state is not None:
            data = await self.remote.evt_dropoutDoorState.next(
                flush=False, timeout=STD_TIMEOUT
            )
            self.assertEqual(data.state, dropout_door_state)

        if shutter_in_position is not None:
            data = await self.remote.evt_shutterInPosition.next(
                flush=False, timeout=STD_TIMEOUT
            )
            if shutter_in_position:
                self.assertTrue(data.inPosition)
            else:
                self.assertFalse(data.inPosition)


if __name__ == "__main__":
    unittest.main()

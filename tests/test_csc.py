# This file is part of ts_atdome.
#
# Developed for Vera C. Rubin Observatory Telescope and Site Systems.
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

import pytest
import yaml

from lsst.ts import salobj
from lsst.ts import utils
from lsst.ts.idl.enums.ATDome import (
    AzimuthCommandedState,
    AzimuthState,
    ShutterDoorCommandedState,
    ShutterDoorState,
)
from lsst.ts import atdome

STD_TIMEOUT = 2  # Standard command timeout (sec)
DOOR_TIMEOUT = 4  # Time limit for shutter door commands (sec)
LONG_TIMEOUT = 20  # Timeout for starting SAL components (sec)
TEST_CONFIG_DIR = pathlib.Path(__file__).parents[1].joinpath("tests", "data", "config")
NODATA_TIMEOUT = 1  # Timeout waiting for data that should not be read (sec)
FLOAT_DELTA = 1e-4  # Delta to use when comparing two float angles


class CscTestCase(salobj.BaseCscTestCase, unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        super().setUp()
        # An azimuth well away from the initial azimuth.
        self.distant_azimuth = utils.angle_wrap_nonnegative(
            atdome.INITIAL_AZIMUTH - 180
        ).deg

    def basic_make_csc(self, initial_state, config_dir, simulation_mode):
        return atdome.ATDomeCsc(
            initial_state=initial_state,
            config_dir=config_dir,
            simulation_mode=simulation_mode,
            mock_port=0,
        )

    async def test_initial_info(self):
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, config_dir=None, simulation_mode=1
        ):
            await self.assert_next_sample(
                self.remote.evt_softwareVersions,
                cscVersion=atdome.__version__,
                subsystemVersions="",
            )
            mock_ctrl = self.csc.mock_ctrl
            await self.check_initial_shutter_events()

            await self.check_initial_az_events()
            await self.assert_next_sample(
                topic=self.remote.evt_allAxesInPosition, inPosition=False
            )

            await self.assert_next_sample(
                topic=self.remote.evt_emergencyStop, active=False
            )

            position = await self.assert_next_sample(
                topic=self.remote.tel_position,
                flush=True,
                dropoutDoorOpeningPercentage=0,
                mainDoorOpeningPercentage=0,
            )
            assert position.azimuthPosition == pytest.approx(atdome.INITIAL_AZIMUTH)

            ctrllr_settings = await self.assert_next_sample(
                topic=self.remote.evt_settingsAppliedDomeController,
                rainSensorEnabled=mock_ctrl.rain_sensor_enabled,
                cloudSensorEnabled=mock_ctrl.cloud_sensor_enabled,
                autoShutdownEnabled=mock_ctrl.auto_shutdown_enabled,
                encoderCountsPer360=mock_ctrl.encoder_counts_per_360,
            )
            assert ctrllr_settings.tolerance == pytest.approx(mock_ctrl.tolerance)
            assert ctrllr_settings.homeAzimuth == pytest.approx(mock_ctrl.home_az)
            assert ctrllr_settings.highSpeedDistance == pytest.approx(
                mock_ctrl.high_speed
            )
            assert ctrllr_settings.watchdogTimer == pytest.approx(
                mock_ctrl.watchdog_reset_time
            )
            assert ctrllr_settings.dropoutTimer == pytest.approx(
                mock_ctrl.dropout_timer
            )
            assert ctrllr_settings.reversalDelay == pytest.approx(
                mock_ctrl.reverse_delay
            )
            assert ctrllr_settings.coast == pytest.approx(mock_ctrl.coast)
            assert ctrllr_settings.azimuthMoveTimeout == pytest.approx(
                mock_ctrl.az_move_timeout
            )
            assert ctrllr_settings.doorMoveTimeout == pytest.approx(
                mock_ctrl.door_move_timeout
            )

            await self.assert_next_sample(
                topic=self.remote.evt_doorEncoderExtremes,
                mainClosed=mock_ctrl.main_door_encoder_closed,
                mainOpened=mock_ctrl.main_door_encoder_opened,
                dropoutClosed=mock_ctrl.dropout_door_encoder_closed,
                dropoutOpened=mock_ctrl.dropout_door_encoder_opened,
            )

            await self.assert_next_sample(
                topic=self.remote.evt_settingsAppliedDomeTcp,
                host=self.csc.config.host,
                port=self.csc.config.port,
                connectionTimeout=self.csc.config.connection_timeout,
                readTimeout=self.csc.config.read_timeout,
            )

            # This az_tolerance value after full status has been received
            standard_az_tolerance = self.csc.az_tolerance
            assert standard_az_tolerance == pytest.approx(
                self.csc.mock_ctrl.tolerance + self.csc.az_tolerance_margin
            )

            # Increase the Azimuth tolerance in the mock controller and check
            # that the tolerance in the CSC is increased by the same amount.
            delta_tolerance = 4.0
            self.csc.mock_ctrl.tolerance += delta_tolerance
            await self.remote.tel_position.next(flush=True)
            await self.remote.tel_position.next(flush=True)
            assert self.csc.az_tolerance == pytest.approx(
                standard_az_tolerance + delta_tolerance
            )

    async def test_default_config_dir(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY, config_dir=None, simulation_mode=1
        ):
            assert self.csc.summary_state == salobj.State.STANDBY
            await self.assert_next_summary_state(salobj.State.STANDBY)

            desired_config_pkg_name = "ts_config_attcs"
            desired_config_env_name = desired_config_pkg_name.upper() + "_DIR"
            desired_config_pkg_dir = os.environ[desired_config_env_name]
            desired_config_dir = pathlib.Path(desired_config_pkg_dir) / "ATDome/v2"
            assert self.csc.get_config_pkg() == desired_config_pkg_name
            assert self.csc.config_dir == desired_config_dir

    async def test_configuration(self):
        async with self.make_csc(
            initial_state=salobj.State.STANDBY,
            config_dir=TEST_CONFIG_DIR,
            simulation_mode=1,
        ):
            assert self.csc.summary_state == salobj.State.STANDBY
            await self.assert_next_summary_state(salobj.State.STANDBY)

            invalid_files = glob.glob(os.path.join(TEST_CONFIG_DIR, "invalid_*.yaml"))
            bad_config_names = [os.path.basename(name) for name in invalid_files]
            bad_config_names.append("no_such_file.yaml")
            for bad_config_name in bad_config_names:
                with self.subTest(bad_config_name=bad_config_name):
                    with salobj.assertRaisesAckError():
                        await self.remote.cmd_start.set_start(
                            configurationOverride=bad_config_name, timeout=STD_TIMEOUT
                        )

            await self.remote.cmd_start.set_start(
                configurationOverride="", timeout=STD_TIMEOUT
            )
            assert self.csc.summary_state == salobj.State.DISABLED
            await self.assert_next_summary_state(salobj.State.DISABLED)
            all_fields_path = os.path.join(TEST_CONFIG_DIR, "_init.yaml")
            with open(all_fields_path, "r") as f:
                all_fields_raw = f.read()
            all_fields_data = yaml.safe_load(all_fields_raw)
            for field, value in all_fields_data.items():
                assert getattr(self.csc.config, field) == value

    async def test_command_failures(self):
        """Test what happens when the mock controller fails a command."""
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, config_dir=None, simulation_mode=1
        ):
            # Check that a CSC command fails if the low-level command fails.
            self.csc.mock_ctrl.fail_command = "SO"  # Open both doors
            with salobj.assertRaisesAckError():
                await self.remote.cmd_openShutter.start(timeout=STD_TIMEOUT)

            # Check that the status loop keeps running even if
            # the low-level status command fails.
            self.csc.mock_ctrl.fail_command = "+"  # Full status
            await self.remote.tel_position.next(flush=True, timeout=STD_TIMEOUT)
            await self.remote.tel_position.next(flush=True, timeout=STD_TIMEOUT)

    async def test_home_azimuth(self):
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, config_dir=None, simulation_mode=1
        ):
            await self.check_initial_az_events()
            await self.assert_next_sample(self.remote.evt_moveCode, code=0)

            # Set home azimuth home switch near the current position,
            # so homing goes quickly.
            curr_az = self.csc.mock_ctrl.az_actuator.position()
            home_azimuth = utils.angle_wrap_nonnegative(curr_az - 5).deg
            self.csc.mock_ctrl.home_az = home_azimuth

            homing_task = asyncio.create_task(
                self.remote.cmd_homeAzimuth.start(timeout=STD_TIMEOUT)
            )

            # Wait for homing to begin and check status.
            az_cmd_state = await self.assert_next_sample(
                topic=self.remote.evt_azimuthCommandedState,
                commandedState=AzimuthCommandedState.HOME,
            )
            assert math.isnan(az_cmd_state.azimuth)
            # Check for MOVINGCCW and homing; this may take up to 2 events.
            await self.assert_next_azimuth_state(
                state=AzimuthState.MOVINGCCW,
                homed=False,
                homeSwitch=False,
                homing=True,
                max_tries=2,
            )
            position = await self.assert_next_sample(
                topic=self.remote.tel_position, flush=True
            )
            assert position.azimuthPosition > home_azimuth

            # Check that moveAzimuth and homeAzimuth are rejected while homing.
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveAzimuth.set_start(
                    azimuth=0, timeout=STD_TIMEOUT
                )
            with salobj.assertRaisesAckError():
                await self.remote.cmd_homeAzimuth.start(timeout=STD_TIMEOUT)

            # Check for MOVINGCW. This will take 2 events if homeSwitch=True
            # is output (common but not guaranteed, due to polling).
            await self.assert_next_azimuth_state(
                state=AzimuthState.MOVINGCW,
                homed=False,
                homeSwitch=False,
                homing=True,
                max_tries=2,
            )
            assert self.csc.mock_ctrl.az_actuator.speed == pytest.approx(
                self.csc.mock_ctrl.home_az_vel
            )

            # Wait for homing motion to finish.
            await asyncio.wait_for(homing_task, timeout=STD_TIMEOUT)
            # Wait for it to halt on the home switch. This will take 2 events
            # if state=MOVINGCW, homeSwitch=True is output (common
            # but not guaranteed, due to polling).
            await self.assert_next_azimuth_state(
                state=AzimuthState.NOTINMOTION,
                homed=True,
                homeSwitch=True,
                homing=False,
                max_tries=2,
            )
            assert self.csc.mock_ctrl.az_actuator.speed == pytest.approx(
                self.csc.mock_ctrl.az_vel
            )
            position = await self.assert_next_sample(
                topic=self.remote.tel_position, flush=True
            )
            assert position.azimuthPosition == pytest.approx(home_azimuth)

    def assert_angle_in_range(self, angle, min_angle, max_angle):
        """Assert that an angle is in the given range.

        All arguments must be in range [0, 360) (and this is checked).
        If min_angle > max_angle then the test is::

            min_angle <= angle < 360 or 0 <= angle <= max_angle

        otherwise the test is the obvious::

            min_angle <= angle <= max_angle

        Parameters
        ----------
        angle : `float`
            Angle to check (deg)
        min_angle : `float`
            Minimum angle, inclusive (deg)
        max_angle : `float`
            Maximum angle, exclusive (deg)

        Raises
        ------
        AssertionError
            If angle, min_angle, or max_angle not in range [0, 360)
            If angle not in the specified range.
        """
        for argname in ("angle", "min_angle", "max_angle"):
            argvalue = locals()[argname]
            if not 0 <= argvalue < 360:
                raise AssertionError(
                    f"Argument {argname} = {argvalue} not in range [0, 360)"
                )
        if min_angle > max_angle:
            if not (min_angle <= angle < 360 or 0 <= angle <= max_angle):
                raise AssertionError(
                    f"angle {angle} not in range [{min_angle}, 360) or [0, {max_angle}]"
                )
        else:
            if not (min_angle <= angle <= max_angle):
                raise AssertionError(
                    f"angle {angle} not in range [{min_angle}, {max_angle}]"
                )

    async def assert_next_azimuth_state(
        self,
        state=None,
        homed=None,
        homeSwitch=None,
        homing=None,
        max_tries=1,
        timeout=STD_TIMEOUT,
    ):
        """Assert that the next azimuth state is as requested.

        Parameters
        ----------
        state : `AzimuthState` or `None`
            Desired state; if `None` then do not check it.
        homing : `bool` or `None`
            Desired homing value; if `None` then do not check it.
        homed : `bool` or `None`
            Desired homed value; if `None` then do not check it.
        homeSwitch : `bool` or None`
            Desired homeSwitch value; if `None` then do not check it.
        max_tries : `int`
            Maximum number of samples to check.
        timeout : `float`
            Maximum time per sample (sec).
        """
        assert max_tries > 0
        for n in range(max_tries):
            data = await self.remote.evt_azimuthState.next(flush=False, timeout=timeout)
            failed_conditions = []
            if state is not None and data.state != state:
                failed_conditions.append(f"state={data.state} != {state!r}")
            if homed is not None and data.homed != homed:
                failed_conditions.append(f"homed={data.homed} != {homed}")
            if homeSwitch is not None and data.homeSwitch != homeSwitch:
                failed_conditions.append(
                    f"homeSwitch={data.homeSwitch} != {homeSwitch}"
                )
            if homing is not None and data.homing != homing:
                failed_conditions.append(f"homing={data.homing} != {homing}")
            if not failed_conditions:
                return
            print(
                f"assert_next_azimuth_state: iter={n+1} of {max_tries}:",
                ", ".join(failed_conditions),
            )

        self.fail(", ".join(failed_conditions))

    async def set_homed(self):
        """Set the mock controller state to homed

        and wait for the associated change of state
        (which only works if it was not already homed).

        This is faster than homing azimuth.
        """
        self.csc.mock_ctrl.homed = True
        await self.assert_next_azimuth_state(
            state=AzimuthState.NOTINMOTION,
            homed=True,
            homing=False,
        )

    async def test_move_azimuth(self):
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, config_dir=None, simulation_mode=1
        ):
            await self.check_initial_az_events()
            await self.assert_next_sample(self.remote.evt_moveCode, code=0)

            # Motion should be rejected because not homed
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveAzimuth.set_start(
                    azimuth=0, timeout=STD_TIMEOUT
                )

            await self.set_homed()

            # Put the azimuth home switch well away from the current position
            # to avoid extra azimuthState events.
            self.csc.mock_ctrl.home_az = self.distant_azimuth

            # Test an initial condition necessary for this test.
            # If this fails then the first entry in
            # `for desired_azimuth, ...` may need to be updated.
            assert atdome.INITIAL_AZIMUTH == pytest.approx(285)

            # Try several angles, including some not in the range [0, 360);
            # CW direction is towards larger azimuth.
            isFirst = True
            for desired_azimuth, desired_moving_state, min_angle, max_angle in (
                (325.1, AzimuthState.MOVINGCW, 285, 325.1),
                (-1.2, AzimuthState.MOVINGCW, 325.1, -1.2 + 360),
                (390.3, AzimuthState.MOVINGCW, -1.2 + 360, 390.3 - 360),
                (-1.2, AzimuthState.MOVINGCCW, -1.2 + 360, 390.3 - 360),
            ):
                wrapped_desired_azimuth = utils.angle_wrap_nonnegative(
                    desired_azimuth
                ).deg
                await self.remote.cmd_moveAzimuth.set_start(
                    azimuth=desired_azimuth, timeout=STD_TIMEOUT
                )
                if not isFirst:
                    await self.assert_next_sample(
                        topic=self.remote.evt_azimuthInPosition,
                        inPosition=False,
                    )

                # Wait for the move to begin and check status.
                az_cmd_state = await self.assert_next_sample(
                    topic=self.remote.evt_azimuthCommandedState,
                    commandedState=AzimuthCommandedState.GOTOPOSITION,
                )
                assert az_cmd_state.azimuth == pytest.approx(
                    wrapped_desired_azimuth, abs=FLOAT_DELTA
                )
                await self.assert_next_azimuth_state(
                    state=desired_moving_state,
                    homed=True,
                    homeSwitch=False,
                    homing=False,
                )
                if desired_moving_state == AzimuthState.MOVINGCW:
                    desired_code = atdome.MoveCode.AZIMUTH_POSITIVE
                elif desired_moving_state == AzimuthState.MOVINGCCW:
                    desired_code = atdome.MoveCode.AZIMUTH_NEGATIVE
                else:
                    self.fail(f"Unsupported moving state {desired_moving_state}")
                await self.assert_next_sample(
                    self.remote.evt_moveCode, code=desired_code
                )

                # While the move occurs test that the position is in range
                # (i.e. the dome didn't go the wrong way around).
                while True:
                    azimuth_state = self.remote.evt_azimuthState.get()
                    if azimuth_state.state != desired_moving_state:
                        break
                    position = self.remote.tel_position.get()
                    self.assert_angle_in_range(
                        position.azimuthPosition, min_angle, max_angle
                    )
                    await asyncio.sleep(0.1)

                # Make sure the final state is as expected.
                await self.assert_next_azimuth_state(
                    state=AzimuthState.NOTINMOTION,
                    homed=True,
                    homeSwitch=False,
                    homing=False,
                )
                position = self.remote.tel_position.get()
                assert position.azimuthPosition == pytest.approx(
                    wrapped_desired_azimuth, abs=FLOAT_DELTA
                )

                await self.assert_next_sample(
                    topic=self.remote.evt_azimuthInPosition,
                    inPosition=True,
                )
                await self.assert_next_sample(self.remote.evt_moveCode, code=0)
                isFirst = False

            # Move the shutter to its current location (for speed)
            # and check evt_allAxesInPosition.
            await self.assert_next_sample(
                topic=self.remote.evt_shutterInPosition, inPosition=False
            )
            await self.assert_next_sample(
                topic=self.remote.evt_allAxesInPosition, inPosition=False
            )
            await self.remote.cmd_closeShutter.set_start(timeout=STD_TIMEOUT)
            await self.assert_next_sample(
                topic=self.remote.evt_shutterInPosition, inPosition=True
            )
            await self.assert_next_sample(
                topic=self.remote.evt_allAxesInPosition, inPosition=True
            )

    async def test_move_shutter(self):
        """Test openShutter and closeShutter commands."""
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, config_dir=None, simulation_mode=1
        ):
            await self.check_initial_shutter_events()
            await self.assert_next_sample(self.remote.evt_moveCode, code=0)

            # Open both doors.
            await self.remote.cmd_openShutter.start(timeout=DOOR_TIMEOUT)

            # Check opening events; note that shutterInPosition was
            # initially False and is not output again.
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.OPENED,
                main_door_cmd_state=ShutterDoorCommandedState.OPENED,
                dropout_door_state=ShutterDoorState.OPENING,
                main_door_state=ShutterDoorState.OPENING,
            )
            await self.assert_next_sample(
                self.remote.evt_moveCode,
                code=atdome.MoveCode.MAIN_DOOR_OPENING
                + atdome.MoveCode.DROPOUT_DOOR_OPENING,
            )

            # Check fully open events.
            await self.check_shutter_events(
                dropout_door_state=ShutterDoorState.OPENED,
                main_door_state=ShutterDoorState.OPENED,
                shutter_in_position=True,
            )
            # I'm not sure if one door will finish moving first,
            # or if both will be reported as done at the same time.
            self.remote.evt_moveCode.flush()
            data = self.remote.evt_moveCode.get()
            assert data.code == 0

            # Close both doors.
            await self.remote.cmd_closeShutter.start(timeout=DOOR_TIMEOUT)

            # Check closing events.
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                main_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                dropout_door_state=ShutterDoorState.CLOSING,
                main_door_state=ShutterDoorState.CLOSING,
                shutter_in_position=False,
            )
            await self.assert_next_sample(
                self.remote.evt_moveCode,
                code=atdome.MoveCode.MAIN_DOOR_CLOSING
                + atdome.MoveCode.DROPOUT_DOOR_CLOSING,
            )

            # Check fully closed events.
            await self.check_shutter_events(
                dropout_door_state=ShutterDoorState.CLOSED,
                main_door_state=ShutterDoorState.CLOSED,
                shutter_in_position=True,
            )
            # Again I'm not sure which will close first,
            # so don't check intermediate states.
            self.remote.evt_moveCode.flush()
            data = self.remote.evt_moveCode.get()
            assert data.code == 0

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
            await self.assert_next_sample(self.remote.evt_moveCode, code=0)

            # Check that we cannot open or close the dropout door
            # because the main door is not fully open.
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveShutterDropoutDoor.set_start(
                    open=True, timeout=STD_TIMEOUT
                )
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveShutterDropoutDoor.set_start(
                    open=False, timeout=STD_TIMEOUT
                )

            # Start opening the main door.
            main_open_task = asyncio.ensure_future(
                self.remote.cmd_moveShutterMainDoor.set_start(
                    open=True, timeout=DOOR_TIMEOUT
                )
            )

            # Check main door opening status;
            # the dropout door is not moving so no dropout events output,
            # and shutter_in_position remains False so is not output.
            await self.check_shutter_events(
                main_door_cmd_state=ShutterDoorCommandedState.OPENED,
                main_door_state=ShutterDoorState.OPENING,
            )
            await self.assert_next_sample(
                self.remote.evt_moveCode,
                code=atdome.MoveCode.MAIN_DOOR_OPENING,
            )

            # Check that we cannot open or close the dropout door
            # because the main door is not fully open.
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveShutterDropoutDoor.set_start(
                    open=True, timeout=STD_TIMEOUT
                )
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveShutterDropoutDoor.set_start(
                    open=False, timeout=STD_TIMEOUT
                )

            # Wait for the move to end.
            await main_open_task

            # Check main fully open status.
            # The dropout door has no commanded position,
            # so shutter_in_position remains False.
            await self.check_shutter_events(main_door_state=ShutterDoorState.OPENED)
            await self.assert_next_sample(self.remote.evt_moveCode, code=0)

            # Open the main door again; this should be quick.
            await self.remote.cmd_moveShutterMainDoor.set_start(
                open=True, timeout=STD_TIMEOUT
            )

            # The only expected event is mainDoorCmdState.
            await self.check_shutter_events(
                main_door_cmd_state=ShutterDoorCommandedState.OPENED
            )

            # Start opening the dropout door.
            dropout_open_task = asyncio.ensure_future(
                self.remote.cmd_moveShutterDropoutDoor.set_start(
                    open=True, timeout=DOOR_TIMEOUT
                )
            )

            # Check opening dropout door status.
            # The main door is not moving so we should not see main events.
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.OPENED,
                dropout_door_state=ShutterDoorState.OPENING,
            )
            await self.assert_next_sample(
                self.remote.evt_moveCode,
                code=atdome.MoveCode.DROPOUT_DOOR_OPENING,
            )

            # Make sure we can't close the main door
            # while the dropout door is moving.
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveShutterMainDoor.set_start(
                    open=False, timeout=STD_TIMEOUT
                )

            # Wait for the dropout door move to finish.
            await dropout_open_task

            # Check dropout door fully opened;
            # finally shutter_in_position should be True.
            await self.check_shutter_events(
                dropout_door_state=ShutterDoorState.OPENED, shutter_in_position=True
            )
            await self.assert_next_sample(self.remote.evt_moveCode, code=0)

            # Open the dropout door again; this should be quick.
            await self.remote.cmd_moveShutterDropoutDoor.set_start(
                open=True, timeout=STD_TIMEOUT
            )

            # The only expected event is dropoutDoorCmdState.
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.OPENED
            )

            # Start closing the main door.
            main_close_task = asyncio.ensure_future(
                self.remote.cmd_moveShutterMainDoor.set_start(
                    open=False, timeout=DOOR_TIMEOUT
                )
            )
            await self.assert_next_sample(
                self.remote.evt_moveCode,
                code=atdome.MoveCode.MAIN_DOOR_CLOSING,
            )

            # Check main door closing status;
            # the dropout door is not moving so no dropout events output.
            await self.check_shutter_events(
                main_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                main_door_state=ShutterDoorState.CLOSING,
                shutter_in_position=False,
            )

            # Check that we cannot open or close the dropout door
            # because the main door is not fully open.
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveShutterDropoutDoor.set_start(
                    open=True, timeout=STD_TIMEOUT
                )
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveShutterDropoutDoor.set_start(
                    open=False, timeout=STD_TIMEOUT
                )

            # Wait for the main door to finish closing.
            await main_close_task

            # Check main fully closed status.
            await self.check_shutter_events(
                main_door_state=ShutterDoorState.CLOSED, shutter_in_position=True
            )
            await self.assert_next_sample(self.remote.evt_moveCode, code=0)

            # Close the main door again; this should be quick.
            await self.remote.cmd_moveShutterMainDoor.set_start(
                open=False, timeout=STD_TIMEOUT
            )

            # The only expected event is mainDoorCmdState.
            await self.check_shutter_events(
                main_door_cmd_state=ShutterDoorCommandedState.CLOSED
            )

            # Open the main door.
            await self.remote.cmd_moveShutterMainDoor.set_start(
                open=True, timeout=DOOR_TIMEOUT
            )

            # Check main door opening status;
            # the dropout door is not moving so no dropout events output.
            await self.check_shutter_events(
                main_door_cmd_state=ShutterDoorCommandedState.OPENED,
                main_door_state=ShutterDoorState.OPENING,
                shutter_in_position=False,
            )
            await self.assert_next_sample(
                self.remote.evt_moveCode,
                code=atdome.MoveCode.MAIN_DOOR_OPENING,
            )

            # Check main door fully opening status;
            # the dropout door is not moving so no dropout events output.
            await self.check_shutter_events(
                main_door_state=ShutterDoorState.OPENED, shutter_in_position=True
            )
            await self.assert_next_sample(self.remote.evt_moveCode, code=0)

            # Start closing the dropout door.
            dropout_close_task = asyncio.ensure_future(
                self.remote.cmd_moveShutterDropoutDoor.set_start(
                    open=False, timeout=DOOR_TIMEOUT
                )
            )

            # Check dropout door closing status;
            # the main door is not moving so no main events output.
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                dropout_door_state=ShutterDoorState.CLOSING,
                shutter_in_position=False,
            )
            await self.assert_next_sample(
                self.remote.evt_moveCode,
                code=atdome.MoveCode.DROPOUT_DOOR_CLOSING,
            )

            # Make sure we can't close the main door
            # while the dropout door is moving.
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveShutterMainDoor.set_start(
                    open=False, timeout=STD_TIMEOUT
                )

            # Wait for the dropout door to finish closing.
            await dropout_close_task

            # Check dropout fully closed status.
            await self.check_shutter_events(
                dropout_door_state=ShutterDoorState.CLOSED, shutter_in_position=True
            )
            await self.assert_next_sample(self.remote.evt_moveCode, code=0)

            # Close the dropout door again; this should be quick.
            await self.remote.cmd_moveShutterDropoutDoor.set_start(
                open=False, timeout=STD_TIMEOUT
            )

            # The only expected event is dropoutDoorCmdState.
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.CLOSED
            )

            # Start closing the main door.
            main_close_task = asyncio.ensure_future(
                self.remote.cmd_moveShutterMainDoor.set_start(
                    open=False, timeout=DOOR_TIMEOUT
                )
            )

            # Check main door closing status;
            # the dropout door is not moving so no dropout events output.
            await self.check_shutter_events(
                main_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                main_door_state=ShutterDoorState.CLOSING,
                shutter_in_position=False,
            )
            await self.assert_next_sample(
                self.remote.evt_moveCode,
                code=atdome.MoveCode.MAIN_DOOR_CLOSING,
            )

            # Check that we cannot open or close the dropout door
            # because the main door is not fully open.
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveShutterDropoutDoor.set_start(
                    open=True, timeout=STD_TIMEOUT
                )
            with salobj.assertRaisesAckError():
                await self.remote.cmd_moveShutterDropoutDoor.set_start(
                    open=False, timeout=STD_TIMEOUT
                )

            # Wait for the main door to finish closing.
            await main_close_task

            # Check main fully closed status.
            await self.check_shutter_events(
                main_door_state=ShutterDoorState.CLOSED, shutter_in_position=True
            )
            await self.assert_next_sample(self.remote.evt_moveCode, code=0)

    async def test_close_shutter_supersedes_open_shutter(self):
        """Test that closing the shutter supersedes opening the shutter."""
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, config_dir=None, simulation_mode=1
        ):
            await self.check_initial_shutter_events()

            # Start opening the both doors.
            open_task = asyncio.ensure_future(
                self.remote.cmd_openShutter.start(timeout=DOOR_TIMEOUT)
            )

            # Check opening status;
            # shutter_in_position is still False, so not output.
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.OPENED,
                main_door_cmd_state=ShutterDoorCommandedState.OPENED,
                dropout_door_state=ShutterDoorState.OPENING,
                main_door_state=ShutterDoorState.OPENING,
            )

            # Start closing the shutter.
            close_task = asyncio.ensure_future(
                self.remote.cmd_closeShutter.start(timeout=DOOR_TIMEOUT)
            )

            # Check that this supersedes opening the shutter.
            with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_ABORTED):
                await open_task

            # Check status of closing both doors.
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                main_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                dropout_door_state=ShutterDoorState.CLOSING,
                main_door_state=ShutterDoorState.CLOSING,
            )

            await close_task

            # Check status of both doors fully closed.
            await self.check_shutter_events(
                dropout_door_state=ShutterDoorState.CLOSED,
                main_door_state=ShutterDoorState.CLOSED,
                shutter_in_position=True,
            )

    async def test_open_shutter_supersedes_move_dropout(self):
        """Test that opening the shutter supersedes moving the dropout door."""
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, config_dir=None, simulation_mode=1
        ):
            await self.check_initial_shutter_events()

            # Open the doors, since the main door has to be fully open
            # to move the dropout door.
            await self.remote.cmd_openShutter.start(timeout=DOOR_TIMEOUT)

            # Check opening status.
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.OPENED,
                main_door_cmd_state=ShutterDoorCommandedState.OPENED,
                dropout_door_state=ShutterDoorState.OPENING,
                main_door_state=ShutterDoorState.OPENING,
            )

            # Check final open status.
            await self.check_shutter_events(
                dropout_door_state=ShutterDoorState.OPENED,
                main_door_state=ShutterDoorState.OPENED,
                shutter_in_position=True,
            )

            # Start closing the dropout door.
            dropout_close_task = asyncio.ensure_future(
                self.remote.cmd_moveShutterDropoutDoor.set_start(
                    open=False, timeout=DOOR_TIMEOUT
                )
            )

            # Check closing dropout status;
            # main door isn't moving so main events are not output.
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                dropout_door_state=ShutterDoorState.CLOSING,
                shutter_in_position=False,
            )

            # Start opening the shutter.
            open_task = asyncio.ensure_future(
                self.remote.cmd_openShutter.start(timeout=DOOR_TIMEOUT)
            )

            # Check that opening the shutter supersedes closing dropout door.
            with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_ABORTED):
                await dropout_close_task

            # Check status of closing both doors;
            # main door was already open but its cmd state is output.
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.OPENED,
                main_door_cmd_state=ShutterDoorCommandedState.OPENED,
                dropout_door_state=ShutterDoorState.OPENING,
            )

            await open_task

            # Check status of both doors fully closed;
            # the main door never moved.
            await self.check_shutter_events(
                dropout_door_state=ShutterDoorState.OPENED, shutter_in_position=True
            )

    async def test_close_shutter_supersedes_move_main(self):
        """Test that closing the shutter supersedes moving the main door."""
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, config_dir=None, simulation_mode=1
        ):
            await self.check_initial_shutter_events()

            # Start opening the main door.
            main_open_task = asyncio.ensure_future(
                self.remote.cmd_moveShutterMainDoor.set_start(
                    open=True, timeout=DOOR_TIMEOUT
                )
            )

            # Check opening main status.
            # The dropout door isn't moving so dropout events are not output,
            # and shutter_in_position is still False, so not output.
            await self.check_shutter_events(
                main_door_cmd_state=ShutterDoorCommandedState.OPENED,
                main_door_state=ShutterDoorState.OPENING,
            )

            # Start closing the shutter;
            # this supersedes opening the main door.
            close_task = asyncio.ensure_future(
                self.remote.cmd_closeShutter.start(timeout=DOOR_TIMEOUT)
            )

            with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_ABORTED):
                await main_open_task

            # Check status of closing both doors;
            # the dropout door was already closed but its cmd state is output.
            await self.check_shutter_events(
                main_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                dropout_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                main_door_state=ShutterDoorState.CLOSING,
            )

            await close_task

            # Check final closed status; the dropout door never moved.
            await self.check_shutter_events(
                main_door_state=ShutterDoorState.CLOSED, shutter_in_position=True
            )

    async def test_open_shutter_supersedes_close_shutter(self):
        """Test that opening the shutter supersedes closing the shutter."""
        async with self.make_csc(
            initial_state=salobj.State.ENABLED, config_dir=None, simulation_mode=1
        ):
            await self.check_initial_shutter_events()

            # Open both doors.
            await self.remote.cmd_openShutter.start(timeout=DOOR_TIMEOUT)

            # Check move begins events.
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.OPENED,
                main_door_cmd_state=ShutterDoorCommandedState.OPENED,
                dropout_door_state=ShutterDoorState.OPENING,
                main_door_state=ShutterDoorState.OPENING,
            )

            # Check move ends events.
            await self.check_shutter_events(
                dropout_door_state=ShutterDoorState.OPENED,
                main_door_state=ShutterDoorState.OPENED,
                shutter_in_position=True,
            )

            # Start closing both doors.
            close_task = asyncio.ensure_future(
                self.remote.cmd_closeShutter.start(timeout=DOOR_TIMEOUT)
            )

            # Check move begins events.
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                main_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                dropout_door_state=ShutterDoorState.CLOSING,
                main_door_state=ShutterDoorState.CLOSING,
                shutter_in_position=False,
            )

            # Start opening both doors (again).
            open_task = asyncio.ensure_future(
                self.remote.cmd_openShutter.start(timeout=DOOR_TIMEOUT)
            )

            # Check that the open command superseded the close command.
            with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_ABORTED):
                await close_task

            # Check opening status;
            # shutter_in_position is still False, so not output.
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.OPENED,
                main_door_cmd_state=ShutterDoorCommandedState.OPENED,
                dropout_door_state=ShutterDoorState.OPENING,
                main_door_state=ShutterDoorState.OPENING,
            )

            await open_task

            # Check final open status.
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

            await self.set_homed()

            # Move azimuth and start opening the shutter.
            await self.remote.cmd_moveAzimuth.set_start(
                azimuth=atdome.INITIAL_AZIMUTH - 10, timeout=STD_TIMEOUT
            )
            shutter_open_task = asyncio.ensure_future(
                self.remote.cmd_openShutter.start(timeout=STD_TIMEOUT)
            )

            # Wait for the moves to start.
            await self.assert_next_azimuth_state(
                state=AzimuthState.MOVINGCCW,
                homed=True,
                homeSwitch=False,
                homing=False,
            )

            # Check that the shutter was told to open;
            # shutter_in_position remains false so is not output.
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.OPENED,
                main_door_cmd_state=ShutterDoorCommandedState.OPENED,
                dropout_door_state=ShutterDoorState.OPENING,
                main_door_state=ShutterDoorState.OPENING,
            )

            # Disable the CSC.
            # This should not produce new "inPosition" events, because
            # motion is stopped while the axes are still not in position.
            await self.remote.cmd_disable.start(timeout=STD_TIMEOUT)

            # Make sure the shutter command was cancelled.
            with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_ABORTED):
                await shutter_open_task

            await self.assert_next_azimuth_state(
                state=AzimuthState.NOTINMOTION,
                homeSwitch=False,
                homing=False,
            )
            with pytest.raises(asyncio.TimeoutError):
                await self.remote.evt_azimuthInPosition.next(
                    flush=False, timeout=NODATA_TIMEOUT
                )

            # Check that the shutter was told to close.
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                main_door_cmd_state=ShutterDoorCommandedState.CLOSED,
                dropout_door_state=ShutterDoorState.CLOSING,
                main_door_state=ShutterDoorState.CLOSING,
            )

            # Check that the shutter closed.
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

            await self.set_homed()

            # Put the azimuth home switch well away from the current position
            # to avoid extra azimuthState events.
            self.csc.mock_ctrl.home_az = self.distant_azimuth

            # Move azimuth and start opening the shutter.
            await self.remote.cmd_moveAzimuth.set_start(
                azimuth=atdome.INITIAL_AZIMUTH - 10, timeout=STD_TIMEOUT
            )
            shutter_open_task = asyncio.ensure_future(
                self.remote.cmd_openShutter.start(timeout=STD_TIMEOUT)
            )

            # Wait for the moves to start.
            await self.assert_next_azimuth_state(
                state=AzimuthState.MOVINGCCW,
                homed=True,
                homeSwitch=False,
                homing=False,
            )

            # Check that the shutter was told to open;
            # shutter_in_position remains false so is not output.
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.OPENED,
                main_door_cmd_state=ShutterDoorCommandedState.OPENED,
                dropout_door_state=ShutterDoorState.OPENING,
                main_door_state=ShutterDoorState.OPENING,
            )

            # Give the shutters a chance to move. This works around
            # a race condition that Tiago reported.
            await asyncio.sleep(0.1)

            # Stop all motion.
            # This should not produce new "inPosition" events, because
            # motion is stopped while the axes are still not in position.
            await self.remote.cmd_stopMotion.start(timeout=STD_TIMEOUT)

            # Make sure the shutter command was cancelled.
            with salobj.assertRaisesAckError(ack=salobj.SalRetCode.CMD_ABORTED):
                await shutter_open_task

            await self.assert_next_azimuth_state(
                state=AzimuthState.NOTINMOTION,
                homed=True,
                homeSwitch=False,
                homing=False,
            )
            with pytest.raises(asyncio.TimeoutError):
                await self.remote.evt_azimuthInPosition.next(
                    flush=False, timeout=NODATA_TIMEOUT
                )

            # Check that the shutter was told to stop;
            # shutter_in_position remains false so is not output.
            await self.check_shutter_events(
                dropout_door_cmd_state=ShutterDoorCommandedState.STOP,
                main_door_cmd_state=ShutterDoorCommandedState.STOP,
                dropout_door_state=ShutterDoorState.PARTIALLYOPENED,
                main_door_state=ShutterDoorState.PARTIALLYOPENED,
            )
            with pytest.raises(asyncio.TimeoutError):
                await self.remote.evt_shutterInPosition.next(
                    flush=False, timeout=NODATA_TIMEOUT
                )

    async def test_bin_script(self):
        await self.check_bin_script(name="ATDome", index=None, exe_name="run_atdome")

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
        """Read and check initial azimuthCommandedState and azimuthState."""
        az_cmd_state = await self.assert_next_sample(
            topic=self.remote.evt_azimuthCommandedState,
            commandedState=AzimuthCommandedState.UNKNOWN,
        )
        assert math.isnan(az_cmd_state.azimuth)

        await self.assert_next_azimuth_state(
            state=AzimuthState.NOTINMOTION,
            homed=False,
            homeSwitch=False,
            homing=False,
        )

        await self.assert_next_sample(
            topic=self.remote.evt_azimuthInPosition,
            inPosition=False,
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
            await self.assert_next_sample(
                topic=self.remote.evt_mainDoorCommandedState,
                commandedState=main_door_cmd_state,
            )

        if dropout_door_cmd_state is not None:
            await self.assert_next_sample(
                topic=self.remote.evt_dropoutDoorCommandedState,
                commandedState=dropout_door_cmd_state,
            )

        if main_door_state is not None:
            await self.assert_next_sample(
                topic=self.remote.evt_mainDoorState, state=main_door_state
            )

        if dropout_door_state is not None:
            await self.assert_next_sample(
                topic=self.remote.evt_dropoutDoorState, state=dropout_door_state
            )

        if shutter_in_position is not None:
            await self.assert_next_sample(
                topic=self.remote.evt_shutterInPosition, inPosition=shutter_in_position
            )


if __name__ == "__main__":
    unittest.main()

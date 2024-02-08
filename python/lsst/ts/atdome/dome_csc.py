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

__all__ = ["ATDomeCsc", "run_atdome"]

import asyncio
import enum
import math

from lsst.ts import salobj, tcpip, utils
from lsst.ts.idl.enums.ATDome import (
    AzimuthCommandedState,
    AzimuthState,
    ShutterDoorCommandedState,
    ShutterDoorState,
)

from . import __version__
from .config_schema import CONFIG_SCHEMA
from .enums import ErrorCode, MoveCode
from .mock_controller import MockDomeController
from .status import Status

# Max time (sec) to home the azimuth.
HOME_AZIMUTH_TIMEOUT = 150

# Max time (sec) to wait for the mock controller to start.
MOCK_CTRL_START_TIMEOUT = 2


class Axis(enum.Flag):
    AZIMUTH = enum.auto()
    DROPOUT_DOOR = enum.auto()
    MAIN_DOOR = enum.auto()


class ATDomeCsc(salobj.ConfigurableCsc):
    """AuxTel dome CSC

    Parameters
    ----------
    initial_state : `salobj.State` or `int` (optional)
        The initial state of the CSC. This is provided for unit testing,
        as real CSCs should start up in `lsst.ts.salobj.StateSTANDBY`,
        the default.
    simulation_mode : `int` (optional)
        Simulation mode.

    Raises
    ------
    salobj.ExpectedError
        If initial_state or simulation_mode is invalid.

    Notes
    -----
    **Simulation Modes**

    Supported simulation modes (TODO DM-19530 update these values):

    * 0: regular operation
    * 1: simulation mode: start a mock TCP/IP ATDome controller and talk to it

    **Error Codes**

    * 1: could not connect to TCP/IP ATDome controller
    * 2: read from TCP/IP ATDome controller timed out
    * 3: could not start the mock controller
    """

    valid_simulation_modes = (0, 1)
    version = __version__

    def __init__(
        self,
        config_dir=None,
        initial_state=salobj.State.STANDBY,
        simulation_mode=0,
    ):
        self.move_code = 0
        self.mock_ctrl = None  # mock controller, or None of not constructed
        self.status_interval = 0.2  # delay between short status commands (sec)

        # Amount to add to "tolerance" reported by the low-level controller
        # to set az_tolarance (deg).
        self.az_tolerance_margin = 0.5

        # Tolerance for "in position" (deg).
        # Set the initial value here, then update from the
        # "Tolerance" reported in long status.
        self.az_tolerance = 1.5
        self.max_dome_move_below_threshold = 3
        self.az_move_tol = 0.1

        # Task for sleeping in the status loop; cancel this to trigger
        # an immediate status update. Warning: do not cancel status_task
        # because that may be waiting for TCP/IP communication.
        self.status_sleep_task = utils.make_done_future()
        # Task for the status loop. To trigger new status cancel
        # status_sleep_task, not status_task.
        self.status_task = utils.make_done_future()
        # Task that waits while connecting to the TCP/IP controller.
        self.connect_task = utils.make_done_future()
        # Task that waits while shutter doors move
        self.shutter_task = utils.make_done_future()
        # Task that waits for dome to move to position and retries
        # operation if it fails.
        self.move_azimuth_task = utils.make_done_future()
        # The conditions that self.shutter_task is waiting for.
        # Must be one of: ShutterDoorState.OPENED,
        # ShutterDoorState.CLOSED or None (for don't care)
        self.desired_main_shutter_state = None
        self.desired_dropout_shutter_state = None
        self.cmd_lock = asyncio.Lock()
        self.config = None
        self.status_event = asyncio.Event()
        super().__init__(
            name="ATDome",
            index=0,
            config_schema=CONFIG_SCHEMA,
            config_dir=config_dir,
            initial_state=initial_state,
            simulation_mode=simulation_mode,
        )

        # TCP/IP client for the low-level controller.
        # Initialize to a client that is already closed, to avoid
        # having to test for "is None".
        self.client = tcpip.Client(host="", port=0, log=self.log)

        # Homing can be slow; rather than queue up these commands
        # (which is unlikely to be what the user wants) allow multiple
        # command to run, so the CSC can reject the overlapping commands.
        self.cmd_homeAzimuth.allow_multiple_callbacks = True

    async def do_moveAzimuth(self, data):
        """Implement the ``moveAzimuth`` command."""
        self.assert_enabled()
        if self.evt_azimuthState.data.homing:
            raise salobj.ExpectedError("Cannot move azimuth while homing")
        if not self.evt_azimuthState.data.homed:
            raise salobj.ExpectedError("The azimuth axis is not homed")
        await self._stop_move_az_task()
        azimuth = utils.angle_wrap_nonnegative(data.azimuth).deg
        await self.run_command(f"{azimuth:0.3f} MV")
        await self.evt_azimuthCommandedState.set_write(
            commandedState=AzimuthCommandedState.GOTOPOSITION,
            azimuth=azimuth,
            force_output=True,
        )
        self.status_sleep_task.cancel()
        self.move_azimuth_task = asyncio.create_task(
            self._handle_azimuth_move(position=azimuth)
        )

    async def do_closeShutter(self, data):
        """Implement the ``closeShutter`` command."""
        self.assert_enabled()
        self.shutter_task.cancel()
        await self._stop_move_az_task()
        await self.run_command("SC")
        await self.evt_dropoutDoorCommandedState.set_write(
            commandedState=ShutterDoorCommandedState.CLOSED, force_output=True
        )
        await self.evt_mainDoorCommandedState.set_write(
            commandedState=ShutterDoorCommandedState.CLOSED, force_output=True
        )
        await self.wait_for_shutter(
            dropout_state=ShutterDoorState.CLOSED, main_state=ShutterDoorState.CLOSED
        )

    async def do_openShutter(self, data):
        """Implement the ``openShutter`` command."""
        self.assert_enabled()
        await self._stop_move_az_task()
        await self.run_command("SO")
        await self.evt_dropoutDoorCommandedState.set_write(
            commandedState=ShutterDoorCommandedState.OPENED, force_output=True
        )
        await self.evt_mainDoorCommandedState.set_write(
            commandedState=ShutterDoorCommandedState.OPENED, force_output=True
        )
        await self.wait_for_shutter(
            dropout_state=ShutterDoorState.OPENED, main_state=ShutterDoorState.OPENED
        )

    async def do_stopMotion(self, data):
        """Implement the ``stopMotion`` command."""
        self.assert_enabled()
        await self._stop_move_az_task()
        await self.evt_azimuthCommandedState.set_write(
            commandedState=AzimuthCommandedState.STOP, force_output=True
        )
        await self.evt_dropoutDoorCommandedState.set_write(
            commandedState=ShutterDoorCommandedState.STOP, force_output=True
        )
        await self.evt_mainDoorCommandedState.set_write(
            commandedState=ShutterDoorCommandedState.STOP, force_output=True
        )
        await self.run_command("ST")
        self.shutter_task.cancel()
        self.status_sleep_task.cancel()

    async def do_homeAzimuth(self, data):
        """Implement the ``homeAzimuth`` command."""
        self.assert_enabled()
        await self._stop_move_az_task()
        put_final_azimuth_commanded_state = False
        if self.evt_azimuthState.data.homing:
            raise salobj.ExpectedError("Already homing")
        if self.evt_azimuthState.data.homeSwitch and self.evt_azimuthState.data.homed:
            self.log.warning(
                "Not homing, because already homed and on the home switch, "
                "and the low-level controller cannot handle that."
                "If you want to home again, first move the dome off the home switch "
                "(via a command or the manual push buttons in the dome)."
            )
            return
        try:
            await self.wait_n_status(n=2)
            if bool(self.evt_moveCode.data.code & MoveCode.AZIMUTH_HOMING):
                raise salobj.ExpectedError("Already homing")
            if not self.evt_azimuthState.data.state == AzimuthState.NOTINMOTION:
                raise salobj.ExpectedError("Azimuth is moving")

            # Homing is allowed; do it!
            await self.cmd_homeAzimuth.ack_in_progress(
                data=data, timeout=HOME_AZIMUTH_TIMEOUT
            )
            await self.evt_azimuthCommandedState.set_write(
                commandedState=AzimuthCommandedState.HOME,
                azimuth=math.nan,
                force_output=True,
            )
            put_final_azimuth_commanded_state = True
            await self.run_command("HM")
            self.status_sleep_task.cancel()
            # Check status until homed.
            # Skip one status to start with, to give the dome
            # time to report that it is homing.
            n_to_await = 2
            while True:
                await self.wait_n_status(n=n_to_await)
                n_to_await = 1

                if (
                    self.evt_azimuthState.data.homed
                    and not self.evt_azimuthState.data.homing
                ):
                    break
        except Exception as e:
            self.log.exception(f"homing failed: {e!r}")
        finally:
            if put_final_azimuth_commanded_state:
                await self.evt_azimuthCommandedState.set_write(
                    commandedState=AzimuthCommandedState.STOP,
                    force_output=True,
                )

    async def do_moveShutterDropoutDoor(self, data):
        """Implement the ``moveShutterDropoutDoor`` command."""
        self.assert_enabled()
        await self._stop_move_az_task()
        if self.evt_mainDoorState.data.state != ShutterDoorState.OPENED:
            raise salobj.ExpectedError(
                "Cannot move the dropout door until the main door is fully open."
            )
        if data.open:
            await self.evt_dropoutDoorCommandedState.set_write(
                commandedState=ShutterDoorCommandedState.OPENED, force_output=True
            )
            await self.run_command("DN")
        else:
            await self.evt_dropoutDoorCommandedState.set_write(
                commandedState=ShutterDoorCommandedState.CLOSED, force_output=True
            )
            await self.run_command("UP")

        await self.wait_for_shutter(
            dropout_state=(
                ShutterDoorState.OPENED if data.open else ShutterDoorState.CLOSED
            ),
            main_state=None,
        )

    async def do_moveShutterMainDoor(self, data):
        """Implement the ``moveShutterMainDoor`` command."""
        self.assert_enabled()
        await self._stop_move_az_task()
        if data.open:
            await self.evt_mainDoorCommandedState.set_write(
                commandedState=ShutterDoorCommandedState.OPENED, force_output=True
            )
            await self.run_command("OP")
        else:
            if self.evt_dropoutDoorState.data.state not in (
                ShutterDoorState.CLOSED,
                ShutterDoorState.OPENED,
            ):
                raise salobj.ExpectedError(
                    "Cannot close the main door "
                    "until the dropout door is fully closed or fully open."
                )
            await self.evt_mainDoorCommandedState.set_write(
                commandedState=ShutterDoorCommandedState.CLOSED, force_output=True
            )
            await self.run_command("CL")

        await self.wait_for_shutter(
            dropout_state=None,
            main_state=(
                ShutterDoorState.OPENED if data.open else ShutterDoorState.CLOSED
            ),
        )

    async def run_command(self, cmd):
        """Send a command to the TCP/IP controller and process its replies.

        Parameters
        ----------
        cmd : `str`
            The command to send, e.g. "5.0 MV", "SO" or "+".

        Raises
        ------
        salobj.ExpectedError
            If communication fails. Also an exception is logged,
            the CSC disconnects from the low level controller,
            and goes into FAULT state.
            If the wrong number of lines is read. Also a warning is logged.
        """
        if not self.client.connected:
            if self.disabled_or_enabled and not self.connect_task.done():
                await self.connect_task
            else:
                raise RuntimeError("Not connected and not trying to connect")

        async with self.cmd_lock:
            if cmd == "?":
                # Turn short status into long status
                cmd = "+"
            await self.client.write_str(cmd)
            # Compute expected number of lines, ignoring the final prompt.
            expected_lines = 27 if cmd == "+" else 0

            read_bytes = await asyncio.wait_for(
                self.client.readuntil(b">"), timeout=self.config.read_timeout
            )
            data = read_bytes.decode()
            # Break into lines, dropping the final line
            # (which is just the prompt).
            lines = [elt.strip() for elt in data.split("\n")[:-1]]
            if len(lines) != expected_lines:
                err_msg = (
                    f"Command {cmd} returned {len(lines)} lines "
                    f"instead of {expected_lines}; read: {data!r}"
                )
                self.log.error(err_msg)
                raise salobj.ExpectedError(err_msg)
            if cmd == "+":
                await self.handle_status(lines)

    def compute_in_position_mask(self, move_code):
        """Compute in_position_mask.

        self.tel_position.data must be current.

        Parameters
        ----------
        move_code : `int`
            Motion code: the integer from line 5 of short status.

        Returns
        -------
        in_position_mask : `MoveCode`
            A bit mask with 1 for each axis that is in position.
        """
        mask = Axis(0)
        az_halted = (
            move_code & (MoveCode.AZIMUTH_POSITIVE | MoveCode.AZIMUTH_NEGATIVE) == 0
        )
        if (
            az_halted
            and self.evt_azimuthCommandedState.data.commandedState
            == AzimuthCommandedState.GOTOPOSITION
        ):
            daz = utils.angle_diff(
                self.tel_position.data.azimuthPosition,
                self.evt_azimuthCommandedState.data.azimuth,
            ).deg
            if abs(daz) < self.az_tolerance:
                mask |= Axis.AZIMUTH

        dropout_halted = (
            move_code & (MoveCode.DROPOUT_DOOR_CLOSING | MoveCode.DROPOUT_DOOR_OPENING)
            == 0
        )
        if dropout_halted:
            if (
                self.evt_dropoutDoorCommandedState.data.commandedState
                == ShutterDoorCommandedState.OPENED
            ):
                if self.tel_position.data.dropoutDoorOpeningPercentage == 100:
                    mask |= Axis.DROPOUT_DOOR
            elif (
                self.evt_dropoutDoorCommandedState.data.commandedState
                == ShutterDoorCommandedState.CLOSED
            ):
                if self.tel_position.data.dropoutDoorOpeningPercentage == 0:
                    mask |= Axis.DROPOUT_DOOR

        dropout_halted = (
            move_code & (MoveCode.DROPOUT_DOOR_CLOSING | MoveCode.DROPOUT_DOOR_OPENING)
            == 0
        )
        if dropout_halted:
            if (
                self.evt_dropoutDoorCommandedState.data.commandedState
                == ShutterDoorCommandedState.OPENED
            ):
                if self.tel_position.data.dropoutDoorOpeningPercentage == 100:
                    mask |= Axis.DROPOUT_DOOR
            elif (
                self.evt_dropoutDoorCommandedState.data.commandedState
                == ShutterDoorCommandedState.CLOSED
            ):
                if self.tel_position.data.dropoutDoorOpeningPercentage == 0:
                    mask |= Axis.DROPOUT_DOOR

        main_halted = (
            move_code & (MoveCode.MAIN_DOOR_CLOSING | MoveCode.MAIN_DOOR_OPENING) == 0
        )
        if main_halted:
            if (
                self.evt_mainDoorCommandedState.data.commandedState
                == ShutterDoorCommandedState.OPENED
            ):
                if self.tel_position.data.mainDoorOpeningPercentage == 100:
                    mask |= Axis.MAIN_DOOR
            elif (
                self.evt_mainDoorCommandedState.data.commandedState
                == ShutterDoorCommandedState.CLOSED
            ):
                if self.tel_position.data.mainDoorOpeningPercentage == 0:
                    mask |= Axis.MAIN_DOOR

        return mask

    def compute_az_state(self, move_code):
        """Compute the state field for the azimuthState event.

        Parameters
        ----------
        move_code : `int`
            Motion code: the integer from line 5 of short status.

        Returns
        -------
        state : `int`
            The appropriate `AzimuthState` enum value.
        """
        if move_code & MoveCode.AZIMUTH_POSITIVE:
            state = AzimuthState.MOVINGCW
        elif move_code & MoveCode.AZIMUTH_NEGATIVE:
            state = AzimuthState.MOVINGCCW
        else:
            state = AzimuthState.NOTINMOTION
        return state

    def compute_door_state(self, open_pct, is_main, move_code):
        """Compute data for the shutterState event.

        Parameters
        ----------
        open_pct : `float`
            Percent opening.
        is_main : `bool`
            True if the main door, False if the dropout door.
        move_code : `int`
            Motion code: the integer from line 5 of short status.
        """
        closing_code = (
            MoveCode.MAIN_DOOR_CLOSING if is_main else MoveCode.DROPOUT_DOOR_CLOSING
        )
        opening_code = (
            MoveCode.MAIN_DOOR_OPENING if is_main else MoveCode.DROPOUT_DOOR_OPENING
        )
        door_mask = closing_code | opening_code
        door_state = None
        if move_code & door_mask == 0:
            if open_pct == 0:
                door_state = ShutterDoorState.CLOSED
            elif open_pct == 100:
                door_state = ShutterDoorState.OPENED
            else:
                door_state = ShutterDoorState.PARTIALLYOPENED
        elif move_code & closing_code:
            door_state = ShutterDoorState.CLOSING
        elif move_code & opening_code:
            door_state = ShutterDoorState.OPENING
        if door_state is None:
            raise RuntimeError(
                f"Could not parse main door state from move_code={move_code}"
            )
        return door_state

    @staticmethod
    def get_config_pkg():
        return "ts_config_attcs"

    async def configure(self, config):
        self.config = config
        await self.evt_settingsAppliedDomeTcp.set_write(
            host=self.config.host,
            port=self.config.port,
            connectionTimeout=self.config.connection_timeout,
            readTimeout=self.config.read_timeout,
            force_output=True,
        )

    async def connect(self):
        """Connect to the dome controller's TCP/IP port.

        Start the mock controller, if simulating.
        """
        self.log.debug("connect")
        if self.config is None:
            raise RuntimeError("Not yet configured")
        if self.client.connected:
            raise RuntimeError("Already connected")
        if self.simulation_mode != 0:
            await self.start_mock_ctrl()
            host = self.mock_ctrl.host
            port = self.mock_ctrl.port
        else:
            host = self.config.host
            port = self.config.port
        try:
            async with self.cmd_lock:
                self.client = tcpip.Client(host=host, port=port, log=self.log)
                await asyncio.wait_for(
                    self.client.start_task, timeout=self.config.connection_timeout
                )
                # drop welcome message
                await asyncio.wait_for(
                    self.client.readuntil(b">"), timeout=self.config.read_timeout
                )
            self.log.debug("connected")
        except Exception as e:
            err_msg = f"Could not open connection to host={host}, port={port}: {e!r}"
            self.log.exception(err_msg)
            await self.fault(code=ErrorCode.TCPIP_CONNECT_ERROR, report=err_msg)
            return

        self.status_task = asyncio.ensure_future(self.status_loop())

    async def end_disable(self, data):
        """End do_disable; called after state changes
        but before command acknowledged.

        Stop azimuth motion and close the shutters, then disconnect.

        Parameters
        ----------
        data : `DataType`
            Command data
        """
        self.shutter_task.cancel()
        if not self.client.connected:
            # this should never happen, but be paranoid
            return

        # Halt all motion, if possible. We just want to stop azimuth,
        # but there is no way to do that.
        await self.evt_azimuthCommandedState.set_write(
            commandedState=AzimuthCommandedState.STOP, force_output=True
        )
        try:
            await self.run_command("ST")
        except salobj.ExpectedError:
            # We tried. A message has been logged.
            pass

        # Close the shutter, if possible.
        await self.evt_dropoutDoorCommandedState.set_write(
            commandedState=ShutterDoorCommandedState.CLOSED, force_output=True
        )
        await self.evt_mainDoorCommandedState.set_write(
            commandedState=ShutterDoorCommandedState.CLOSED, force_output=True
        )
        try:
            await self.run_command("SC")
        except salobj.ExpectedError:
            # We tried. A message has been logged.
            pass
        self.status_sleep_task.cancel()

    async def disconnect(self):
        """Disconnect from the TCP/IP controller, if connected, and stop
        the mock controller, if running.
        """
        self.log.debug("disconnect")
        self.connect_task.cancel()
        self.status_sleep_task.cancel()
        await self.client.close()
        if not self.status_task.done():
            await asyncio.wait_for(
                self.status_task, timeout=self.config.read_timeout * 2
            )
        await self.stop_mock_ctrl()

    async def handle_status(self, lines):
        """Handle output of "?" command.

        Parameters
        ----------
        lines : `iterable` of `str`
            Lines of output from "?", the short status command, or the
            first 5 lines of output from the full status command "+".

        Returns
        -------
        settingsAppliedDomeController : `bool`
            True if ``self.evt_settingsAppliedDomeController`` updated.
        """
        status = Status(lines)

        # TODO: DM-23808 the azimuthEncoderPosition isn't big enough.
        # Once that is fixed, ditch the try/except and set the field normally.
        try:
            self.tel_position.set(azimuthEncoderPosition=status.encoder_counts)
        except ValueError:
            self.log.warning(
                f"status.encoder_counts={status.encoder_counts} too big for SAL topic!"
            )
            self.tel_position.set(azimuthEncoderPosition=0)
        await self.tel_position.set_write(
            mainDoorOpeningPercentage=status.main_door_pct,
            dropoutDoorOpeningPercentage=status.dropout_door_pct,
            azimuthPosition=status.az_pos,
        )

        move_code = status.move_code
        await self.evt_moveCode.set_write(code=move_code)
        await self.evt_azimuthState.set_write(
            state=self.compute_az_state(move_code),
            homed=status.homed,
            homing=bool(move_code & MoveCode.AZIMUTH_HOMING),
            homeSwitch=status.az_home_switch,
        )

        dropout_door_state = self.compute_door_state(
            open_pct=self.tel_position.data.dropoutDoorOpeningPercentage,
            is_main=False,
            move_code=move_code,
        )
        main_door_state = self.compute_door_state(
            open_pct=self.tel_position.data.mainDoorOpeningPercentage,
            is_main=True,
            move_code=move_code,
        )
        await self.evt_dropoutDoorState.set_write(state=dropout_door_state)
        await self.evt_mainDoorState.set_write(state=main_door_state)

        await self.evt_emergencyStop.set_write(active=move_code & MoveCode.ESTOP > 0)

        in_position_mask = self.compute_in_position_mask(move_code)

        def in_position(mask):
            return in_position_mask & mask == mask

        azimuth_in_position = in_position(Axis.AZIMUTH)
        shutter_in_position = in_position(Axis.DROPOUT_DOOR | Axis.MAIN_DOOR)
        await self.evt_azimuthInPosition.set_write(inPosition=azimuth_in_position)
        await self.evt_shutterInPosition.set_write(inPosition=shutter_in_position)
        await self.evt_allAxesInPosition.set_write(
            inPosition=azimuth_in_position and shutter_in_position
        )

        if not self.shutter_task.done():
            end_shutter_task = True
            if self.desired_dropout_shutter_state is not None:
                if self.desired_dropout_shutter_state != dropout_door_state:
                    end_shutter_task = False
            if self.desired_main_shutter_state is not None:
                if self.desired_main_shutter_state != main_door_state:
                    end_shutter_task = False
            if end_shutter_task:
                self.shutter_task.set_result(None)

        await self.evt_emergencyStop.set_write(active=status.estop_active)

        await self.evt_scbLink.set_write(active=status.scb_link_ok)

        await self.evt_doorEncoderExtremes.set_write(
            mainClosed=status.main_door_encoder_closed,
            mainOpened=status.main_door_encoder_opened,
            dropoutClosed=status.dropout_door_encoder_closed,
            dropoutOpened=status.dropout_door_encoder_opened,
        )

        await self.evt_lastAzimuthGoTo.set_write(position=status.last_azimuth_goto)

        await self.evt_settingsAppliedDomeController.set_write(
            rainSensorEnabled=status.rain_sensor_enabled,
            cloudSensorEnabled=status.cloud_sensor_enabled,
            tolerance=status.tolerance,
            homeAzimuth=status.home_azimuth,
            highSpeedDistance=status.high_speed,
            watchdogTimer=status.watchdog_timer,
            dropoutTimer=status.dropout_timer,
            reversalDelay=status.reversal_delay,
            autoShutdownEnabled=status.auto_shutdown_enabled,
            coast=status.coast,
            encoderCountsPer360=status.encoder_counts_per_360,
            azimuthMoveTimeout=status.azimuth_move_timeout,
            doorMoveTimeout=status.door_move_timeout,
        )

        self.az_tolerance = status.tolerance + self.az_tolerance_margin

        self.status_event.set()

    async def wait_for_shutter(self, *, dropout_state, main_state):
        """Wait for the shutter doors to move to a specified position.

        Cancel an existing wait, if any, trigger a status update,
        and let the status update set self.shutter_task done.

        Parameters
        ----------
        dropout_state : `lsst.ts.idl.enums.ATDome.ShutterDoorState` or `None`
            Desired state of dropout door.
        main_state : `lsst.ts.idl.enums.ATDome.ShutterDoorState` or `None`
            Desired state of main door.

        Notes
        -----
        Triggers an immediate status update.
        """
        allowed_values = (ShutterDoorState.OPENED, ShutterDoorState.CLOSED, None)
        if dropout_state not in allowed_values:
            raise ValueError(
                f"dropout_state={dropout_state!r}; must be one of {allowed_values}"
            )
        if main_state not in allowed_values:
            raise ValueError(
                f"main_state={main_state!r}; must be one of {allowed_values}"
            )
        if dropout_state is None and main_state is None:
            raise ValueError("dropout_state and main_state cannot both be None")
        self.shutter_task.cancel()
        self.desired_dropout_shutter_state = dropout_state
        self.desired_main_shutter_state = main_state
        self.shutter_task = asyncio.Future()
        self.status_sleep_task.cancel()
        await self.shutter_task

    async def start_mock_ctrl(self):
        """Start the mock controller.

        The simulation mode must be 1.
        """
        try:
            assert self.simulation_mode == 1
            self.mock_ctrl = MockDomeController(port=0, log=self.log)
            await asyncio.wait_for(
                self.mock_ctrl.start_task, timeout=MOCK_CTRL_START_TIMEOUT
            )
        except Exception as e:
            err_msg = f"Could not start mock controller: {e!r}"
            self.log.exception(err_msg)
            await self.fault(
                code=ErrorCode.CANNOT_START_MOCK_CONTROLLER, report=err_msg
            )
            raise

    async def handle_summary_state(self):
        if self.disabled_or_enabled:
            if not self.client.connected and self.connect_task.done():
                await self.connect()
        else:
            await self.disconnect()

    async def start(self):
        await super().start()
        await self.evt_azimuthCommandedState.set_write(
            commandedState=AzimuthCommandedState.UNKNOWN,
            azimuth=math.nan,
            force_output=True,
        )
        await self.evt_dropoutDoorCommandedState.set_write(
            commandedState=ShutterDoorCommandedState.UNKNOWN, force_output=True
        )
        await self.evt_mainDoorCommandedState.set_write(
            commandedState=ShutterDoorCommandedState.UNKNOWN, force_output=True
        )

    async def status_loop(self):
        """Read and report status from the TCP/IP controller."""
        self.status_sleep_task.cancel()
        while self.client.connected:
            try:
                await self.run_command("+")
            except Exception as e:
                self.log.exception(
                    f"Status request failed; status loop continues: {e!r}"
                )
            try:
                self.status_sleep_task = asyncio.ensure_future(
                    asyncio.sleep(self.status_interval)
                )
                await self.status_sleep_task
            except asyncio.CancelledError:
                pass

    async def close_tasks(self):
        """Disconnect from the TCP/IP controller, if connected, and stop
        the mock controller, if running.
        """
        if not self.move_azimuth_task.done():
            self.move_azimuth_task.cancel()
            try:
                await self.move_azimuth_task
            except asyncio.CancelledError:
                pass
            except Exception:
                self.log.exception("Error in move azimuth task.")
        await super().close_tasks()
        await self.disconnect()

    async def stop_mock_ctrl(self):
        """Stop the mock controller, if running."""
        mock_ctrl = self.mock_ctrl
        self.mock_ctrl = None
        if mock_ctrl:
            await mock_ctrl.close()

    async def wait_n_status(self, n=2):
        """Wait for the specified number of status."""
        for i in range(n):
            self.status_event.clear()
            await self.status_event.wait()

    async def _handle_azimuth_move(self, position):
        """Handle azimuth move.

        This coroutine will monitor the position of the dome and make
        sure the dome is moving towards the specified position. If the
        dome stops before arriving in position, it will send the move
        command again.

        Parameters
        ----------
        position : `float`
            The desired position of the dome.
        """

        await asyncio.sleep(self.status_interval * 4)
        dome_current_position = self.tel_position.data.azimuthPosition
        dome_move_below_threshold = 0
        total_retries = 0

        self.log.debug(
            f"Starting azimuth motion handler; {dome_current_position=}, {position=}."
        )
        while utils.angle_diff(dome_current_position, position).deg > self.az_tolerance:
            await asyncio.sleep(self.status_interval * 4)
            dome_new_position = self.tel_position.data.azimuthPosition
            if (
                utils.angle_diff(dome_current_position, dome_new_position).deg
                < self.az_move_tol
            ):
                dome_move_below_threshold += 1
                self.log.debug(
                    f"Dome did not move substantially {dome_move_below_threshold=}; "
                    f"{dome_current_position=}, {dome_new_position}."
                )
            else:
                dome_move_below_threshold = 0
            dome_current_position = dome_new_position
            if dome_move_below_threshold > self.max_dome_move_below_threshold:
                self.log.info(
                    "Dome is not moving, resending move command: "
                    f"{dome_current_position=} {position=}."
                )
                dome_move_below_threshold = 0
                total_retries += 1
                await self.run_command(f"{position:0.3f} MV")

        self.log.debug(
            f"Dome arrived in {position=}; {total_retries=}. Done monitoring loop."
        )

    async def _stop_move_az_task(self):

        if not self.move_azimuth_task.done():
            self.log.debug("Dome move task active; stopping.")
            self.move_azimuth_task.cancel()
            try:
                await self.move_azimuth_task
            except asyncio.CancelledError:
                pass
            except Exception:
                self.log.exception("Error cancelling background move task.")


def run_atdome():
    """Run the ATDome CSC."""
    asyncio.run(ATDomeCsc.amain(index=None))

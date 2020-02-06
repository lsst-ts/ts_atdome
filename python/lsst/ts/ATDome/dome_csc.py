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

__all__ = ["ATDomeCsc"]

import asyncio
import enum
import math
import pathlib

from astropy.coordinates import Angle
import astropy.units as u

from lsst.ts import salobj
from lsst.ts.idl.enums.ATDome import (
    AzimuthCommandedState,
    AzimuthState,
    ShutterDoorCommandedState,
    ShutterDoorState,
)
from .mock_controller import MockDomeController
from .status import Status

_LOCAL_HOST = "127.0.0.1"


class MoveCode(enum.IntFlag):
    AZPOSITIVE = 1
    AZNEGATIVE = 2
    MAINDOORCLOSING = 4
    MAINDOOROPENING = 8
    DROPOUTDOORCLOSING = 16
    DROPOUTDOOROPENING = 32
    HOMING = 64
    ESTOP = 128


class Axis(enum.Flag):
    AZ = enum.auto()
    DROPOUTDOOR = enum.auto()
    MAINDOOR = enum.auto()


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
    mock_port : `int` (optional)
        Port for mock controller TCP/IP interface. If `None` then use the
        port specified by the configuration. Only used in simulation mode.

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

    def __init__(
        self,
        config_dir=None,
        initial_state=salobj.State.STANDBY,
        simulation_mode=0,
        mock_port=None,
    ):
        schema_path = (
            pathlib.Path(__file__)
            .resolve()
            .parents[4]
            .joinpath("schema", "ATDome.yaml")
        )

        self.reader = None
        self.writer = None
        self.move_code = 0
        self.mock_ctrl = None  # mock controller, or None of not constructed
        self.status_interval = 0.2  # delay between short status commands (sec)
        self.az_tolerance = Angle(0.2, u.deg)  # tolerance for "in position"
        # Task for sleeping in the status loop; cancel this to trigger
        # an immediate status update. Warning: do not cancel status_task
        # because that may be waiting for TCP/IP communication.
        self.status_sleep_task = salobj.make_done_future()
        # Task for the status loop. To trigger new status cancel
        # status_sleep_task, not status_task.
        self.status_task = salobj.make_done_future()
        # Task that waits while connecting to the TCP/IP controller.
        self.connect_task = salobj.make_done_future()
        # Task that waits while shutter doors move
        self.shutter_task = salobj.make_done_future()
        # The conditions that self.shutter_task is waiting for.
        # Must be one of: ShutterDoorState.OPENED,
        # ShutterDoorState.CLOSED or None (for don't care)
        self.desired_main_shutter_state = None
        self.desired_dropout_shutter_state = None
        self.cmd_lock = asyncio.Lock()
        self.config = None
        self.mock_port = mock_port
        super().__init__(
            "ATDome",
            index=0,
            schema_path=schema_path,
            config_dir=config_dir,
            initial_state=initial_state,
            simulation_mode=simulation_mode,
        )

    async def do_moveAzimuth(self, data):
        """Implement the ``moveAzimuth`` command."""
        self.assert_enabled("moveAzimuth")
        if self.evt_azimuthState.data.homing:
            raise salobj.ExpectedError("Cannot move azimuth while homing")
        azimuth = data.azimuth
        if azimuth < 0 or azimuth > 360:
            raise salobj.ExpectedError(
                f"azimuth={azimuth} deg; must be in range [0, 360]"
            )
        await self.run_command(f"{azimuth:0.3f} MV")
        self.evt_azimuthCommandedState.set_put(
            commandedState=AzimuthCommandedState.GOTOPOSITION,
            azimuth=azimuth,
            force_output=True,
        )
        self.status_sleep_task.cancel()

    async def do_closeShutter(self, data):
        """Implement the ``closeShutter`` command."""
        self.assert_enabled("closeShutter")
        self.shutter_task.cancel()
        await self.run_command("SC")
        self.evt_dropoutDoorCommandedState.set_put(
            commandedState=ShutterDoorCommandedState.CLOSED, force_output=True
        )
        self.evt_mainDoorCommandedState.set_put(
            commandedState=ShutterDoorCommandedState.CLOSED, force_output=True
        )
        await self.wait_for_shutter(
            dropout_state=ShutterDoorState.CLOSED, main_state=ShutterDoorState.CLOSED
        )

    async def do_openShutter(self, data):
        """Implement the ``openShutter`` command."""
        self.assert_enabled("openShutter")
        await self.run_command("SO")
        self.evt_dropoutDoorCommandedState.set_put(
            commandedState=ShutterDoorCommandedState.OPENED, force_output=True
        )
        self.evt_mainDoorCommandedState.set_put(
            commandedState=ShutterDoorCommandedState.OPENED, force_output=True
        )
        await self.wait_for_shutter(
            dropout_state=ShutterDoorState.OPENED, main_state=ShutterDoorState.OPENED
        )

    async def do_stopMotion(self, data):
        """Implement the ``stopMotion`` command."""
        self.assert_enabled("stopMotion")
        self.evt_azimuthCommandedState.set_put(
            commandedState=AzimuthCommandedState.STOP, force_output=True
        )
        self.evt_dropoutDoorCommandedState.set_put(
            commandedState=ShutterDoorCommandedState.STOP, force_output=True
        )
        self.evt_mainDoorCommandedState.set_put(
            commandedState=ShutterDoorCommandedState.STOP, force_output=True
        )
        await self.run_command("ST")
        self.shutter_task.cancel()
        self.status_sleep_task.cancel()

    async def do_homeAzimuth(self, data):
        """Implement the ``homeAzimuth`` command."""
        self.assert_enabled("homeAzimuth")
        if self.evt_azimuthState.data.homing:
            raise salobj.ExpectedError("Already homing")
        self.evt_azimuthCommandedState.set_put(
            commandedState=AzimuthCommandedState.HOME,
            azimuth=math.nan,
            force_output=True,
        )
        await self.run_command("HM")
        self.status_sleep_task.cancel()

    async def do_moveShutterDropoutDoor(self, data):
        """Implement the ``moveShutterDropoutDoor`` command."""
        self.assert_enabled("moveShutterDropoutDoor")
        if self.evt_mainDoorState.data.state != ShutterDoorState.OPENED:
            raise salobj.ExpectedError(
                "Cannot move the dropout door until the main door is fully open."
            )
        if data.open:
            self.evt_dropoutDoorCommandedState.set_put(
                commandedState=ShutterDoorCommandedState.OPENED, force_output=True
            )
            await self.run_command("DN")
        else:
            self.evt_dropoutDoorCommandedState.set_put(
                commandedState=ShutterDoorCommandedState.CLOSED, force_output=True
            )
            await self.run_command("UP")

        await self.wait_for_shutter(
            dropout_state=ShutterDoorState.OPENED
            if data.open
            else ShutterDoorState.CLOSED,
            main_state=None,
        )

    async def do_moveShutterMainDoor(self, data):
        """Implement the ``moveShutterMainDoor`` command."""
        self.assert_enabled("moveShutterMainDoor")
        if data.open:
            self.evt_mainDoorCommandedState.set_put(
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
            self.evt_mainDoorCommandedState.set_put(
                commandedState=ShutterDoorCommandedState.CLOSED, force_output=True
            )
            await self.run_command("CL")

        await self.wait_for_shutter(
            dropout_state=None,
            main_state=ShutterDoorState.OPENED
            if data.open
            else ShutterDoorState.CLOSED,
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
        if not self.connected:
            if self.disabled_or_enabled and not self.connect_task.done():
                await self.connect_task
            else:
                raise RuntimeError("Not connected and not trying to connect")

        async with self.cmd_lock:
            self.writer.write(f"{cmd}\r\n".encode())
            await self.writer.drain()
            if cmd == "?":
                # Turn short status into long status
                cmd = "+"
            expected_lines = {"+": 25}.get(cmd, 0)  # excluding final ">" line

            try:
                read_bytes = await asyncio.wait_for(
                    self.reader.readuntil(">".encode()),
                    timeout=self.config.read_timeout,
                )
            except Exception as e:
                if isinstance(e, asyncio.streams.IncompleteReadError):
                    err_msg = "TCP/IP controller exited"
                else:
                    err_msg = "TCP/IP read failed"
                self.log.exception(err_msg)
                await self.disconnect()
                self.fault(code=2, report=f"{err_msg}: {e}")
                raise salobj.ExpectedError(err_msg)

            data = read_bytes.decode()
            lines = data.split("\n")[:-1]  # strip final > line
            lines = [elt.strip() for elt in lines]
            if len(lines) != expected_lines:
                self.log.warning(
                    f"Command {cmd} returned {data}; expected {expected_lines} lines"
                )
                raise salobj.ExpectedError(err_msg)
            elif cmd == "+":
                self.handle_status(lines)

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
        az_halted = move_code & (MoveCode.AZPOSITIVE | MoveCode.AZNEGATIVE) == 0
        if (
            az_halted
            and self.evt_azimuthCommandedState.data.commandedState
            == AzimuthCommandedState.GOTOPOSITION
        ):
            daz = salobj.angle_diff(
                self.tel_position.data.azimuthPosition,
                self.evt_azimuthCommandedState.data.azimuth,
            )
            if abs(daz) < self.az_tolerance:
                mask |= Axis.AZ

        dropout_halted = (
            move_code & (MoveCode.DROPOUTDOORCLOSING | MoveCode.DROPOUTDOOROPENING) == 0
        )
        if dropout_halted:
            if (
                self.evt_dropoutDoorCommandedState.data.commandedState
                == ShutterDoorCommandedState.OPENED
            ):
                if self.tel_position.data.dropoutDoorOpeningPercentage == 100:
                    mask |= Axis.DROPOUTDOOR
            elif (
                self.evt_dropoutDoorCommandedState.data.commandedState
                == ShutterDoorCommandedState.CLOSED
            ):
                if self.tel_position.data.dropoutDoorOpeningPercentage == 0:
                    mask |= Axis.DROPOUTDOOR

        dropout_halted = (
            move_code & (MoveCode.DROPOUTDOORCLOSING | MoveCode.DROPOUTDOOROPENING) == 0
        )
        if dropout_halted:
            if (
                self.evt_dropoutDoorCommandedState.data.commandedState
                == ShutterDoorCommandedState.OPENED
            ):
                if self.tel_position.data.dropoutDoorOpeningPercentage == 100:
                    mask |= Axis.DROPOUTDOOR
            elif (
                self.evt_dropoutDoorCommandedState.data.commandedState
                == ShutterDoorCommandedState.CLOSED
            ):
                if self.tel_position.data.dropoutDoorOpeningPercentage == 0:
                    mask |= Axis.DROPOUTDOOR

        main_halted = (
            move_code & (MoveCode.MAINDOORCLOSING | MoveCode.MAINDOOROPENING) == 0
        )
        if main_halted:
            if (
                self.evt_mainDoorCommandedState.data.commandedState
                == ShutterDoorCommandedState.OPENED
            ):
                if self.tel_position.data.mainDoorOpeningPercentage == 100:
                    mask |= Axis.MAINDOOR
            elif (
                self.evt_mainDoorCommandedState.data.commandedState
                == ShutterDoorCommandedState.CLOSED
            ):
                if self.tel_position.data.mainDoorOpeningPercentage == 0:
                    mask |= Axis.MAINDOOR

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
        if move_code & MoveCode.AZPOSITIVE:
            state = AzimuthState.MOVINGCW
        elif move_code & MoveCode.AZNEGATIVE:
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
            MoveCode.MAINDOORCLOSING if is_main else MoveCode.DROPOUTDOORCLOSING
        )
        opening_code = (
            MoveCode.MAINDOOROPENING if is_main else MoveCode.DROPOUTDOOROPENING
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
        self.evt_settingsAppliedDomeTcp.set_put(
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
        if self.connected:
            raise RuntimeError("Already connected")
        host = _LOCAL_HOST if self.simulation_mode == 1 else self.config.host
        if self.simulation_mode == 1:
            await self.start_mock_ctrl()
            host = _LOCAL_HOST
        else:
            host = self.config.host
        try:
            async with self.cmd_lock:
                if self.simulation_mode != 0:
                    if self.mock_ctrl is None:
                        raise RuntimeError(
                            "In simulation mode but no mock controller found."
                        )
                    port = self.mock_ctrl.port
                else:
                    port = self.config.port
                connect_coro = asyncio.open_connection(host=host, port=port)
                self.reader, self.writer = await asyncio.wait_for(
                    connect_coro, timeout=self.config.connection_timeout
                )
                # drop welcome message
                await asyncio.wait_for(
                    self.reader.readuntil(">".encode()),
                    timeout=self.config.read_timeout,
                )
            self.log.debug("connected")
        except Exception as e:
            err_msg = (
                f"Could not open connection to host={host}, port={self.config.port}"
            )
            self.log.exception(err_msg)
            self.fault(code=1, report=f"{err_msg}: {e}")
            return

        self.status_task = asyncio.ensure_future(self.status_loop())

    @property
    def connected(self):
        if None in (self.reader, self.writer):
            return False
        return True

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
        if not self.connected:
            # this should never happen, but be paranoid
            return

        # Halt all motion, if possible. We just want to stop azimuth,
        # but there is no way to do that.
        self.evt_azimuthCommandedState.set_put(
            commandedState=AzimuthCommandedState.STOP, force_output=True
        )
        try:
            await self.run_command("ST")
        except salobj.ExpectedError:
            # We tried. A message has been logged.
            pass

        # Close the shutter, if possible.
        self.evt_dropoutDoorCommandedState.set_put(
            commandedState=ShutterDoorCommandedState.CLOSED, force_output=True
        )
        self.evt_mainDoorCommandedState.set_put(
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
        writer = self.writer
        self.reader = None
        self.writer = None
        if writer:
            try:
                writer.write_eof()
                await asyncio.wait_for(writer.drain(), timeout=2)
            finally:
                writer.close()
        self.status_sleep_task.cancel()
        if not self.status_task.done():
            await asyncio.wait_for(
                self.status_task, timeout=self.config.read_timeout * 2
            )
        await self.stop_mock_ctrl()

    def handle_status(self, lines):
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

        self.tel_position.set_put(
            mainDoorOpeningPercentage=status.main_door_pct,
            dropoutDoorOpeningPercentage=status.dropout_door_pct,
            azimuthPosition=status.az_pos.deg,
            azimuthEncoderPosition=status.encoder_counts,
        )

        move_code = status.move_code
        self.evt_azimuthState.set_put(
            state=self.compute_az_state(move_code),
            homing=bool(move_code & MoveCode.HOMING),
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
        self.evt_dropoutDoorState.set_put(state=dropout_door_state)
        self.evt_mainDoorState.set_put(state=main_door_state)

        self.evt_emergencyStop.set_put(active=move_code & MoveCode.ESTOP > 0)

        in_position_mask = self.compute_in_position_mask(move_code)

        def in_position(mask):
            return in_position_mask & mask == mask

        azimuth_in_position = in_position(Axis.AZ)
        shutter_in_position = in_position(Axis.DROPOUTDOOR | Axis.MAINDOOR)
        self.evt_azimuthInPosition.set_put(inPosition=azimuth_in_position)
        self.evt_shutterInPosition.set_put(inPosition=shutter_in_position)
        self.evt_allAxesInPosition.set_put(
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

        self.evt_emergencyStop.set_put(active=status.estop_active)

        self.evt_scbLink.set_put(active=status.scb_link_ok)

        self.evt_doorEncoderExtremes.set_put(
            mainClosed=status.main_door_encoder_closed,
            mainOpened=status.main_door_encoder_opened,
            dropoutClosed=status.dropout_door_encoder_closed,
            dropoutOpened=status.dropout_door_encoder_opened,
        )

        self.evt_lastAzimuthGoTo.set_put(position=status.last_azimuth_goto)

        self.evt_settingsAppliedDomeController.set_put(
            rainSensorEnabled=status.rain_sensor_enabled,
            cloudSensorEnabled=status.cloud_sensor_enabled,
            tolerance=status.tolerance.deg,
            homeAzimuth=status.home_azimuth.deg,
            highSpeedDistance=status.high_speed.deg,
            watchdogTimer=status.watchdog_timer,
            dropoutTimer=status.dropout_timer,
            reversalDelay=status.reversal_delay,
            autoShutdownEnabled=status.auto_shutdown_enabled,
            coast=status.coast.deg,
            encoderCountsPer360=status.encoder_counts_per_360,
            azimuthMoveTimeout=status.azimuth_move_timeout,
            doorMoveTimeout=status.door_move_timeout,
        )

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
            raise ValueError(f"dropout_state and main_state cannot both be None")
        self.shutter_task.cancel()
        self.desired_dropout_shutter_state = dropout_state
        self.desired_main_shutter_state = main_state
        self.shutter_task = asyncio.Future()
        self.status_sleep_task.cancel()
        await self.shutter_task

    async def implement_simulation_mode(self, simulation_mode):
        if simulation_mode not in (0, 1):
            raise salobj.ExpectedError(
                f"Simulation_mode={simulation_mode} must be 0 or 1"
            )

    async def start_mock_ctrl(self):
        """Start the mock controller.

        The simulation mode must be 1.
        """
        try:
            assert self.simulation_mode == 1
            if self.mock_port is not None:
                port = self.mock_port
            else:
                port = self.config.port
            self.mock_ctrl = MockDomeController(port=port)
            await asyncio.wait_for(self.mock_ctrl.start(), timeout=2)
        except Exception as e:
            err_msg = "Could not start mock controller"
            self.log.exception(e)
            self.fault(code=3, report=f"{err_msg}: {e}")
            raise

    async def handle_summary_state(self):
        if self.disabled_or_enabled:
            if not self.connected and self.connect_task.done():
                await self.connect()
        else:
            await self.disconnect()

    async def start(self):
        await super().start()
        self.evt_azimuthCommandedState.set_put(
            commandedState=AzimuthCommandedState.UNKNOWN,
            azimuth=math.nan,
            force_output=True,
        )
        self.evt_dropoutDoorCommandedState.set_put(
            commandedState=ShutterDoorCommandedState.UNKNOWN, force_output=True
        )
        self.evt_mainDoorCommandedState.set_put(
            commandedState=ShutterDoorCommandedState.UNKNOWN, force_output=True
        )

    async def status_loop(self):
        """Read and report status from the TCP/IP controller.
        """
        self.status_sleep_task.cancel()
        while self.connected:
            try:
                await self.run_command("+")
            except Exception as e:
                self.log.warning(f"Status request failed: {e}")
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
        await super().close_tasks()
        await self.disconnect()

    async def stop_mock_ctrl(self):
        """Stop the mock controller, if running.
        """
        mock_ctrl = self.mock_ctrl
        self.mock_ctrl = None
        if mock_ctrl:
            await mock_ctrl.stop()

    @classmethod
    def add_arguments(cls, parser):
        super(ATDomeCsc, cls).add_arguments(parser)
        parser.add_argument(
            "-s", "--simulate", action="store_true", help="Run in simuation mode?"
        )

    @classmethod
    def add_kwargs_from_args(cls, args, kwargs):
        super(ATDomeCsc, cls).add_kwargs_from_args(args, kwargs)
        kwargs["simulation_mode"] = 1 if args.simulate else 0

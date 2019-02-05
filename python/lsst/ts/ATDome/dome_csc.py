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

from astropy.coordinates import Angle
import astropy.units as u

from lsst.ts import salobj
from .utils import angle_diff
from .mock_controller import MockDomeController
from .status import ShortStatus, RemainingStatus

import SALPY_ATDome

_LOCAL_HOST = "127.0.0.1"


class MoveCode(enum.IntFlag):
    AzPositive = 1
    AzNegative = 2
    MainDoorClosing = 4
    MainDoorOpening = 8
    DropoutDoorClosing = 16
    DropoutDoorOpening = 32
    Homing = 64
    EStop = 128


class Axis(enum.Flag):
    Az = enum.auto()
    DropoutDoor = enum.auto()
    MainDoor = enum.auto()


class ATDomeCsc(salobj.BaseCsc):
    """AuxTel dome CSC

    Parameters
    ----------
    index : `int` or `None`
        SAL component index, or 0 or None if the component is not indexed.
    port : `int`
        TCP/IP port for ATDome controller.
    initial_state : `salobj.State` or `int` (optional)
        The initial state of the CSC. This is provided for unit testing,
        as real CSCs should start up in `State.STANDBY`, the default.
    initial_simulation_mode : `int` (optional)
        Initial simulation mode.

    Raises
    ------
    salobj.ExpectedError
        If initial_state or initial_simulation_mode is invalid.

    Notes
    -----
    Supported simulation modes:

    * 0: regular operation
    * 1: simulation mode: start a mock TCP/IP ATDome controller and talk to it
    """
    def __init__(self, index, initial_state=salobj.State.STANDBY, initial_simulation_mode=0):
        self.reader = None
        self.writer = None
        self.cmd_queue = asyncio.Queue()
        self.move_code = 0
        self.mock_ctrl = None  # mock controller, or None of not constructed
        self.status_task = None
        self.status_interval = 0.2  # delay between short status commands (sec)
        self.n_short_status = 0
        self.short_per_full = 5  # number of short status between full status
        self.az_tolerance = Angle(0.2, u.deg)  # tolerance for "in position"
        self.status_sleep_task = None  # sleep in status_loop
        super().__init__(SALPY_ATDome, index=index, initial_state=initial_state,
                         initial_simulation_mode=initial_simulation_mode)
        self.configure()
        # initialize commanded positions
        self.tel_position.set(azimuthPositionSet=math.nan,
                              dropoutDoorOpeningPercentageSet=math.nan,
                              mainDoorOpeningPercentageSet=math.nan)

    async def do_moveAzimuth(self, id_data):
        self.assert_enabled("moveAzimuth")
        if self.evt_azimuthState.data.homing:
            raise salobj.ExpectedError("Cannot move azimuth while homing")
        azimuth = id_data.data.azimuth
        if azimuth < 0 or azimuth > 360:
            raise salobj.ExpectedError(f"azimuth={azimuth} deg; must be in range [0, 360]")
        await self.cmd_queue.put(f"{azimuth:0.3f} MV")
        self.tel_position.set_put(azimuthPositionSet=azimuth)
        self.status_loop()

    async def do_closeShutter(self, id_data):
        self.assert_enabled("closeShutter")
        await self.cmd_queue.put("SC")
        self.tel_position.set_put(dropoutDoorOpeningPercentageSet=0,
                                  mainDoorOpeningPercentageSet=0)
        self.status_loop()

    async def do_openShutter(self, id_data):
        self.assert_enabled("openShutter")
        await self.cmd_queue.put("SO")
        self.tel_position.set_put(dropoutDoorOpeningPercentageSet=100,
                                  mainDoorOpeningPercentageSet=100)
        self.status_loop()

    async def do_stopMotion(self, id_data):
        self.assert_enabled("stopMotion")
        await self.cmd_queue.put("ST")
        self.status_loop()

    async def do_homeAzimuth(self, id_data):
        self.assert_enabled("homeAzimuth")
        if self.evt_azimuthState.data.homing:
            raise salobj.ExpectedError("Already homing")
        await self.cmd_queue.put("HM")
        self.tel_position.set(azimuthPositionSet=math.nan)
        self.status_loop()

    async def do_moveShutterDropoutDoor(self, id_data):
        self.assert_enabled("moveShutterDropoutDoor")
        if self.evt_mainDoorState.data.state != SALPY_ATDome.ATDome_shared_ShutterDoorState_OpenedState:
            raise salobj.ExpectedError("Cannot move the dropout door until the main door is fully open.")
        if id_data.data.open:
            await self.cmd_queue.put("DN")
            amount = 100
        else:
            await self.cmd_queue.put("UP")
            amount = 0
        self.tel_position.set(dropoutDoorOpeningPercentageSet=amount)
        self.status_loop()

    async def do_moveShutterMainDoor(self, id_data):
        self.assert_enabled("moveShutterMainDoor")
        if id_data.data.open:
            await self.cmd_queue.put("OP")
            amount = 100
        else:
            if self.evt_mainDoorState.data.state not in (
                    SALPY_ATDome.ATDome_shared_ShutterDoorState_ClosedState,
                    SALPY_ATDome.ATDome_shared_ShutterDoorState_OpenedState):
                raise salobj.ExpectedError("Cannot close the main door "
                                           "until the dropout door is fully closed or fully open.")
            await self.cmd_queue.put("CL")
            amount = 0
        self.tel_position.set_put(mainDoorOpeningPercentageSet=amount)
        self.status_loop()

    async def cmd_loop(self):
        while self.connected:
            cmd = await self.cmd_queue.get()
            self.writer.write(f"{cmd}\n".encode())
            await self.writer.drain()
            expected_lines = {  # excluding final ">" line
                "?": 5,
                "+": 23,
            }.get(cmd, 0)

            try:
                read_bytes = await asyncio.wait_for(self.reader.readuntil(">".encode()),
                                                    timeout=self.read_timeout)
            except Exception as e:
                if isinstance(e, asyncio.streams.IncompleteReadError):
                    err_msg = "TCP/IP controller exited"
                else:
                    err_msg = "TCP/IP read failed"
                self.log.exception(err_msg)
                await self.disconnect()
                self.summary_state = salobj.State.FAULT
                self.evt_errorCode.set_put(errorCode=1, errorReport=f"{err_msg}: {e}", force_output=True)
                return

            data = read_bytes.decode()
            lines = data.split("\n")[:-1]  # strip final > line
            if len(lines) != expected_lines:
                self.log.error(f"Read {data} but expected {expected_lines} lines")
            if cmd == "?":
                if self.handle_short_status(lines):
                    self.evt_settingsAppliedDomeController.put()
            elif cmd == "+":
                self.handle_full_status(lines)

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
        az_halted = move_code & (MoveCode.AzPositive | MoveCode.AzNegative) == 0
        if az_halted:
            daz = angle_diff(self.tel_position.data.azimuthPosition,
                             self.tel_position.data.azimuthPositionSet)
            if abs(daz) < self.az_tolerance:
                mask |= Axis.Az

        main_halted = move_code & (MoveCode.MainDoorClosing | MoveCode.MainDoorOpening) == 0
        if main_halted and self.tel_position.data.mainDoorOpeningPercentage == \
                self.tel_position.data.mainDoorOpeningPercentageSet:
            mask |= Axis.MainDoor

        dropout_halted = move_code & (MoveCode.DropoutDoorClosing | MoveCode.DropoutDoorOpening) == 0
        if dropout_halted and self.tel_position.data.dropoutDoorOpeningPercentage == \
                self.tel_position.data.dropoutDoorOpeningPercentageSet:
            mask |= Axis.DropoutDoor

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
            The appropriate ``AzimuthState`` enum value.
        """
        if move_code & MoveCode.AzPositive:
            state = SALPY_ATDome.ATDome_shared_AzimuthState_MovingCWState
        elif move_code & MoveCode.AzNegative:
            state = SALPY_ATDome.ATDome_shared_AzimuthState_MovingCCWState
        else:
            state = SALPY_ATDome.ATDome_shared_AzimuthState_NotInMotionState
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
        closing_code = MoveCode.MainDoorClosing if is_main else MoveCode.DropoutDoorClosing
        opening_code = MoveCode.MainDoorOpening if is_main else MoveCode.DropoutDoorOpening
        door_mask = closing_code | opening_code
        door_state = None
        if move_code & door_mask == 0:
            if open_pct == 0:
                door_state = SALPY_ATDome.ATDome_shared_ShutterDoorState_ClosedState
            elif open_pct == 100:
                door_state = SALPY_ATDome.ATDome_shared_ShutterDoorState_OpenedState
            else:
                door_state = SALPY_ATDome.ATDome_shared_ShutterDoorState_PartiallyOpenedState
        elif move_code & closing_code:
            door_state = SALPY_ATDome.ATDome_shared_ShutterDoorState_ClosingState
        elif move_code & opening_code:
            door_state = SALPY_ATDome.ATDome_shared_ShutterDoorState_OpeningState
        if door_state is None:
            raise RuntimeError(f"Could not parse main door state from move_code={move_code}")
        return door_state

    def configure(self, host=_LOCAL_HOST, port=3210, connection_timeout=2, read_timeout=2):
        """Configure the CSC.

        Parameters
        ----------
        host : `str`
            Host IP address of ATDome TCP/IP controller.
            Ignored in simulation mode.
        port : `int`
            Port of ATDome TCP/IP controller.
        connection_timeout : `float`
            Time limit for TCP/IP connection (sec).
        read_timeout : `float`
            Time limit for TCP/IP read (sec).
        """
        assert read_timeout > 0
        assert connection_timeout > 0
        self.host = host
        self.port = port
        self.connection_timeout = connection_timeout
        self.read_timeout = read_timeout
        self.evt_settingsAppliedDomeTcp.set_put(
            host=host,
            port=port,
            connectionTimeout=connection_timeout,
            readTimeout=read_timeout,
        )

    async def connect(self):
        """Connect to the dome controller's TCP/IP port.
        """
        self.log.debug("connect")
        if self.connected:
            raise RuntimeError("Already connected")
        try:
            host = _LOCAL_HOST if self.simulation_mode == 1 else self.host
            coro = asyncio.open_connection(host=host, port=self.port)
            self.reader, self.writer = await asyncio.wait_for(coro, timeout=self.connection_timeout)
            self.log.debug("connected")
        except Exception as e:
            err_msg = f"Could not open connection to host={self.host}, port={self.port}"
            self.log.exception(err_msg)
            self.summary_state = salobj.State.FAULT
            self.evt_errorCode.set_put(errorCode=1, errorReport=f"{err_msg}: {e}", force_output=True)
            return

        asyncio.ensure_future(self.cmd_loop())
        self.status_loop()

    @property
    def connected(self):
        if None in (self.reader, self.writer):
            return False
        return True

    async def disconnect(self):
        """Disconnect from the dome controller's TCP/IP port.
        """
        self.log.debug("disconnect")
        writer = self.writer
        self.reader = None
        self.writer = None
        if writer:
            try:
                writer.write_eof()
                await asyncio.wait_for(writer.drain(), timeout=2)
            finally:
                writer.close()

    def handle_short_status(self, lines):
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
        status = ShortStatus(lines)

        self.tel_position.data.mainDoorOpeningPercentage = status.main_door_pct
        self.tel_position.data.dropoutDoorOpeningPercentage = status.dropout_door_pct
        settings_updated = self.evt_settingsAppliedDomeController.set(
            autoShutdownEnabled=status.auto_shutdown_enabled)

        self.tel_position.set_put(azimuthPosition=status.az_pos.deg)

        move_code = status.move_code
        self.evt_azimuthState.set_put(
            state=self.compute_az_state(move_code),
            homing=bool(move_code & MoveCode.Homing))

        dropout_door_state = self.compute_door_state(
            open_pct=self.tel_position.data.dropoutDoorOpeningPercentage,
            is_main=False,
            move_code=move_code)
        main_door_state = self.compute_door_state(
            open_pct=self.tel_position.data.mainDoorOpeningPercentage,
            is_main=True,
            move_code=move_code)
        self.evt_dropoutDoorState.set_put(state=dropout_door_state)
        self.evt_mainDoorState.set_put(state=main_door_state)

        self.evt_emergencyStop.set_put(active=move_code & MoveCode.EStop > 0)

        in_position_mask = self.compute_in_position_mask(move_code)

        def in_position(mask):
            return in_position_mask & mask == mask

        azimuth_in_position = in_position(Axis.Az)
        shutter_in_position = in_position(Axis.DropoutDoor | Axis.MainDoor)
        self.evt_azimuthInPosition.set_put(inPosition=azimuth_in_position)
        self.evt_shutterInPosition.set_put(inPosition=shutter_in_position)
        self.evt_allAxesInPosition.set_put(inPosition=azimuth_in_position and shutter_in_position)

        return settings_updated

    def handle_full_status(self, lines):
        """Handle output of "+" command.
        """
        status = RemainingStatus(lines)

        # The first five lines are identical to short status.
        # Unfortunately they include one item of data for the
        # settingsAppliedDomeController event: autoShutdownEnabled;
        # settings_updated is set True if that changes
        settings_updated = self.handle_short_status(lines[0:5])

        self.evt_emergencyStop.set_put(active=status.estop_active)
        self.evt_scbLink.set_put(active=status.scb_link_ok)

        self.evt_settingsAppliedDomeController.set_put(
            rainSensorEnabled=status.rain_sensor_enabled,
            cloudSensorEnabled=status.cloud_sensor_enabled,
            tolerance=status.tolerance.deg,
            homeAzimuth=status.home_azimuth.deg,
            highSpeedDistance=status.high_speed.deg,
            watchdogTimer=status.watchdog_timer,
            reversalDelay=status.reversal_delay,
            force_output=settings_updated,
        )

        self.is_first_status = False

    async def implement_simulation_mode(self, simulation_mode):
        if simulation_mode not in (0, 1):
            raise salobj.ExpectedError(
                f"Simulation_mode={simulation_mode} must be 0 or 1")

        if self.simulation_mode == simulation_mode:
            return

        await self.disconnect()
        await self.stop_mock_ctrl()
        if simulation_mode == 1:
            self.mock_ctrl = MockDomeController(port=self.port)
            await asyncio.wait_for(self.mock_ctrl.start(), timeout=2)
        if self.want_connection:
            await self.connect()

    def report_summary_state(self):
        super().report_summary_state()
        if self.connected != self.want_connection:
            if self.want_connection:
                asyncio.ensure_future(self.connect())
            else:
                asyncio.ensure_future(self.disconnect())

    def status_loop(self):
        """Read and report status from the TCP/IP controller.
        """
        if self.status_sleep_task and not self.status_sleep_task.done():
            self.status_sleep_task.cancel()
        if self.cmd_queue.qsize() < 2:
            asyncio.ensure_future(self._status_implementation())
        self.status_sleep_task = asyncio.ensure_future(asyncio.sleep(self.status_interval))

    async def _status_implementation(self):
        while self.connected:
            if self.n_short_status % self.short_per_full == 0:
                self.n_short_status = 0
                await self.cmd_queue.put("+")
            else:
                await self.cmd_queue.put("?")
            self.n_short_status += 1
            await asyncio.sleep(self.status_interval)

    async def stop(self, exception=None):
        """Disconnect from the TCP/IP controller and stop the CSC.
        """
        await self.disconnect()
        await self.stop_mock_ctrl()
        await super().stop(exception=exception)

    async def stop_mock_ctrl(self):
        """Stop the mock controller, if present.

        Safe to call even if there is no mock controller.
        """
        mock_ctrl = self.mock_ctrl
        self.mock_ctrl = None
        if mock_ctrl:
            await mock_ctrl.stop()

    @property
    def want_connection(self):
        return self.summary_state in (salobj.State.DISABLED, salobj.State.ENABLED)

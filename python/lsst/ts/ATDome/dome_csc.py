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

from astropy.coordinates import Angle
import astropy.units as u

from lsst.ts import salobj
from .utils import angle_diff
from .mock_controller import MockDomeController
from .status import ShortStatus, RemainingStatus

import SALPY_ATDome


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
    """AuxTel dome CSC simulator

    Parameters
    ----------
    sallib : ``module``
        salpy component library generatedby SAL
    port : `int`
        TCP/IP port for ATDome controller.
    index : `int` or `None`
        SAL component index, or 0 or None if the component is not indexed.
    initial_state : `salobj.State` or `int` (optional)
        The initial state of the CSC. This is provided for unit testing,
        as real CSCs should start up in `State.STANDBY`, the default.
    initial_simulation_mode : `int` (optional)
        Initial simulation mode.
        The only allowed value is 1: simulating.

    Raises
    ------
    salobj.ExpectedException
        If initial_state or initial_simulation_mode is invalid.
    """
    def __init__(self, index, port, initial_state=salobj.State.STANDBY, initial_simulation_mode=1):
        self.host = "127.0.0.1"
        self.port = port
        self.reader = None
        self.writer = None
        self.cmd_queue = asyncio.Queue()
        self.move_code = 0
        self.mock_ctrl = None  # mock controller, or None of not constructed
        self.in_position_mask = Axis(0)  # mask of Axis enums
        self.status_task = None
        self.status_interval = 0.2  # delay between short status commands (sec)
        self.n_short_status = 0
        self.short_per_full = 5  # number of short status between full status
        self.az_tolerance = Angle(0.2, u.deg)  # tolerance for "in position"
        super().__init__(SALPY_ATDome, index=index, initial_state=initial_state,
                         initial_simulation_mode=initial_simulation_mode)
        self.position_data = self.tel_position.DataType()
        self.az_state_data = self.evt_azimuthState.DataType()
        self.az_move_dir_data = self.evt_azimuthMovingDirection.DataType()
        self.main_door_state_data = self.evt_mainDoorState.DataType()
        self.dropout_door_state_data = self.evt_dropoutDoorState.DataType()
        self.estop_data = self.evt_emergencyStop.DataType()
        self.rain_detected = False
        self.clouds_detected = False
        self.scb_link_data = self.evt_scbLink.DataType()
        self.settings_tcp_data = self.evt_settingsAppliedDomeTcp.DataType()
        self.settings_ctrl_data = self.evt_settingsAppliedDomeController.DataType()
        self.is_first_status = True

    async def do_moveAzimuth(self, id_data):
        azimuth = id_data.data.azimuth
        if azimuth < 0 or azimuth > 360:
            raise salobj.ExpectedError(f"azimuth={azimuth} deg; must be in range [0, 360]")
        await self.cmd_queue.put(f"{azimuth:0.3f} MV")
        self.position_data.azimuthPositionSet = azimuth

    async def do_closeShutter(self, id_data):
        await self.cmd_queue.put("SC")
        self.position_data.mainDoorOpeningPercentageSet = 0
        self.position_data.dropoutOpeningPercentageSet = 0

    async def do_openShutter(self, id_data):
        await self.cmd_queue.put("SO")
        self.position_data.mainDoorOpeningPercentageSet = 100
        self.position_data.dropoutOpeningPercentageSet = 100

    async def do_stopMotionAllAxis(self, id_data):
        await self.cmd_queue.put("ST")

    async def do_moveShutterDropoutDoor(self, id_data):
        amount = id_data.data.dropoutDoorOpening
        if amount == 0:
            await self.cmd_queue.put("UP")
        elif amount == 100:
            await self.cmd_queue.put("DN")
        else:
            raise salobj.ExpectedException(f"dropoutDoorOpening={amount}; must be 0 or 100")
        self.position_data.dropoutOpeningPercentageSet = amount

    async def do_moveShutterMainDoor(self, id_data):
        amount = id_data.data.mainDoorOpening
        if amount == 0:
            await self.cmd_queue.put("CL")
        elif amount == 100:
            await self.cmd_queue.put("OP")
        else:
            raise salobj.ExpectedException(f"dropoutDoorOpening={amount}; must be 0 or 100")
        self.position_data.mainDoorOpeningPercentageSet = amount

    async def do_stopAzimuth(self, id_data):
        await self.cmd_queue.put("ST")

    async def do_stopShutter(self, id_data):
        await self.cmd_queue.put("ST")

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
                read_bytes = await asyncio.wait_for(self.reader.readuntil(">".encode()), timeout=2)
            except Exception as e:
                err_msg = "TCP/IP read failed"
                self.log.exception(err_msg)
                await self.disconnect()
                self.summary_state = salobj.State.FAULT
                error_code_data = self.evt_errorCode.DataType()
                error_code_data.errorCode = 1
                error_code_data.errorReport = f"{err_msg}: {e}"
                self.evt_errorCode.put(error_code_data)
                return

            data = read_bytes.decode()
            lines = data.split("\n")[:-1]  # strip final > line
            if len(lines) != expected_lines:
                self.log.error(f"Read {data} but expected {expected_lines} lines")
            if cmd == "?":
                if self.handle_short_status(lines):
                    self.evt_settingsAppliedDomeController.put(self.settings_ctrl_data)
            elif cmd == "+":
                self.handle_full_status(lines)

    def compute_in_position_mask(self, move_code):
        """Compute in_position_mask, but do not update self.in_position_mask.

        self.position_data must be current.

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
            daz = angle_diff(self.position_data.azimuthPosition, self.position_data.azimuthPositionSet)
            if abs(daz) < self.az_tolerance:
                mask |= Axis.Az

        main_halted = move_code & (MoveCode.MainDoorClosing | MoveCode.MainDoorOpening) == 0
        if main_halted and self.position_data.mainDoorOpeningPercentage == \
                self.position_data.mainDoorOpeningPercentageSet:
            mask |= Axis.MainDoor

        dropout_halted = move_code & (MoveCode.DropoutDoorClosing | MoveCode.DropoutDoorOpening) == 0
        if dropout_halted and self.position_data.dropoutOpeningPercentage == \
                self.position_data.dropoutOpeningPercentageSet:
            mask |= Axis.DropoutDoor

        return mask

    def compute_az_move_dir_status(self, move_code):
        """Compute data for the azimuthMovingDirection event.

        Parameters
        ----------
        move_code : `int`
            Motion code: the integer from line 5 of short status.
        """
        if move_code & MoveCode.AzPositive:
            status = SALPY_ATDome.ATDome_shared_MovingDirection_ClockWise
        elif move_code & MoveCode.AzNegative:
            status = SALPY_ATDome.ATDome_shared_MovingDirection_CounterClockWise
        else:
            status = SALPY_ATDome.ATDome_shared_MovingDirection_NotMoving
        return status

    def compute_az_state(self, move_code):
        """Compute azimuth state.

        This is alarmingly similar to azimuth move direction.

        Parameters
        ----------
        move_code : `int`
            Motion code: the integer from line 5 of short status.
        """
        if move_code & MoveCode.AzPositive:
            status = SALPY_ATDome.ATDome_shared_AzimuthState_MovingCWState
        elif move_code & MoveCode.AzNegative:
            status = SALPY_ATDome.ATDome_shared_AzimuthState_MovingCCWState
        else:
            status = SALPY_ATDome.ATDome_shared_AzimuthState_NotInMotionState
        return status

    def compute_door_state(self, open_pct, is_main, move_code):
        """Compute data for the mainDoorState or dropoutDoorState event.

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

    async def connect(self):
        """Connect to the dome controller's TCP/IP port.
        """
        self.log.debug("connect")
        if self.connected:
            raise RuntimeError("Already connected")
        try:
            host = "127.0.0.1" if self.simulation_mode == 1 else self.host
            coro = asyncio.open_connection(host=host, port=self.port)
            self.reader, self.writer = await asyncio.wait_for(coro, timeout=5)
            self.log.debug("connected")
        except Exception as e:
            err_msg = f"Could not open connection to host={self.host}, port={self.port}"
            self.log.exception(err_msg)
            self.summary_state = salobj.State.FAULT
            error_code_data = self.evt_errorCode.DataType()
            error_code_data.errorCode = 1
            error_code_data.errorReport = f"{err_msg}: {e}"
            self.evt_errorCode.put(error_code_data)
            return

        asyncio.ensure_future(self.cmd_loop())
        asyncio.ensure_future(self.status_loop())

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
        settings_ctrl_data_updated : `bool`
            True if ``self.settings_ctrl_data updated``.
        """
        status = ShortStatus(lines)

        self.position_data.mainDoorOpeningPercentage = status.main_door_pct
        self.position_data.dropoutOpeningPercentage = status.dropout_door_pct
        old_shutdown_enabled = self.settings_ctrl_data.autoShutdownActivated
        self.set_field(self.settings_ctrl_data, "autoShutdownActivated", status.auto_shutdown_enabled,
                       force=self.is_first_status)
        settings_updated = self.settings_ctrl_data.autoShutdownActivated != old_shutdown_enabled

        # There is no event for rain or clouds detected (yet),
        # so output a log message. Note that both states (detected
        # and not detected) must be output at the same level,
        # so that both states are seen or not seen, depending on log level.
        rain_detected = status.sensor_code & 1 != 0
        if rain_detected != self.rain_detected or self.is_first_status:
            self.rain_detected = rain_detected
            self.log.warning(f"rain_detected={rain_detected}")
        clouds_detected = status.sensor_code & 2 != 0
        if clouds_detected != self.clouds_detected or self.is_first_status:
            self.clouds_detected = clouds_detected
            self.log.warning(f"clouds_detected={clouds_detected}")
        self.position_data.azimuthPosition = status.az_pos.deg
        self.tel_position.put(self.position_data)

        move_code = status.move_code
        az_state = self.compute_az_state(move_code)
        if self.set_field(self.az_state_data, "state", az_state, force=self.is_first_status):
            self.evt_azimuthState.put(self.az_state_data)
        az_move_dir_status = self.compute_az_move_dir_status(move_code)
        if self.set_field(self.az_move_dir_data, "motionStatus", az_move_dir_status,
                          force=self.is_first_status):
            self.evt_azimuthMovingDirection.put(self.az_move_dir_data)

        main_door_state = self.compute_door_state(
            open_pct=self.position_data.mainDoorOpeningPercentage,
            is_main=True,
            move_code=move_code)
        if self.set_field(self.main_door_state_data, "state", main_door_state, force=self.is_first_status):
            self.evt_mainDoorState.put(self.main_door_state_data)

        dropout_door_state = self.compute_door_state(
            open_pct=self.position_data.dropoutOpeningPercentage,
            is_main=False,
            move_code=move_code)
        if self.set_field(self.dropout_door_state_data, "state", dropout_door_state,
                          force=self.is_first_status):
            self.evt_dropoutDoorState.put(self.dropout_door_state_data)

        estop_active = move_code & MoveCode.EStop > 0
        if self.set_field(self.estop_data, "active", estop_active, force=self.is_first_status):
            self.evt_emergencyStop.put(self.estop_data)

        self.report_in_position_events(old_in_position_mask=self.in_position_mask,
                                       move_code=move_code, force=self.is_first_status)
        return settings_updated

    def handle_full_status(self, lines):
        """Handle output of "+" command.
        """
        status = RemainingStatus(lines)

        # The first five lines are identical to short status.
        # Unfortunately they include one item of data for the
        # settingsAppliedDomeController event
        settings_updated = self.handle_short_status(lines[0:5])

        if self.set_field(self.estop_data, "active", status.estop_active, force=self.is_first_status):
            self.evt_emergencyStop.put(self.estop_data)

        if self.set_field(self.scb_link_data, "active", status.scb_link_ok, force=self.is_first_status):
            self.evt_scbLink.put(self.scb_link_data)

        settings_updated |= self.set_field(self.settings_ctrl_data, "rainSensorActivated",
                                           status.rain_sensor_enabled, force=self.is_first_status)
        settings_updated |= self.set_field(self.settings_ctrl_data, "cloudSensorActivated",
                                           status.cloud_sensor_enabled, force=self.is_first_status)
        settings_updated |= self.set_field(self.settings_ctrl_data, "tolerance",
                                           status.tolerance.deg, force=self.is_first_status)
        settings_updated |= self.set_field(self.settings_ctrl_data, "highSpeedDistance",
                                           status.high_speed.deg, force=self.is_first_status)
        # the learnManual field cannot be set from full status

        settings_updated |= self.set_field(self.settings_ctrl_data, "watchdogTimer",
                                           status.watchdog_timer, force=self.is_first_status)
        settings_updated |= self.set_field(self.settings_ctrl_data, "reversalDelay",
                                           status.reversal_delay, force=self.is_first_status)

        # the autoShutdownActivated field is set by handle_short_status

        if settings_updated:
            self.evt_settingsAppliedDomeController.put(self.settings_ctrl_data)
        self.is_first_status = False

    async def implement_simulation_mode(self, simulation_mode):
        if simulation_mode != 1:
            raise salobj.ExpectedError(
                f"This CSC only supports simulation; simulation_mode={simulation_mode} but must be 1")

        if self.simulation_mode == simulation_mode:
            return

        await self.disconnect()
        if simulation_mode == 1:
            await self.stop_mock_ctrl()
            self.mock_ctrl = MockDomeController(port=self.port)
            await asyncio.wait_for(self.mock_ctrl.start(), timeout=2)
        else:
            self.mock_ctrl = None
        if self.want_connection:
            await self.connect()

    def report_in_position_events(self, old_in_position_mask, move_code, force):
        """Update ``self.in_position_mask`` and report inPosition events.

        self.position_data must be current.

        Parameters
        ----------
        old_in_position_mask : `int`
            Old value of ``self.in_position_mask``
        move_code : `int`
            Motion code: the integer from line 5 of short status.
        force : `bool`
            If True then set the field and output the event
            regardless of its current value.
        """
        self.in_position_mask = self.compute_in_position_mask(move_code)
        if old_in_position_mask != self.in_position_mask:
            old_az_in_position = old_in_position_mask & Axis.Az == Axis.Az
            az_in_position = self.in_position_mask & Axis.Az == Axis.Az
            if old_az_in_position != az_in_position or force:
                az_in_position_data = self.evt_azimuthInPosition.DataType()
                az_in_position_data.inPosition = az_in_position
                self.evt_azimuthInPosition.put(az_in_position_data)
            doors_mask = Axis.MainDoor | Axis.DropoutDoor
            old_doors_in_position = old_in_position_mask & doors_mask == doors_mask
            doors_in_position = self.in_position_mask & doors_mask == doors_mask
            if old_doors_in_position != doors_in_position or force:
                shutter_in_position_data = self.evt_shutterInPosition.DataType()
                shutter_in_position_data.inPosition = doors_in_position
                self.evt_shutterInPosition.put(shutter_in_position_data)
            all_axes_mask = Axis.Az | Axis.MainDoor | Axis.DropoutDoor
            old_all_axes_in_position = old_in_position_mask & all_axes_mask == all_axes_mask
            all_axes_in_position = self.in_position_mask & all_axes_mask == all_axes_mask
            if old_all_axes_in_position != all_axes_in_position or force:
                all_axes_in_position_data = self.evt_allAxisInPosition.DataType()
                all_axes_in_position_data.inPosition = all_axes_in_position
                self.evt_allAxisInPosition.put(all_axes_in_position_data)

    def report_summary_state(self):
        super().report_summary_state()
        if self.connected != self.want_connection:
            if self.want_connection:
                asyncio.ensure_future(self.connect())
            else:
                asyncio.ensure_future(self.disconnect())

    def set_field(self, topic, field_name, value, force):
        """Set a field of a topic, if its value has changed.

        Parameters
        ----------
        topic : ``struct``
            SALPY event or telemetry topic.
        field_name : `str`
            Name of field to set
        value : ``any``
            New value for field
        force : `bool`
            If True then set the field regardless of its current value.

        Returns
        -------
        was_set : `bool`
            True if the field was set, i.e. if ``force`` is true
            or the field value has changed.
        """
        if force or getattr(topic, field_name) != value:
            setattr(topic, field_name, value)
            return True
        return False

    async def status_loop(self):
        while self.connected:
            if self.cmd_queue.qsize() < 2:
                # avoid flooding
                if self.n_short_status % self.short_per_full == 0:
                    self.n_short_status = 0
                    await self.cmd_queue.put("+")
                else:
                    await self.cmd_queue.put("?")
                self.n_short_status += 1
            await asyncio.sleep(self.status_interval)

    async def stop(self):
        await self.disconnect()
        await self.stop_mock_ctrl()
        await super().stop()

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

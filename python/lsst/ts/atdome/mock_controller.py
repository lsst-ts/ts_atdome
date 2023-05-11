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

__all__ = ["INITIAL_AZIMUTH", "MockDomeController"]

import asyncio
import enum
import functools
import logging

from lsst.ts import simactuators, tcpip, utils

from .enums import MoveCode

logging.basicConfig()

# Initial azimuth (deg)
INITIAL_AZIMUTH = 285


class Door(enum.Flag):
    """Shutter door identifiers."""

    Main = enum.auto()
    Dropout = enum.auto()


class MockDomeController(tcpip.OneClientReadLoopServer):
    """Mock DomeController that talks over TCP/IP.

    Parameters
    ----------
    port : int
        TCP/IP port. If 0 then pick an available port.
    door_time : `float`
        Time to open or close either door (sec)
    az_vel : `float`
        Azimuth velocity (deg/sec)
    home_az : `float`
        Azimuth home position (deg)
    home_az_overshoot : `float`
        Distance to move CCW past the home switch at full speed while homing,
        before coming back slowly (deg)
    home_az_vel : `float`
        Final velocity for azimuth homing,
        once the home switch has been contacted at full speed (deg/sec)

    Notes
    -----
    To start the server:

        ctrl = MockDomeController(...)
        await ctrl.start_task

    To stop the server:

        await ctrl.close()

    Known Limitations:

    * The following commands are not supported:
      AO, AF, RO, RF, CO, CF, TOL, HZ, HS, CS, LM, WT, RD, CFS, CFR, HELP
    * Encoder counts are not supported; reported values are bogus.
    * Door sequencing is not supported; doors move as if independent.
    """

    def __init__(
        self,
        port,
        log,
        door_time=1,
        az_vel=6,
        home_az=10,
        home_az_overshoot=1,
        home_az_vel=1,
    ):
        self.door_time = door_time
        self.az_vel = az_vel
        self.high_speed = 5
        self.coast = 0.5
        self.tolerance = 1.0
        self.home_az = home_az
        self.home_az_overshoot = home_az_overshoot
        self.home_az_vel = home_az_vel
        # Maximum az - home_az (deg) for which to report
        # being on the home switch
        self.home_az_tolerance = 0.2
        self.encoder_counts_per_360 = 4018143232
        self.az_actuator = simactuators.CircularPointToPointActuator(
            speed=az_vel, start_position=INITIAL_AZIMUTH
        )
        self.az_move_timeout = 120
        self.homed = False
        self.watchdog_reset_time = 600
        self.dropout_timer = 5
        self.reverse_delay = 4
        self.main_door_encoder_closed = 118449181478
        self.main_door_encoder_opened = 8287616388
        self.dropout_door_encoder_closed = 5669776578
        self.dropout_door_encoder_opened = 5710996184
        self.door_move_timeout = 360
        self.door_actuators = {
            enum: simactuators.PointToPointActuator(
                min_position=0,
                max_position=100,
                start_position=0,
                speed=100 / door_time,
            )
            for enum in Door
        }

        self._homing_task = utils.make_done_future()
        self.rain_sensor_enabled = True
        self.rain_detected = False
        self.cloud_sensor_enabled = True
        self.clouds_detected = False
        self.scb_link_ok = True
        self.auto_shutdown_enabled = False
        self.estop_active = False
        # Name of a command to report as failed once, the next time it is seen,
        # or None if no failures. Used to test CSC handling of failed commands.
        self.fail_command = None

        self.last_rot_right = None

        # Dict of command: (has_argument, function).
        # The function is called with:
        # * No arguments, if `has_argument` False.
        # * The argument as a string, if `has_argument` is True.
        self.dispatch_dict = {
            "?": (False, self.do_short_status),
            "+": (False, self.do_full_status),
            "ST": (False, self.do_stop),
            "CL": (False, functools.partial(self.do_close_doors, Door.Main)),
            "OP": (False, functools.partial(self.do_open_doors, Door.Main)),
            "UP": (False, functools.partial(self.do_close_doors, Door.Dropout)),
            "DN": (False, functools.partial(self.do_open_doors, Door.Dropout)),
            "SC": (
                False,
                functools.partial(self.do_close_doors, Door.Main | Door.Dropout),
            ),
            "SO": (
                False,
                functools.partial(self.do_open_doors, Door.Main | Door.Dropout),
            ),
            "HM": (False, self.do_home),
            "MV": (True, self.do_set_cmd_az),
        }
        super().__init__(port=port, log=log, terminator=b"\n")

    async def read_loop(self):
        print(f"read_loop begins; {self.connected=}; {self.port=}")
        try:
            await self.write_str("ACE Main Box")
            await self.write_prompt()
            await super().read_loop()
        except Exception as e:
            print(f"read loop failed: {e!r}")

    async def write_prompt(self):
        """Write an unterminated ">" prompt."""
        await self.write(">".encode("utf-8"))

    @property
    def homing(self):
        """Is azimuth being homed?"""
        return not self._homing_task.done()

    async def read_and_dispatch(self):
        """Read, parse and execute one command and output replies."""
        data = await self.read_str()
        self.log.debug(f"read command: {data!r}")
        if not data:
            return
        try:
            items = data.split()
            cmd = items[-1]
            if cmd not in self.dispatch_dict:
                raise KeyError(f"Unsupported command {cmd}")
            if cmd == self.fail_command:
                self.fail_command = None
                outputs = [f"Command {cmd} failed by request"]
            else:
                has_data, func = self.dispatch_dict[cmd]
                desired_len = 2 if has_data else 1
                if len(items) != desired_len:
                    raise RuntimeError(
                        f"{data} split into {len(items)} pieces; expected {desired_len}"
                    )
                if has_data:
                    outputs = func(items[0])
                else:
                    outputs = func()
            if outputs:
                for msg in outputs:
                    await self.write_str(msg)
        except Exception as e:
            self.log.exception(f"command {data} failed: {e!r}")
        await self.write_prompt()

    def do_close_doors(self, doors):
        """Close the specified doors.

        Parameters
        ----------
        doors : `Door`
            Bit mask specifying the doors
        """
        self.log.debug(f"do_close_doors({doors})")
        for door in self.door_iter(doors):
            self.door_actuators[door].set_position(0)

    def do_open_doors(self, doors):
        """Open the specified doors.

        Parameters
        ----------
        doors : `Door`
            Bit mask specifying the doors
        """
        self.log.debug(f"do_open_doors({doors})")
        for door in self.door_iter(doors):
            self.door_actuators[door].set_position(100)

    def door_iter(self, doors):
        """Convert a doors bitmask to a list of Doors"""
        for door in Door:
            if door & doors:
                yield door

    def do_home(self):
        """Rotate azimuth to the home position."""
        self._homing_task.cancel()
        self._homing_task = asyncio.ensure_future(self.implement_home())

    def do_set_cmd_az(self, data):
        """Set commanded azimuth position.

        Parameters
        ----------
        data : `str`
            Desired azimuth as a float encoded as a string (deg)
        """
        if self.homing:
            raise RuntimeError("Cannot set azimuth while homing")
        cmd_az = float(data)
        self.set_cmd_az(cmd_az)

    def do_short_status(self):
        """Create short status as a list of strings."""
        curr_tai = utils.current_tai()
        move_code = MoveCode(0)
        outputs = []
        for door, name, closing_code, opening_code in (
            (
                Door.Main,
                "MAIN",
                MoveCode.MAIN_DOOR_CLOSING,
                MoveCode.MAIN_DOOR_OPENING,
            ),
            (
                Door.Dropout,
                "DROP",
                MoveCode.DROPOUT_DOOR_CLOSING,
                MoveCode.DROPOUT_DOOR_OPENING,
            ),
        ):
            state_str = "AJAR"
            actuator = self.door_actuators[door]
            current_position = actuator.position(curr_tai)
            current_velocity = actuator.velocity(curr_tai)
            if current_velocity < 0:
                move_code |= closing_code
            elif current_velocity > 0:
                move_code |= opening_code
            else:
                if current_position == 0:
                    state_str = "CLOSED"
                elif current_position == 100:
                    state_str = "OPEN"
            outputs.append(f"{name} {state_str} {current_position:03.0f}")
        enabled_str = "ON" if self.auto_shutdown_enabled else "OFF"
        sensor_mask = 0
        if self.rain_detected:
            sensor_mask |= 1
        if self.clouds_detected:
            sensor_mask |= 2
        outputs.append(f"[{enabled_str}] {sensor_mask:02d}")

        az_moving = self.az_actuator.moving(curr_tai)
        curr_az = self.az_actuator.position(curr_tai)
        if abs(curr_az - self.home_az) < self.home_az_tolerance:
            az_str = "HOME"
        else:
            az_str = "POSN"
        outputs.append(f"{az_str} {curr_az:0.2f}")
        if self.last_rot_right is None:
            dir_code = "--"
        elif self.last_rot_right:
            dir_code = "RR"
        else:
            dir_code = "RL"
        if az_moving:
            if self.last_rot_right:
                move_code |= MoveCode.AZIMUTH_POSITIVE
            else:
                move_code |= MoveCode.AZIMUTH_NEGATIVE
        if self.homing:
            move_code |= MoveCode.AZIMUTH_HOMING

        if self.estop_active:
            move_code |= MoveCode.ESTOP
        outputs.append(f"{dir_code} {move_code.value:03d}")
        outputs.append("Dome homed" if self.homed else "Dome not homed")
        return outputs

    def do_full_status(self):
        """Create full status as a list of strings."""
        az_encoder_counts = int(self.home_az * self.encoder_counts_per_360 / 360)
        outputs = self.do_short_status()
        outputs.append(f"Emergency Stop Active: {1 if self.estop_active else 0}")
        outputs.append(f"Top Comm Link OK:    {1 if self.scb_link_ok else 0}")
        outputs.append(f"Home Azimuth: {self.home_az:5.2f}")
        outputs.append(f"High Speed (degrees): {self.high_speed:5.2f}")
        outputs.append(f"Coast (degrees): {self.coast:0.2f}")
        outputs.append(f"Tolerance (degrees): {self.tolerance:0.2f}")
        outputs.append(f"Encoder Counts per 360: {self.encoder_counts_per_360:d}")
        outputs.append(f"Encoder Counts:  {az_encoder_counts:d}")
        outputs.append(f"Last Azimuth GoTo: {self.az_actuator.end_position:05.2f}")
        outputs.append(f"Azimuth Move Timeout (secs): {self.az_move_timeout}")
        outputs.append(f"Rain-Snow enabled:  {1 if self.rain_sensor_enabled else 0}")
        outputs.append(f"Cloud Sensor enabled: {1 if self.cloud_sensor_enabled else 0}")
        outputs.append(f"Watchdog Reset Time: {self.watchdog_reset_time}")
        outputs.append(f"Dropout Timer: {self.dropout_timer}")
        outputs.append(f"Reverse Delay: {self.reverse_delay}")
        outputs.append(f"Main Door Encoder Closed: {self.main_door_encoder_closed:d}")
        outputs.append(f"Main Door Encoder Opened: {self.main_door_encoder_opened:d}")
        outputs.append(f"Dropout Encoder Closed: {self.dropout_door_encoder_closed:d}")
        outputs.append(f"Dropout Encoder Opened: {self.dropout_door_encoder_opened:d}")
        outputs.append(f"Door Move Timeout (secs): {self.door_move_timeout}")
        az_homed_str = "True" if self.homed else "False"
        outputs.append(f"Dome has been homed: {az_homed_str}")
        return outputs

    def do_stop(self):
        """Stop all motion."""
        self.az_actuator.stop()
        for actuator in self.door_actuators.values():
            actuator.stop()

    def set_cmd_az(self, cmd_az):
        """Set cmd_az and last_rot_right.

        Parameters
        ----------
        cmd_az : `float`
            Desired azimuth (degree). Must be in the range [0, 360).

        Returns
        -------
        duration : `float`
            Move duration (second)
        """
        if not 0 <= cmd_az < 360:
            raise ValueError(f"cmd_az={cmd_az} not in range [0, 360)")
        duration = self.az_actuator.set_position(
            position=cmd_az, direction=simactuators.Direction.NEAREST
        )
        self.last_rot_right = True if self.az_actuator.direction == 1 else False
        return duration

    async def implement_home(self):
        """Home the azimuth axis."""
        try:
            duration = self.set_cmd_az(self.home_az - self.home_az_overshoot)
            await asyncio.sleep(duration)
            self.az_actuator.speed = self.home_az_vel
            duration = self.set_cmd_az(self.home_az)
            await asyncio.sleep(duration)
            self.homed = True
        finally:
            self.az_actuator.speed = self.az_vel

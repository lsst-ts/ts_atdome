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
import unittest

import pytest

from lsst.ts import utils
from lsst.ts import atdome


class MockTestCase(unittest.IsolatedAsyncioTestCase):
    """Test MockDomeController, Status and LongStatus"""

    async def asyncSetUp(self):
        self.ctrl = None
        self.writer = None

        self.ctrl = atdome.MockDomeController(port=0)
        await asyncio.wait_for(self.ctrl.start(), 5)
        rw_coro = asyncio.open_connection(host="127.0.0.1", port=self.ctrl.port)
        self.reader, self.writer = await asyncio.wait_for(rw_coro, timeout=5)
        read_bytes = await asyncio.wait_for(
            self.reader.readuntil(">".encode()), timeout=5
        )
        read_str = read_bytes.decode().strip()
        assert read_str == "ACE Main Box\n>"

    async def asyncTearDown(self):
        if self.ctrl:
            await asyncio.wait_for(self.ctrl.stop(), 5)
        if self.writer:
            self.writer.close()

    async def send_cmd(self, cmd, timeout=2):
        """Send a command to the mock controller and wait for the reply.

        Return the decoded reply as 0 or more lines of text
        with the final ">" stripped.
        """
        self.writer.write(f"{cmd}\n".encode())
        await self.writer.drain()
        read_bytes = await asyncio.wait_for(
            self.reader.readuntil(">".encode()), timeout=timeout
        )
        # [0:-1] strips the final ">"
        read_str = read_bytes.decode()[0:-1].strip()
        return read_str.split("\n")

    async def test_initial_full_status(self):
        reply_lines = await self.send_cmd("+")

        status = atdome.Status(reply_lines)
        assert status.main_door_pct == 0
        assert status.dropout_door_pct == 0
        assert status.auto_shutdown_enabled == self.ctrl.auto_shutdown_enabled
        assert status.sensor_code == 0
        assert status.az_pos == pytest.approx(atdome.INITIAL_AZIMUTH)
        assert status.move_code == 0
        assert not status.homed
        assert status.estop_active == self.ctrl.estop_active
        assert status.scb_link_ok == self.ctrl.scb_link_ok
        assert status.rain_sensor_enabled == self.ctrl.rain_sensor_enabled
        assert status.cloud_sensor_enabled == self.ctrl.cloud_sensor_enabled
        assert status.coast == pytest.approx(self.ctrl.coast)
        assert status.tolerance == pytest.approx(self.ctrl.tolerance)
        assert status.home_azimuth == pytest.approx(self.ctrl.home_az)
        assert status.high_speed == pytest.approx(self.ctrl.high_speed)
        assert status.watchdog_timer == pytest.approx(self.ctrl.watchdog_reset_time)
        assert status.reversal_delay == pytest.approx(self.ctrl.reverse_delay)
        assert status.encoder_counts_per_360 == self.ctrl.encoder_counts_per_360
        assert status.main_door_encoder_closed == self.ctrl.main_door_encoder_closed
        assert status.main_door_encoder_opened == self.ctrl.main_door_encoder_opened
        assert (
            status.dropout_door_encoder_closed == self.ctrl.dropout_door_encoder_closed
        )
        assert (
            status.dropout_door_encoder_opened == self.ctrl.dropout_door_encoder_opened
        )
        assert status.door_move_timeout == pytest.approx(self.ctrl.door_move_timeout)

    async def test_fail_cmd(self):
        self.ctrl.fail_command = "+"
        reply_lines = await self.send_cmd("+")
        assert len(reply_lines) == 1
        assert "failed" in reply_lines[0]
        # Make sure the command is only failed once.
        reply_lines = await self.send_cmd("+")
        assert len(reply_lines) == 27

    async def test_move_az(self):
        # Test initial conditions; some details of this test
        # may have to be changed if this value changes.
        assert atdome.INITIAL_AZIMUTH == pytest.approx(285)

        daz = -3
        az = atdome.INITIAL_AZIMUTH + daz
        est_duration = abs(daz / self.ctrl.az_vel)
        reply_lines = await self.send_cmd(f"{az:0.2f} MV")
        assert reply_lines == [""]
        await asyncio.sleep(est_duration / 2)
        reply_lines = await self.send_cmd("+")
        status = atdome.Status(reply_lines)
        assert status.az_pos < atdome.INITIAL_AZIMUTH
        assert status.az_pos > atdome.INITIAL_AZIMUTH + daz
        assert status.move_code == 2

        # wait long enough for the move to finish and check status
        await asyncio.sleep(est_duration / 2 + 0.5)
        reply_lines = await self.send_cmd("+")
        status = atdome.Status(reply_lines)
        assert status.az_pos == pytest.approx(az)
        assert status.move_code == 0

    async def test_home_az(self):
        assert not self.ctrl.homed
        daz = -2
        est_ccw_duration = abs(daz / self.ctrl.az_vel)
        curr_az = self.ctrl.az_actuator.position(utils.current_tai())
        home_azimuth = utils.angle_wrap_nonnegative(curr_az + daz).deg
        self.ctrl.home_az = home_azimuth

        reply_lines = await self.send_cmd("HM")
        assert reply_lines == [""]

        # sleep until halfway through CCW motion and check status
        await asyncio.sleep(est_ccw_duration / 2)
        reply_lines = await self.send_cmd("+")
        status = atdome.Status(reply_lines)
        assert status.move_code == 2 + 64
        assert self.ctrl.az_actuator.speed == pytest.approx(self.ctrl.az_vel)
        assert not self.ctrl.homed
        assert not status.homed

        # sleep until halfway through CW motion and check status
        await asyncio.sleep(
            self.ctrl.az_actuator.remaining_time() + est_ccw_duration / 2
        )
        reply_lines = await self.send_cmd("+")
        status = atdome.Status(reply_lines)
        assert status.move_code == 1 + 64
        assert self.ctrl.az_actuator.speed == pytest.approx(self.ctrl.home_az_vel)
        assert not self.ctrl.homed
        assert not status.homed

        # sleep until we're done and check status
        await asyncio.sleep(self.ctrl.az_actuator.remaining_time() + 0.1)
        reply_lines = await self.send_cmd("+")
        status = atdome.Status(reply_lines)
        assert status.move_code == 0
        assert self.ctrl.az_actuator.speed == pytest.approx(self.ctrl.az_vel)
        assert self.ctrl.az_actuator.position(utils.current_tai()) == pytest.approx(
            self.ctrl.home_az
        )
        assert self.ctrl.homed
        assert status.homed

    async def test_az_home_switch(self):
        daz = -2
        curr_az = self.ctrl.az_actuator.position(utils.current_tai())
        self.ctrl.home_az = utils.angle_wrap_nonnegative(curr_az + daz).deg

        # Move to left edge, center, and right edge of home switch
        # and assert on the home switch each time.
        for az in (
            self.ctrl.home_az - self.ctrl.home_az_tolerance + 0.01,
            self.ctrl.home_az + self.ctrl.home_az_tolerance - 0.01,
            self.ctrl.home_az,
        ):
            reply_lines = await self.send_cmd(f"{az:0.2f} MV")
            assert reply_lines == [""]

            # Sleep until motion finished, then check status.
            # Wait a bit extra for Docker macOS clock non-monotonicity.
            await asyncio.sleep(self.ctrl.az_actuator.remaining_time() + 0.2)
            reply_lines = await self.send_cmd("+")
            status = atdome.Status(reply_lines)
            assert status.move_code == 0
            assert status.az_pos == pytest.approx(az)
            assert status.az_home_switch

        # Move just off of the az switch in both directions
        for az in (
            self.ctrl.home_az - self.ctrl.home_az_tolerance - 0.01,
            self.ctrl.home_az + self.ctrl.home_az_tolerance + 0.01,
        ):
            reply_lines = await self.send_cmd(f"{az:0.2f} MV")
            assert reply_lines == [""]

            # Sleep until motion finished, then check status.
            # Wait a bit extra for Docker macOS clock non-monotonicity.
            await asyncio.sleep(self.ctrl.az_actuator.remaining_time() + 0.2)
            reply_lines = await self.send_cmd("+")
            status = atdome.Status(reply_lines)
            assert status.move_code == 0
            assert status.az_pos == pytest.approx(az)
            assert not status.az_home_switch

    async def test_main_door(self):
        est_duration = self.ctrl.door_time

        # open main door
        reply_lines = await self.send_cmd("OP")
        assert reply_lines == [""]
        await asyncio.sleep(est_duration / 2)
        reply_lines = await self.send_cmd("+")
        status = atdome.Status(reply_lines)
        assert status.main_door_pct < 100
        assert status.main_door_pct > 0
        assert status.move_code == 8

        # wait long enough for the move to finish and check status
        await asyncio.sleep(est_duration / 2 + 0.5)
        reply_lines = await self.send_cmd("+")
        status = atdome.Status(reply_lines)
        assert status.main_door_pct == pytest.approx(100)
        assert status.move_code == 0

        # close main door
        reply_lines = await self.send_cmd("CL")
        assert reply_lines == [""]
        await asyncio.sleep(est_duration / 2)
        reply_lines = await self.send_cmd("+")
        status = atdome.Status(reply_lines)
        assert status.main_door_pct < 100
        assert status.main_door_pct > 0
        assert status.move_code == 4

        # wait long enough for the move to finish and check status
        await asyncio.sleep(est_duration / 2 + 0.5)
        reply_lines = await self.send_cmd("+")
        status = atdome.Status(reply_lines)
        assert status.main_door_pct == pytest.approx(0)
        assert status.move_code == 0

    async def test_dropout_door(self):
        est_duration = self.ctrl.door_time

        # open dropout door
        reply_lines = await self.send_cmd("DN")
        assert reply_lines == [""]
        await asyncio.sleep(est_duration / 2)
        reply_lines = await self.send_cmd("+")
        status = atdome.Status(reply_lines)
        assert status.dropout_door_pct < 100
        assert status.dropout_door_pct > 0
        assert status.move_code == 32

        # wait long enough for the move to finish and check status
        await asyncio.sleep(est_duration / 2 + 0.5)
        reply_lines = await self.send_cmd("+")
        status = atdome.Status(reply_lines)
        assert status.dropout_door_pct == pytest.approx(100)
        assert status.move_code == 0

        # close dropout door
        reply_lines = await self.send_cmd("UP")
        assert reply_lines == [""]
        await asyncio.sleep(est_duration / 2)
        reply_lines = await self.send_cmd("+")
        status = atdome.Status(reply_lines)
        assert status.dropout_door_pct < 100
        assert status.dropout_door_pct > 0
        assert status.move_code == 16

        # wait long enough for the move to finish and check status
        await asyncio.sleep(est_duration / 2 + 0.5)
        reply_lines = await self.send_cmd("+")
        status = atdome.Status(reply_lines)
        assert status.dropout_door_pct == pytest.approx(0)
        assert status.move_code == 0

    async def test_simultaneous_door_motion(self):
        est_duration = self.ctrl.door_time

        # open both doors
        reply_lines = await self.send_cmd("SO")
        assert reply_lines == [""]
        await asyncio.sleep(est_duration / 2)
        reply_lines = await self.send_cmd("+")
        status = atdome.Status(reply_lines)
        assert status.main_door_pct < 100
        assert status.main_door_pct > 0
        assert status.dropout_door_pct < 100
        assert status.dropout_door_pct > 0
        assert status.move_code == 8 + 32

        # wait long enough for the move to finish and check status
        await asyncio.sleep(est_duration / 2 + 0.5)
        reply_lines = await self.send_cmd("+")
        status = atdome.Status(reply_lines)
        assert status.main_door_pct == pytest.approx(100)
        assert status.dropout_door_pct == pytest.approx(100)
        assert status.move_code == 0

        # close dropout door
        reply_lines = await self.send_cmd("SC")
        assert reply_lines == [""]
        await asyncio.sleep(est_duration / 2)
        reply_lines = await self.send_cmd("+")
        status = atdome.Status(reply_lines)
        assert status.main_door_pct < 100
        assert status.main_door_pct > 0
        assert status.dropout_door_pct < 100
        assert status.dropout_door_pct > 0
        assert status.move_code == 4 + 16

        # wait long enough for the move to finish and check status
        await asyncio.sleep(est_duration / 2 + 0.5)
        reply_lines = await self.send_cmd("+")
        status = atdome.Status(reply_lines)
        assert status.main_door_pct == pytest.approx(0)
        assert status.dropout_door_pct == pytest.approx(0)
        assert status.move_code == 0

    async def test_stop(self):
        est_duration = self.ctrl.door_time
        daz = self.ctrl.az_vel * est_duration
        az = atdome.INITIAL_AZIMUTH + daz

        # start azimuth motion
        reply_lines = await self.send_cmd(f"{az:0.2f} MV")
        # open both doors
        reply_lines = await self.send_cmd("SO")
        assert reply_lines == [""]

        # wait for the moves to get about halfway and stop
        await asyncio.sleep(est_duration / 2)
        reply_lines = await self.send_cmd("ST")
        reply_lines = await self.send_cmd("+")
        status = atdome.Status(reply_lines)
        assert status.az_pos < az
        assert status.az_pos > 0
        assert status.main_door_pct < 100
        assert status.main_door_pct > 0
        assert status.dropout_door_pct < 100
        assert status.dropout_door_pct > 0
        assert status.move_code == 0

    async def test_long_status(self):
        self.ctrl.rain_enabled = False

        reply_lines = await self.send_cmd("+")
        status = atdome.Status(reply_lines)
        assert status.estop_active == self.ctrl.estop_active
        assert status.scb_link_ok == status.scb_link_ok
        assert status.rain_sensor_enabled == self.ctrl.rain_sensor_enabled
        assert status.cloud_sensor_enabled == self.ctrl.cloud_sensor_enabled

        # Toggle several values, one at a time, and check that the output
        # is updated as expected
        for name in (
            "estop_active",
            "scb_link_ok",
            "rain_sensor_enabled",
            "cloud_sensor_enabled",
        ):
            setattr(self.ctrl, name, not getattr(self.ctrl, name))

            reply_lines = await self.send_cmd("+")
            status = atdome.Status(reply_lines)
            assert status.estop_active == self.ctrl.estop_active
            assert status.scb_link_ok == status.scb_link_ok
            assert status.rain_sensor_enabled == self.ctrl.rain_sensor_enabled
            assert status.cloud_sensor_enabled == self.ctrl.cloud_sensor_enabled


if __name__ == "__main__":
    unittest.main()

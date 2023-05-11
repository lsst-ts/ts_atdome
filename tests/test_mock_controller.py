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
import contextlib
import unittest

import pytest
from lsst.ts import atdome, tcpip, utils

# Standard timeout (sec)
STD_TIMEOUT = 2


class MockTestCase(tcpip.BaseOneClientServerTestCase):
    """Test MockDomeController, Status and LongStatus"""

    server_class = atdome.MockDomeController

    @contextlib.asynccontextmanager
    async def create_client(self, server):
        async with super().create_client(server, terminator=b"\n") as client:
            read_bytes = await asyncio.wait_for(
                client.readuntil(b">"), timeout=STD_TIMEOUT
            )
            assert read_bytes == b"ACE Main Box\n>"
            yield client

    async def send_cmd(self, client, cmd, timeout=2):
        """Send a command to the mock controller and wait for the reply.

        Return the decoded reply as 0 or more lines of text
        with the final ">" stripped.
        """
        await asyncio.wait_for(client.write_str(cmd), timeout=STD_TIMEOUT)
        read_bytes = await asyncio.wait_for(client.readuntil(b">"), timeout=timeout)
        # [0:-1] strips the final ">"
        read_str = read_bytes.decode()[0:-1].strip()
        return read_str.split("\n")

    async def test_initial_full_status(self):
        async with self.create_server() as mock_ctrl, self.create_client(
            mock_ctrl
        ) as client:
            reply_lines = await self.send_cmd(client=client, cmd="+")

            status = atdome.Status(reply_lines)
            assert status.main_door_pct == 0
            assert status.dropout_door_pct == 0
            assert status.auto_shutdown_enabled == mock_ctrl.auto_shutdown_enabled
            assert status.sensor_code == 0
            assert status.az_pos == pytest.approx(atdome.INITIAL_AZIMUTH)
            assert status.move_code == 0
            assert not status.homed
            assert status.estop_active == mock_ctrl.estop_active
            assert status.scb_link_ok == mock_ctrl.scb_link_ok
            assert status.rain_sensor_enabled == mock_ctrl.rain_sensor_enabled
            assert status.cloud_sensor_enabled == mock_ctrl.cloud_sensor_enabled
            assert status.coast == pytest.approx(mock_ctrl.coast)
            assert status.tolerance == pytest.approx(mock_ctrl.tolerance)
            assert status.home_azimuth == pytest.approx(mock_ctrl.home_az)
            assert status.high_speed == pytest.approx(mock_ctrl.high_speed)
            assert status.watchdog_timer == pytest.approx(mock_ctrl.watchdog_reset_time)
            assert status.reversal_delay == pytest.approx(mock_ctrl.reverse_delay)
            assert status.encoder_counts_per_360 == mock_ctrl.encoder_counts_per_360
            assert status.main_door_encoder_closed == mock_ctrl.main_door_encoder_closed
            assert status.main_door_encoder_opened == mock_ctrl.main_door_encoder_opened
            assert (
                status.dropout_door_encoder_closed
                == mock_ctrl.dropout_door_encoder_closed
            )
            assert (
                status.dropout_door_encoder_opened
                == mock_ctrl.dropout_door_encoder_opened
            )
            assert status.door_move_timeout == pytest.approx(
                mock_ctrl.door_move_timeout
            )

    async def test_fail_cmd(self):
        async with self.create_server() as mock_ctrl, self.create_client(
            mock_ctrl
        ) as client:
            mock_ctrl.fail_command = "+"
            reply_lines = await self.send_cmd(client=client, cmd="+")
            assert len(reply_lines) == 1
            assert "failed" in reply_lines[0]
            # Make sure the command is only failed once.
            reply_lines = await self.send_cmd(client=client, cmd="+")
            assert len(reply_lines) == 27

    async def test_move_az(self):
        async with self.create_server() as mock_ctrl, self.create_client(
            mock_ctrl
        ) as client:
            # Test initial conditions; some details of this test
            # may have to be changed if this value changes.
            assert atdome.INITIAL_AZIMUTH == pytest.approx(285)

            daz = -3
            az = atdome.INITIAL_AZIMUTH + daz
            est_duration = abs(daz / mock_ctrl.az_vel)
            reply_lines = await self.send_cmd(client=client, cmd=f"{az:0.2f} MV")
            assert reply_lines == [""]
            await asyncio.sleep(est_duration / 2)
            reply_lines = await self.send_cmd(client=client, cmd="+")
            status = atdome.Status(reply_lines)
            assert status.az_pos < atdome.INITIAL_AZIMUTH
            assert status.az_pos > atdome.INITIAL_AZIMUTH + daz
            assert status.move_code == 2

            # wait long enough for the move to finish and check status
            await asyncio.sleep(est_duration / 2 + 0.5)
            reply_lines = await self.send_cmd(client=client, cmd="+")
            status = atdome.Status(reply_lines)
            assert status.az_pos == pytest.approx(az)
            assert status.move_code == 0

    async def test_home_az(self):
        async with self.create_server() as mock_ctrl, self.create_client(
            mock_ctrl
        ) as client:
            assert not mock_ctrl.homed
            daz = -2
            est_ccw_duration = abs(daz / mock_ctrl.az_vel)
            curr_az = mock_ctrl.az_actuator.position(utils.current_tai())
            home_azimuth = utils.angle_wrap_nonnegative(curr_az + daz).deg
            mock_ctrl.home_az = home_azimuth

            reply_lines = await self.send_cmd(client=client, cmd="HM")
            assert reply_lines == [""]

            # sleep until halfway through CCW motion and check status
            await asyncio.sleep(est_ccw_duration / 2)
            reply_lines = await self.send_cmd(client=client, cmd="+")
            status = atdome.Status(reply_lines)
            assert status.move_code == 2 + 64
            assert mock_ctrl.az_actuator.speed == pytest.approx(mock_ctrl.az_vel)
            assert not mock_ctrl.homed
            assert not status.homed

            # sleep until halfway through CW motion and check status
            await asyncio.sleep(
                mock_ctrl.az_actuator.remaining_time() + est_ccw_duration / 2
            )
            reply_lines = await self.send_cmd(client=client, cmd="+")
            status = atdome.Status(reply_lines)
            assert status.move_code == 1 + 64
            assert mock_ctrl.az_actuator.speed == pytest.approx(mock_ctrl.home_az_vel)
            assert not mock_ctrl.homed
            assert not status.homed

            # sleep until we're done and check status
            await asyncio.sleep(mock_ctrl.az_actuator.remaining_time() + 0.1)
            reply_lines = await self.send_cmd(client=client, cmd="+")
            status = atdome.Status(reply_lines)
            assert status.move_code == 0
            assert mock_ctrl.az_actuator.speed == pytest.approx(mock_ctrl.az_vel)
            assert mock_ctrl.az_actuator.position(utils.current_tai()) == pytest.approx(
                mock_ctrl.home_az
            )
            assert mock_ctrl.homed
            assert status.homed

    async def test_az_home_switch(self):
        async with self.create_server() as mock_ctrl, self.create_client(
            mock_ctrl
        ) as client:
            daz = -2
            curr_az = mock_ctrl.az_actuator.position(utils.current_tai())
            mock_ctrl.home_az = utils.angle_wrap_nonnegative(curr_az + daz).deg

            # Move to left edge, center, and right edge of home switch
            # and assert on the home switch each time.
            for az in (
                mock_ctrl.home_az - mock_ctrl.home_az_tolerance + 0.01,
                mock_ctrl.home_az + mock_ctrl.home_az_tolerance - 0.01,
                mock_ctrl.home_az,
            ):
                reply_lines = await self.send_cmd(client=client, cmd=f"{az:0.2f} MV")
                assert reply_lines == [""]

                # Sleep until motion finished, then check status.
                # Wait a bit extra for Docker macOS clock non-monotonicity.
                await asyncio.sleep(mock_ctrl.az_actuator.remaining_time() + 0.2)
                reply_lines = await self.send_cmd(client=client, cmd="+")
                status = atdome.Status(reply_lines)
                assert status.move_code == 0
                assert status.az_pos == pytest.approx(az)
                assert status.az_home_switch

            # Move just off of the az switch in both directions
            for az in (
                mock_ctrl.home_az - mock_ctrl.home_az_tolerance - 0.01,
                mock_ctrl.home_az + mock_ctrl.home_az_tolerance + 0.01,
            ):
                reply_lines = await self.send_cmd(client=client, cmd=f"{az:0.2f} MV")
                assert reply_lines == [""]

                # Sleep until motion finished, then check status.
                # Wait a bit extra for Docker macOS clock non-monotonicity.
                await asyncio.sleep(mock_ctrl.az_actuator.remaining_time() + 0.2)
                reply_lines = await self.send_cmd(client=client, cmd="+")
                status = atdome.Status(reply_lines)
                assert status.move_code == 0
                assert status.az_pos == pytest.approx(az)
                assert not status.az_home_switch

    async def test_main_door(self):
        async with self.create_server() as mock_ctrl, self.create_client(
            mock_ctrl
        ) as client:
            est_duration = mock_ctrl.door_time

            # open main door
            reply_lines = await self.send_cmd(client=client, cmd="OP")
            assert reply_lines == [""]
            await asyncio.sleep(est_duration / 2)
            reply_lines = await self.send_cmd(client=client, cmd="+")
            status = atdome.Status(reply_lines)
            assert status.main_door_pct < 100
            assert status.main_door_pct > 0
            assert status.move_code == 8

            # wait long enough for the move to finish and check status
            await asyncio.sleep(est_duration / 2 + 0.5)
            reply_lines = await self.send_cmd(client=client, cmd="+")
            status = atdome.Status(reply_lines)
            assert status.main_door_pct == pytest.approx(100)
            assert status.move_code == 0

            # close main door
            reply_lines = await self.send_cmd(client=client, cmd="CL")
            assert reply_lines == [""]
            await asyncio.sleep(est_duration / 2)
            reply_lines = await self.send_cmd(client=client, cmd="+")
            status = atdome.Status(reply_lines)
            assert status.main_door_pct < 100
            assert status.main_door_pct > 0
            assert status.move_code == 4

            # wait long enough for the move to finish and check status
            await asyncio.sleep(est_duration / 2 + 0.5)
            reply_lines = await self.send_cmd(client=client, cmd="+")
            status = atdome.Status(reply_lines)
            assert status.main_door_pct == pytest.approx(0)
            assert status.move_code == 0

    async def test_dropout_door(self):
        async with self.create_server() as mock_ctrl, self.create_client(
            mock_ctrl
        ) as client:
            est_duration = mock_ctrl.door_time

            # open dropout door
            reply_lines = await self.send_cmd(client=client, cmd="DN")
            assert reply_lines == [""]
            await asyncio.sleep(est_duration / 2)
            reply_lines = await self.send_cmd(client=client, cmd="+")
            status = atdome.Status(reply_lines)
            assert status.dropout_door_pct < 100
            assert status.dropout_door_pct > 0
            assert status.move_code == 32

            # wait long enough for the move to finish and check status
            await asyncio.sleep(est_duration / 2 + 0.5)
            reply_lines = await self.send_cmd(client=client, cmd="+")
            status = atdome.Status(reply_lines)
            assert status.dropout_door_pct == pytest.approx(100)
            assert status.move_code == 0

            # close dropout door
            reply_lines = await self.send_cmd(client=client, cmd="UP")
            assert reply_lines == [""]
            await asyncio.sleep(est_duration / 2)
            reply_lines = await self.send_cmd(client=client, cmd="+")
            status = atdome.Status(reply_lines)
            assert status.dropout_door_pct < 100
            assert status.dropout_door_pct > 0
            assert status.move_code == 16

            # wait long enough for the move to finish and check status
            await asyncio.sleep(est_duration / 2 + 0.5)
            reply_lines = await self.send_cmd(client=client, cmd="+")
            status = atdome.Status(reply_lines)
            assert status.dropout_door_pct == pytest.approx(0)
            assert status.move_code == 0

    async def test_simultaneous_door_motion(self):
        async with self.create_server() as mock_ctrl, self.create_client(
            mock_ctrl
        ) as client:
            est_duration = mock_ctrl.door_time

            # open both doors
            reply_lines = await self.send_cmd(client=client, cmd="SO")
            assert reply_lines == [""]
            await asyncio.sleep(est_duration / 2)
            reply_lines = await self.send_cmd(client=client, cmd="+")
            status = atdome.Status(reply_lines)
            assert status.main_door_pct < 100
            assert status.main_door_pct > 0
            assert status.dropout_door_pct < 100
            assert status.dropout_door_pct > 0
            assert status.move_code == 8 + 32

            # wait long enough for the move to finish and check status
            await asyncio.sleep(est_duration / 2 + 0.5)
            reply_lines = await self.send_cmd(client=client, cmd="+")
            status = atdome.Status(reply_lines)
            assert status.main_door_pct == pytest.approx(100)
            assert status.dropout_door_pct == pytest.approx(100)
            assert status.move_code == 0

            # close dropout door
            reply_lines = await self.send_cmd(client=client, cmd="SC")
            assert reply_lines == [""]
            await asyncio.sleep(est_duration / 2)
            reply_lines = await self.send_cmd(client=client, cmd="+")
            status = atdome.Status(reply_lines)
            assert status.main_door_pct < 100
            assert status.main_door_pct > 0
            assert status.dropout_door_pct < 100
            assert status.dropout_door_pct > 0
            assert status.move_code == 4 + 16

            # wait long enough for the move to finish and check status
            await asyncio.sleep(est_duration / 2 + 0.5)
            reply_lines = await self.send_cmd(client=client, cmd="+")
            status = atdome.Status(reply_lines)
            assert status.main_door_pct == pytest.approx(0)
            assert status.dropout_door_pct == pytest.approx(0)
            assert status.move_code == 0

    async def test_stop(self):
        async with self.create_server() as mock_ctrl, self.create_client(
            mock_ctrl
        ) as client:
            est_duration = mock_ctrl.door_time
            daz = mock_ctrl.az_vel * est_duration
            az = atdome.INITIAL_AZIMUTH + daz

            # start azimuth motion
            reply_lines = await self.send_cmd(client=client, cmd=f"{az:0.2f} MV")
            # open both doors
            reply_lines = await self.send_cmd(client=client, cmd="SO")
            assert reply_lines == [""]

            # wait for the moves to get about halfway and stop
            await asyncio.sleep(est_duration / 2)
            reply_lines = await self.send_cmd(client=client, cmd="ST")
            reply_lines = await self.send_cmd(client=client, cmd="+")
            status = atdome.Status(reply_lines)
            assert status.az_pos < az
            assert status.az_pos > 0
            assert status.main_door_pct < 100
            assert status.main_door_pct > 0
            assert status.dropout_door_pct < 100
            assert status.dropout_door_pct > 0
            assert status.move_code == 0

    async def test_long_status(self):
        async with self.create_server() as mock_ctrl, self.create_client(
            mock_ctrl
        ) as client:
            mock_ctrl.rain_enabled = False

            reply_lines = await self.send_cmd(client=client, cmd="+")
            status = atdome.Status(reply_lines)
            assert status.estop_active == mock_ctrl.estop_active
            assert status.scb_link_ok == status.scb_link_ok
            assert status.rain_sensor_enabled == mock_ctrl.rain_sensor_enabled
            assert status.cloud_sensor_enabled == mock_ctrl.cloud_sensor_enabled

            # Toggle several values, one at a time, and check that the output
            # is updated as expected
            for name in (
                "estop_active",
                "scb_link_ok",
                "rain_sensor_enabled",
                "cloud_sensor_enabled",
            ):
                setattr(mock_ctrl, name, not getattr(mock_ctrl, name))

                reply_lines = await self.send_cmd(client=client, cmd="+")
                status = atdome.Status(reply_lines)
                assert status.estop_active == mock_ctrl.estop_active
                assert status.scb_link_ok == status.scb_link_ok
                assert status.rain_sensor_enabled == mock_ctrl.rain_sensor_enabled
                assert status.cloud_sensor_enabled == mock_ctrl.cloud_sensor_enabled


if __name__ == "__main__":
    unittest.main()

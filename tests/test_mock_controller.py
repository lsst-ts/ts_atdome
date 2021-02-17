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
import unittest

import asynctest

from lsst.ts import salobj
from lsst.ts import ATDome


class MockTestCase(asynctest.TestCase):
    """Test MockDomeController, Status and LongStatus
    """

    async def setUp(self):
        self.ctrl = None
        self.writer = None

        self.ctrl = ATDome.MockDomeController(port=0)
        await asyncio.wait_for(self.ctrl.start(), 5)
        rw_coro = asyncio.open_connection(host="127.0.0.1", port=self.ctrl.port)
        self.reader, self.writer = await asyncio.wait_for(rw_coro, timeout=5)
        read_bytes = await asyncio.wait_for(
            self.reader.readuntil(">".encode()), timeout=5
        )
        read_str = read_bytes.decode().strip()
        self.assertEqual(read_str, "ACE Main Box\n>")

    async def tearDown(self):
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

        status = ATDome.Status(reply_lines)
        self.assertEqual(status.main_door_pct, 0)
        self.assertEqual(status.dropout_door_pct, 0)
        self.assertEqual(status.auto_shutdown_enabled, self.ctrl.auto_shutdown_enabled)
        self.assertEqual(status.sensor_code, 0)
        self.assertAlmostEqual(status.az_pos, ATDome.INITIAL_AZIMUTH)
        self.assertEqual(status.move_code, 0)
        self.assertEqual(status.estop_active, self.ctrl.estop_active)
        self.assertEqual(status.scb_link_ok, self.ctrl.scb_link_ok)
        self.assertEqual(status.rain_sensor_enabled, self.ctrl.rain_sensor_enabled)
        self.assertEqual(status.cloud_sensor_enabled, self.ctrl.cloud_sensor_enabled)
        self.assertAlmostEqual(status.coast, self.ctrl.coast)
        self.assertAlmostEqual(status.tolerance, self.ctrl.tolerance)
        self.assertAlmostEqual(status.home_azimuth, self.ctrl.home_az)
        self.assertAlmostEqual(status.high_speed, self.ctrl.high_speed)
        self.assertAlmostEqual(status.watchdog_timer, self.ctrl.watchdog_reset_time)
        self.assertAlmostEqual(status.reversal_delay, self.ctrl.reverse_delay)
        self.assertEqual(
            status.encoder_counts_per_360, self.ctrl.encoder_counts_per_360
        )
        self.assertEqual(
            status.main_door_encoder_closed, self.ctrl.main_door_encoder_closed
        )
        self.assertEqual(
            status.main_door_encoder_opened, self.ctrl.main_door_encoder_opened
        )
        self.assertEqual(
            status.dropout_door_encoder_closed, self.ctrl.dropout_door_encoder_closed
        )
        self.assertEqual(
            status.dropout_door_encoder_opened, self.ctrl.dropout_door_encoder_opened
        )
        self.assertAlmostEqual(status.door_move_timeout, self.ctrl.door_move_timeout)

    async def test_fail_cmd(self):
        self.ctrl.fail_command = "+"
        reply_lines = await self.send_cmd("+")
        self.assertEqual(len(reply_lines), 1)
        self.assertIn("failed", reply_lines[0])
        # Make sure the command is only failed once.
        reply_lines = await self.send_cmd("+")
        self.assertEqual(len(reply_lines), 25)

    async def test_move_az(self):
        # Test initial conditions; some details of this test
        # may have to be changed if this value changes.
        self.assertAlmostEqual(ATDome.INITIAL_AZIMUTH, 285)

        daz = -3
        az = ATDome.INITIAL_AZIMUTH + daz
        est_duration = abs(daz / self.ctrl.az_vel)
        reply_lines = await self.send_cmd(f"{az:0.2f} MV")
        self.assertEqual(reply_lines, [""])
        await asyncio.sleep(est_duration / 2)
        reply_lines = await self.send_cmd("+")
        status = ATDome.Status(reply_lines)
        self.assertLess(status.az_pos, ATDome.INITIAL_AZIMUTH)
        self.assertGreater(status.az_pos, ATDome.INITIAL_AZIMUTH + daz)
        self.assertEqual(status.move_code, 2)

        # wait long enough for the move to finish and check status
        await asyncio.sleep(est_duration / 2 + 0.5)
        reply_lines = await self.send_cmd("+")
        status = ATDome.Status(reply_lines)
        self.assertAlmostEqual(status.az_pos, az)
        self.assertEqual(status.move_code, 0)

    async def test_home_az(self):
        daz = -2
        est_ccw_duration = abs(daz / self.ctrl.az_vel)
        curr_az = self.ctrl.az_actuator.position(salobj.current_tai())
        home_azimuth = salobj.angle_wrap_nonnegative(curr_az - 2).deg
        self.ctrl.home_az = home_azimuth

        reply_lines = await self.send_cmd("HM")
        self.assertEqual(reply_lines, [""])

        # sleep until halfway through CCW motion and check status
        await asyncio.sleep(est_ccw_duration / 2)
        reply_lines = await self.send_cmd("+")
        status = ATDome.Status(reply_lines)
        self.assertEqual(status.move_code, 2 + 64)
        self.assertAlmostEqual(self.ctrl.az_actuator.speed, self.ctrl.az_vel)

        # sleep until halfway through CW motion and check status
        await asyncio.sleep(
            self.ctrl.az_actuator.remaining_time() + est_ccw_duration / 2
        )
        reply_lines = await self.send_cmd("+")
        status = ATDome.Status(reply_lines)
        self.assertEqual(status.move_code, 1 + 64)
        self.assertAlmostEqual(self.ctrl.az_actuator.speed, self.ctrl.home_az_vel)

        # sleep until we're done and check status
        await asyncio.sleep(self.ctrl.az_actuator.remaining_time() + 0.1)
        reply_lines = await self.send_cmd("+")
        status = ATDome.Status(reply_lines)
        self.assertEqual(status.move_code, 0)
        self.assertAlmostEqual(self.ctrl.az_actuator.speed, self.ctrl.az_vel)
        self.assertAlmostEqual(
            self.ctrl.az_actuator.position(salobj.current_tai()), self.ctrl.home_az
        )

    async def test_main_door(self):
        est_duration = self.ctrl.door_time

        # open main door
        reply_lines = await self.send_cmd("OP")
        self.assertEqual(reply_lines, [""])
        await asyncio.sleep(est_duration / 2)
        reply_lines = await self.send_cmd("+")
        status = ATDome.Status(reply_lines)
        self.assertLess(status.main_door_pct, 100)
        self.assertGreater(status.main_door_pct, 0)
        self.assertEqual(status.move_code, 8)

        # wait long enough for the move to finish and check status
        await asyncio.sleep(est_duration / 2 + 0.5)
        reply_lines = await self.send_cmd("+")
        status = ATDome.Status(reply_lines)
        self.assertAlmostEqual(status.main_door_pct, 100)
        self.assertEqual(status.move_code, 0)

        # close main door
        reply_lines = await self.send_cmd("CL")
        self.assertEqual(reply_lines, [""])
        await asyncio.sleep(est_duration / 2)
        reply_lines = await self.send_cmd("+")
        status = ATDome.Status(reply_lines)
        self.assertLess(status.main_door_pct, 100)
        self.assertGreater(status.main_door_pct, 0)
        self.assertEqual(status.move_code, 4)

        # wait long enough for the move to finish and check status
        await asyncio.sleep(est_duration / 2 + 0.5)
        reply_lines = await self.send_cmd("+")
        status = ATDome.Status(reply_lines)
        self.assertAlmostEqual(status.main_door_pct, 0)
        self.assertEqual(status.move_code, 0)

    async def test_dropout_door(self):
        est_duration = self.ctrl.door_time

        # open dropout door
        reply_lines = await self.send_cmd("DN")
        self.assertEqual(reply_lines, [""])
        await asyncio.sleep(est_duration / 2)
        reply_lines = await self.send_cmd("+")
        status = ATDome.Status(reply_lines)
        self.assertLess(status.dropout_door_pct, 100)
        self.assertGreater(status.dropout_door_pct, 0)
        self.assertEqual(status.move_code, 32)

        # wait long enough for the move to finish and check status
        await asyncio.sleep(est_duration / 2 + 0.5)
        reply_lines = await self.send_cmd("+")
        status = ATDome.Status(reply_lines)
        self.assertAlmostEqual(status.dropout_door_pct, 100)
        self.assertEqual(status.move_code, 0)

        # close dropout door
        reply_lines = await self.send_cmd("UP")
        self.assertEqual(reply_lines, [""])
        await asyncio.sleep(est_duration / 2)
        reply_lines = await self.send_cmd("+")
        status = ATDome.Status(reply_lines)
        self.assertLess(status.dropout_door_pct, 100)
        self.assertGreater(status.dropout_door_pct, 0)
        self.assertEqual(status.move_code, 16)

        # wait long enough for the move to finish and check status
        await asyncio.sleep(est_duration / 2 + 0.5)
        reply_lines = await self.send_cmd("+")
        status = ATDome.Status(reply_lines)
        self.assertAlmostEqual(status.dropout_door_pct, 0)
        self.assertEqual(status.move_code, 0)

    async def test_simultaneous_door_motion(self):
        est_duration = self.ctrl.door_time

        # open both doors
        reply_lines = await self.send_cmd("SO")
        self.assertEqual(reply_lines, [""])
        await asyncio.sleep(est_duration / 2)
        reply_lines = await self.send_cmd("+")
        status = ATDome.Status(reply_lines)
        self.assertLess(status.main_door_pct, 100)
        self.assertGreater(status.main_door_pct, 0)
        self.assertLess(status.dropout_door_pct, 100)
        self.assertGreater(status.dropout_door_pct, 0)
        self.assertEqual(status.move_code, 8 + 32)

        # wait long enough for the move to finish and check status
        await asyncio.sleep(est_duration / 2 + 0.5)
        reply_lines = await self.send_cmd("+")
        status = ATDome.Status(reply_lines)
        self.assertAlmostEqual(status.main_door_pct, 100)
        self.assertAlmostEqual(status.dropout_door_pct, 100)
        self.assertEqual(status.move_code, 0)

        # close dropout door
        reply_lines = await self.send_cmd("SC")
        self.assertEqual(reply_lines, [""])
        await asyncio.sleep(est_duration / 2)
        reply_lines = await self.send_cmd("+")
        status = ATDome.Status(reply_lines)
        self.assertLess(status.main_door_pct, 100)
        self.assertGreater(status.main_door_pct, 0)
        self.assertLess(status.dropout_door_pct, 100)
        self.assertGreater(status.dropout_door_pct, 0)
        self.assertEqual(status.move_code, 4 + 16)

        # wait long enough for the move to finish and check status
        await asyncio.sleep(est_duration / 2 + 0.5)
        reply_lines = await self.send_cmd("+")
        status = ATDome.Status(reply_lines)
        self.assertAlmostEqual(status.main_door_pct, 0)
        self.assertAlmostEqual(status.dropout_door_pct, 0)
        self.assertEqual(status.move_code, 0)

    async def test_stop(self):
        est_duration = self.ctrl.door_time
        daz = self.ctrl.az_vel * est_duration
        az = ATDome.INITIAL_AZIMUTH + daz

        # start azimuth motion
        reply_lines = await self.send_cmd(f"{az:0.2f} MV")
        # open both doors
        reply_lines = await self.send_cmd("SO")
        self.assertEqual(reply_lines, [""])

        # wait for the moves to get about halfway and stop
        await asyncio.sleep(est_duration / 2)
        reply_lines = await self.send_cmd("ST")
        reply_lines = await self.send_cmd("+")
        status = ATDome.Status(reply_lines)
        self.assertLess(status.az_pos, az)
        self.assertGreater(status.az_pos, 0)
        self.assertLess(status.main_door_pct, 100)
        self.assertGreater(status.main_door_pct, 0)
        self.assertLess(status.dropout_door_pct, 100)
        self.assertGreater(status.dropout_door_pct, 0)
        self.assertEqual(status.move_code, 0)

    async def test_long_status(self):
        self.ctrl.rain_enabled = False

        reply_lines = await self.send_cmd("+")
        status = ATDome.Status(reply_lines)
        self.assertEqual(status.estop_active, self.ctrl.estop_active)
        self.assertEqual(status.scb_link_ok, status.scb_link_ok)
        self.assertEqual(status.rain_sensor_enabled, self.ctrl.rain_sensor_enabled)
        self.assertEqual(status.cloud_sensor_enabled, self.ctrl.cloud_sensor_enabled)

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
            status = ATDome.Status(reply_lines)
            self.assertEqual(status.estop_active, self.ctrl.estop_active)
            self.assertEqual(status.scb_link_ok, status.scb_link_ok)
            self.assertEqual(status.rain_sensor_enabled, self.ctrl.rain_sensor_enabled)
            self.assertEqual(
                status.cloud_sensor_enabled, self.ctrl.cloud_sensor_enabled
            )


if __name__ == "__main__":
    unittest.main()

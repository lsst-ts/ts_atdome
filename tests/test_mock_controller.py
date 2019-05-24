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

from astropy.coordinates import Angle
import astropy.units as u

from lsst.ts import salobj
from lsst.ts import ATDome

port_generator = salobj.index_generator(imin=3100)


class MockTestCase(unittest.TestCase):
    """Test MockDomeController, ShortStatus and LongStatus
    """
    def setUp(self):
        self.port = next(port_generator)
        self.ctrl = None
        self.writer = None

        async def doit():
            self.ctrl = ATDome.MockDomeController(port=self.port)
            await asyncio.wait_for(self.ctrl.start(), 5)
            rw_coro = asyncio.open_connection(host="127.0.0.1", port=self.port)
            self.reader, self.writer = await asyncio.wait_for(rw_coro, timeout=5)
            read_bytes = await asyncio.wait_for(self.reader.readuntil(">".encode()), timeout=5)
            read_str = read_bytes.decode().strip()
            self.assertEqual(read_str, "ACE Main Box\n>")
        asyncio.get_event_loop().run_until_complete(doit())

    def tearDown(self):
        async def doit():
            if self.ctrl:
                await asyncio.wait_for(self.ctrl.stop(), 5)
            if self.writer:
                self.writer.close()

        asyncio.get_event_loop().run_until_complete(doit())

    async def send_cmd(self, cmd, timeout=2):
        """Send a command to the mock controller and wait for the reply.

        Return the decoded reply as 0 or more lines of text
        with the final ">" stripped.
        """
        self.writer.write(f"{cmd}\n".encode())
        await self.writer.drain()
        read_bytes = await asyncio.wait_for(self.reader.readuntil(">".encode()), timeout=timeout)
        # [0:-1] strips the final ">"
        read_str = read_bytes.decode()[0:-1].strip()
        return read_str.split("\n")

    def test_initial_short_status(self):
        async def doit():
            reply_lines = await self.send_cmd("?")

            status = ATDome.ShortStatus(reply_lines)
            self.assertEqual(status.main_door_pct, 0)
            self.assertEqual(status.dropout_door_pct, 0)
            self.assertTrue(status.auto_shutdown_enabled)
            self.assertEqual(status.sensor_code, 0)
            self.assertAlmostEqual(status.az_pos.deg, 0)
            self.assertEqual(status.move_code, 0)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_initial_full_status(self):
        async def doit():
            reply_lines = await self.send_cmd("+")

            status = ATDome.ShortStatus(reply_lines[0:5])
            self.assertEqual(status.main_door_pct, 0)
            self.assertEqual(status.dropout_door_pct, 0)
            self.assertTrue(status.auto_shutdown_enabled)
            self.assertEqual(status.sensor_code, 0)
            self.assertAlmostEqual(status.az_pos.deg, 0)
            self.assertEqual(status.move_code, 0)

            rem_status = ATDome.RemainingStatus(reply_lines)
            self.assertFalse(rem_status.estop_active)
            self.assertTrue(rem_status.scb_link_ok)
            self.assertTrue(rem_status.rain_sensor_enabled)
            self.assertTrue(rem_status.cloud_sensor_enabled)
            self.assertAlmostEqual(rem_status.tolerance.deg, 1)
            self.assertAlmostEqual(rem_status.home_azimuth.deg, self.ctrl.home_az.deg)
            self.assertAlmostEqual(rem_status.high_speed.deg, 6)
            self.assertAlmostEqual(rem_status.watchdog_timer, 600)
            self.assertAlmostEqual(rem_status.reversal_delay, 2)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_move_az(self):
        async def doit():
            daz = -3
            az = 360 + daz
            est_duration = abs(daz / self.ctrl.az_vel.deg)
            reply_lines = await self.send_cmd(f"{az:0.2f} MV")
            self.assertEqual(reply_lines, [""])
            await asyncio.sleep(est_duration/2)
            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertLess(status.az_pos.deg, 360)
            self.assertGreater(status.az_pos.deg, 357)
            self.assertEqual(status.move_code, 2)

            # wait long enough for the move to finish and check status
            await asyncio.sleep(est_duration/2 + 0.5)
            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertAlmostEqual(status.az_pos.deg, 357)
            self.assertEqual(status.move_code, 0)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_home_az(self):
        async def doit():
            daz = -2
            est_ccw_duration = abs(daz / self.ctrl.az_vel.deg)
            curr_az = self.ctrl.az_actuator.curr_pos
            home_azimuth = (curr_az - 2*u.deg).wrap_at(Angle(360, u.deg))
            self.ctrl.home_az = home_azimuth

            reply_lines = await self.send_cmd("HM")
            self.assertEqual(reply_lines, [""])

            # sleep until halfway through CCW motion and check status
            await asyncio.sleep(est_ccw_duration/2)
            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertEqual(status.move_code, 2 + 64)
            self.assertAlmostEqual(self.ctrl.az_actuator.speed.deg, self.ctrl.az_vel.deg)

            # sleep until halfway through CW motion and check status
            await asyncio.sleep(self.ctrl.az_actuator.remaining_time + est_ccw_duration/2)
            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertEqual(status.move_code, 1 + 64)
            self.assertAlmostEqual(self.ctrl.az_actuator.speed.deg, self.ctrl.home_az_vel.deg)

            # sleep until we're done and check status
            await asyncio.sleep(self.ctrl.az_actuator.remaining_time + 0.1)
            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertEqual(status.move_code, 0)
            self.assertAlmostEqual(self.ctrl.az_actuator.speed.deg, self.ctrl.az_vel.deg)
            self.assertAlmostEqual(self.ctrl.az_actuator.curr_pos.deg, self.ctrl.home_az.deg)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_main_door(self):
        async def doit():
            est_duration = self.ctrl.door_time

            # open main door
            reply_lines = await self.send_cmd("OP")
            self.assertEqual(reply_lines, [""])
            await asyncio.sleep(est_duration/2)
            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertLess(status.main_door_pct, 100)
            self.assertGreater(status.main_door_pct, 0)
            self.assertEqual(status.move_code, 8)

            # wait long enough for the move to finish and check status
            await asyncio.sleep(est_duration/2 + 0.5)
            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertAlmostEqual(status.main_door_pct, 100)
            self.assertEqual(status.move_code, 0)

            # close main door
            reply_lines = await self.send_cmd("CL")
            self.assertEqual(reply_lines, [""])
            await asyncio.sleep(est_duration/2)
            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertLess(status.main_door_pct, 100)
            self.assertGreater(status.main_door_pct, 0)
            self.assertEqual(status.move_code, 4)

            # wait long enough for the move to finish and check status
            await asyncio.sleep(est_duration/2 + 0.5)
            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertAlmostEqual(status.main_door_pct, 0)
            self.assertEqual(status.move_code, 0)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_dropout_door(self):
        async def doit():
            est_duration = self.ctrl.door_time

            # open dropout door
            reply_lines = await self.send_cmd("DN")
            self.assertEqual(reply_lines, [""])
            await asyncio.sleep(est_duration/2)
            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertLess(status.dropout_door_pct, 100)
            self.assertGreater(status.dropout_door_pct, 0)
            self.assertEqual(status.move_code, 32)

            # wait long enough for the move to finish and check status
            await asyncio.sleep(est_duration/2 + 0.5)
            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertAlmostEqual(status.dropout_door_pct, 100)
            self.assertEqual(status.move_code, 0)

            # close dropout door
            reply_lines = await self.send_cmd("UP")
            self.assertEqual(reply_lines, [""])
            await asyncio.sleep(est_duration/2)
            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertLess(status.dropout_door_pct, 100)
            self.assertGreater(status.dropout_door_pct, 0)
            self.assertEqual(status.move_code, 16)

            # wait long enough for the move to finish and check status
            await asyncio.sleep(est_duration/2 + 0.5)
            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertAlmostEqual(status.dropout_door_pct, 0)
            self.assertEqual(status.move_code, 0)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_simultaneous_door_motion(self):
        async def doit():
            est_duration = self.ctrl.door_time

            # open both doors
            reply_lines = await self.send_cmd("SO")
            self.assertEqual(reply_lines, [""])
            await asyncio.sleep(est_duration/2)
            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertLess(status.main_door_pct, 100)
            self.assertGreater(status.main_door_pct, 0)
            self.assertLess(status.dropout_door_pct, 100)
            self.assertGreater(status.dropout_door_pct, 0)
            self.assertEqual(status.move_code, 8 + 32)

            # wait long enough for the move to finish and check status
            await asyncio.sleep(est_duration/2 + 0.5)
            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertAlmostEqual(status.main_door_pct, 100)
            self.assertAlmostEqual(status.dropout_door_pct, 100)
            self.assertEqual(status.move_code, 0)

            # close dropout door
            reply_lines = await self.send_cmd("SC")
            self.assertEqual(reply_lines, [""])
            await asyncio.sleep(est_duration/2)
            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertLess(status.main_door_pct, 100)
            self.assertGreater(status.main_door_pct, 0)
            self.assertLess(status.dropout_door_pct, 100)
            self.assertGreater(status.dropout_door_pct, 0)
            self.assertEqual(status.move_code, 4 + 16)

            # wait long enough for the move to finish and check status
            await asyncio.sleep(est_duration/2 + 0.5)
            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertAlmostEqual(status.main_door_pct, 0)
            self.assertAlmostEqual(status.dropout_door_pct, 0)
            self.assertEqual(status.move_code, 0)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_stop(self):
        async def doit():
            est_duration = self.ctrl.door_time
            daz = self.ctrl.az_vel * est_duration
            az = daz

            # start azimuth motion
            reply_lines = await self.send_cmd(f"{az.deg:0.2f} MV")
            # open both doors
            reply_lines = await self.send_cmd("SO")
            self.assertEqual(reply_lines, [""])

            # wait for the moves to get about halfway and stop
            await asyncio.sleep(est_duration/2)
            reply_lines = await self.send_cmd("ST")
            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertLess(status.az_pos.deg, az.deg)
            self.assertGreater(status.az_pos.deg, 0)
            self.assertLess(status.main_door_pct, 100)
            self.assertGreater(status.main_door_pct, 0)
            self.assertLess(status.dropout_door_pct, 100)
            self.assertGreater(status.dropout_door_pct, 0)
            self.assertEqual(status.move_code, 0)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_short_status(self):
        async def doit():
            self.ctrl.auto_shutdown_enabled = False

            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertFalse(status.auto_shutdown_enabled)
            self.assertEqual(status.sensor_code, 0)
            self.assertEqual(status.move_code, 0)

            self.ctrl.auto_shutdown_enabled = True
            self.ctrl.rain_detected = True

            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertTrue(status.auto_shutdown_enabled)
            self.assertEqual(status.sensor_code, 1)
            self.assertEqual(status.move_code, 0)

            self.ctrl.clouds_detected = True

            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertTrue(status.auto_shutdown_enabled)
            self.assertEqual(status.sensor_code, 3)
            self.assertEqual(status.move_code, 0)

            self.ctrl.rain_detected = False

            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertTrue(status.auto_shutdown_enabled)
            self.assertEqual(status.sensor_code, 2)
            self.assertEqual(status.move_code, 0)

            self.ctrl.clouds_detected = False
            self.ctrl.estop_active = True

            reply_lines = await self.send_cmd("?")
            status = ATDome.ShortStatus(reply_lines)
            self.assertTrue(status.auto_shutdown_enabled)
            self.assertEqual(status.move_code, 128)

        asyncio.get_event_loop().run_until_complete(doit())

    def test_long_status(self):
        async def doit():
            self.ctrl.rain_enabled = False

            reply_lines = await self.send_cmd("+")
            rem_status = ATDome.RemainingStatus(reply_lines)
            self.assertFalse(rem_status.estop_active)
            self.assertTrue(rem_status.scb_link_ok)
            self.assertFalse(rem_status.rain_sensor_enabled)
            self.assertTrue(rem_status.cloud_sensor_enabled)

            self.ctrl.rain_enabled = True
            self.ctrl.clouds_enabled = False

            reply_lines = await self.send_cmd("+")
            rem_status = ATDome.RemainingStatus(reply_lines)
            self.assertFalse(rem_status.estop_active)
            self.assertTrue(rem_status.scb_link_ok)
            self.assertTrue(rem_status.rain_sensor_enabled)
            self.assertFalse(rem_status.cloud_sensor_enabled)

            self.ctrl.clouds_enabled = True
            self.ctrl.scb_link_ok = False

            reply_lines = await self.send_cmd("+")
            rem_status = ATDome.RemainingStatus(reply_lines)
            self.assertFalse(rem_status.estop_active)
            self.assertFalse(rem_status.scb_link_ok)
            self.assertTrue(rem_status.rain_sensor_enabled)
            self.assertTrue(rem_status.cloud_sensor_enabled)

            self.ctrl.scb_link_ok = True
            self.ctrl.estop_active = True

            reply_lines = await self.send_cmd("+")
            rem_status = ATDome.RemainingStatus(reply_lines)
            self.assertTrue(rem_status.estop_active)
            self.assertTrue(rem_status.scb_link_ok)
            self.assertTrue(rem_status.rain_sensor_enabled)
            self.assertTrue(rem_status.cloud_sensor_enabled)

        asyncio.get_event_loop().run_until_complete(doit())


if __name__ == "__main__":
    unittest.main()

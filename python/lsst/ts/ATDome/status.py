#
# Developed for the LSST Telescope and Site Systems.
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
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

__all__ = ["ShortStatus", "RemainingStatus"]

import re

from astropy.coordinates import Angle
import astropy.units as u

MIDDLE_WRAP_ANGLE = Angle(180, u.deg)


class ShortStatus:
    """Short status from the TCP/IP AT Dome controller.
    """
    def __init__(self, lines):
        if len(lines) != 5:
            raise RuntimeError(f"Got {len(lines)} lines; need 5")

        shutter_match = re.match(r"MAIN +[A-Z]+ +(\d+)", lines[0])
        assert shutter_match
        self.main_door_pct = float(shutter_match.group(1))

        door_match = re.match(r"DROP +[A-Z]+ +(\d+)", lines[1])
        assert door_match
        self.dropout_door_pct = float(door_match.group(1))

        auto_shutdown_match = re.match(r"\[(ON|OFF)\] +(\d+)", lines[2])
        assert auto_shutdown_match
        self.auto_shutdown_enabled = auto_shutdown_match.group(1) == "ON"

        self.sensor_code = int(auto_shutdown_match.group(2))

        az_match = re.match(r"Posn +(\d*\.?\d+)", lines[3])
        assert az_match
        self.az_pos = Angle(float(az_match.group(1)), u.deg)

        code_match = re.match(r"R(L|R) +(\d+)", lines[4])
        assert code_match
        move_code = int(code_match.group(2))
        self.move_code = move_code


class RemainingStatus:
    """Full status from the TCP/IP AT Dome controller,
    excluding short status (the first 5 lines).

    Breaking full status into two object simplifies CSC code.
    """
    def __init__(self, lines):
        if len(lines) != 23:
            raise RuntimeError(f"Got {len(lines)} lines; need 23")

        estop_active_match = re.match(r"Emergency Stop Active: +(\d)", lines[5])
        assert estop_active_match
        self.estop_active = bool(int(estop_active_match.group(1)))

        scb_link_ok_match = re.match(r"SCB radio link OK: +(\d)", lines[6])
        assert scb_link_ok_match
        self.scb_link_ok = bool(int(scb_link_ok_match.group(1)))

        rain_sensor_match = re.match(r"Rain-Snow enabled: +(\d)", lines[14])
        assert rain_sensor_match
        self.rain_sensor_enabled = bool(int(rain_sensor_match.group(1)))

        cloud_sensor_match = re.match(r"Cloud Sensor enabled: +(\d)", lines[15])
        assert cloud_sensor_match
        self.cloud_sensor_enabled = bool(int(cloud_sensor_match.group(1)))

        tolerance_match = re.match(r"Tolerance \(degrees\): +(\d*\.?\d+)", lines[10])
        assert tolerance_match
        self.tolerance = Angle(float(tolerance_match.group(1)), u.deg)

        home_azimuth = re.match(r"Home Azimuth: +(\d*\.?\d+)", lines[7])
        assert home_azimuth
        self.home_azimuth = Angle(float(home_azimuth.group(1)), u.deg)

        high_speed_match = re.match(r"High Speed \(degrees\): +(\d*\.?\d+)", lines[8])
        assert high_speed_match
        self.high_speed = Angle(float(high_speed_match.group(1)), u.deg)

        watchdog_timer_match = re.match(r"Watchdog Reset Time: +(\d*\.?\d+)", lines[16])
        assert watchdog_timer_match
        self.watchdog_timer = float(watchdog_timer_match.group(1))

        reversal_delay_match = re.match(r"Reverse Delay: +(\d*\.?\d+)", lines[18])
        assert reversal_delay_match
        self.reversal_delay = float(reversal_delay_match.group(1))

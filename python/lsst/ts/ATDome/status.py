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


def parse(regex, line):
    """Parse a line of status.

    Parameters
    ----------
    regex : `str`
        Regex to match
    line : `str`
        Line to parse

    Returns
    -------
    match : `re.Match`
        The match.

    Raises
    ------
    RuntimeError
        If the line does not match the regex.
    """
    match = re.match(regex, line)
    if match is None:
        raise RuntimeError(f"Cound not parse {line!r} as {regex}")
    return match


class ShortStatus:
    """Short status from the TCP/IP AT Dome controller.
    """
    def __init__(self, lines):
        if len(lines) != 5:
            raise RuntimeError(f"Got {len(lines)} lines; need 5")

        shutter_match = parse(r"MAIN +[A-Z]+ +(\d+)", lines[0])
        self.main_door_pct = float(shutter_match.group(1))

        door_match = parse(r"DROP +[A-Z]+ +(\d+)", lines[1])
        self.dropout_door_pct = float(door_match.group(1))

        auto_shutdown_match = parse(r"\[(ON|OFF)\] +(\d+)", lines[2])
        self.auto_shutdown_enabled = auto_shutdown_match.group(1) == "ON"

        self.sensor_code = int(auto_shutdown_match.group(2))

        az_match = parse(r"(POSN|HOME) +(\d*\.?\d+)", lines[3])
        self.az_pos = Angle(float(az_match.group(2)), u.deg)

        code_match = parse(r"(RL|RR|--) +(\d+)", lines[4])
        move_code = int(code_match.group(2))
        self.move_code = move_code


class RemainingStatus:
    """Full status from the TCP/IP AT Dome controller,
    excluding short status (the first 5 lines).

    Breaking full status into two object simplifies CSC code.
    """
    def __init__(self, lines):
        if len(lines) != 25:
            raise RuntimeError(f"Got {len(lines)} lines; need 25")

        estop_active_match = parse(r"Emergency Stop Active: +(\d)", lines[5])
        self.estop_active = bool(int(estop_active_match.group(1)))

        scb_link_ok_match = parse(r"Top Comm Link OK: +(\d)", lines[6])
        self.scb_link_ok = bool(int(scb_link_ok_match.group(1)))

        rain_sensor_match = parse(r"Rain-Snow enabled: +(\d)", lines[15])
        self.rain_sensor_enabled = bool(int(rain_sensor_match.group(1)))

        cloud_sensor_match = parse(r"Cloud Sensor enabled: +(\d)", lines[16])
        self.cloud_sensor_enabled = bool(int(cloud_sensor_match.group(1)))

        tolerance_match = parse(r"Tolerance \(degrees\): +(\d*\.?\d+)", lines[10])
        self.tolerance = Angle(float(tolerance_match.group(1)), u.deg)

        home_azimuth = parse(r"Home Azimuth: +(\d*\.?\d+)", lines[7])
        self.home_azimuth = Angle(float(home_azimuth.group(1)), u.deg)

        high_speed_match = parse(r"High Speed \(degrees\): +(\d*\.?\d+)", lines[8])
        self.high_speed = Angle(float(high_speed_match.group(1)), u.deg)

        watchdog_timer_match = parse(r"Watchdog Reset Time: +(\d*\.?\d+)", lines[17])
        self.watchdog_timer = float(watchdog_timer_match.group(1))

        reversal_delay_match = parse(r"Reverse Delay: +(\d*\.?\d+)", lines[19])
        self.reversal_delay = float(reversal_delay_match.group(1))

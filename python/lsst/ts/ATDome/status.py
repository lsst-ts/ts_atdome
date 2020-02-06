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

__all__ = ["Status"]

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


def parse_get(regex, line):
    """Parse a line of status and return the value of group(1)."""
    match = parse(regex, line)
    return match.group(1)


class Status:
    """Parsed data of the output from "+", the full status command.
    """

    def __init__(self, lines):
        if len(lines) != 25:
            raise RuntimeError(f"Got {len(lines)} lines; need 25")

        self.main_door_pct = float(parse_get(r"MAIN +[A-Z]+ +(\d+)", lines[0]))

        self.dropout_door_pct = float(parse_get(r"DROP +[A-Z]+ +(\d+)", lines[1]))

        auto_shutdown_match = parse(r"\[(ON|OFF)\] +(\d+)", lines[2])
        self.auto_shutdown_enabled = auto_shutdown_match.group(1) == "ON"
        self.sensor_code = int(auto_shutdown_match.group(2))

        self.az_pos = Angle(
            float(parse_get(r"(?:POSN|HOME) +(\d*\.?\d+)", lines[3])), u.deg
        )

        self.move_code = int(parse_get(r"(?:RL|RR|--) +(\d+)", lines[4]))

        self.estop_active = bool(
            int(parse_get(r"Emergency Stop Active: +(\d)", lines[5]))
        )

        self.scb_link_ok = bool(int(parse_get(r"Top Comm Link OK: +(\d)", lines[6])))

        self.home_azimuth = Angle(
            float(parse_get(r"Home Azimuth: +(\d*\.?\d+)", lines[7])), u.deg
        )

        self.high_speed = Angle(
            float(parse_get(r"High Speed.+: +(\d*\.?\d+)", lines[8])), u.deg
        )

        self.coast = Angle(float(parse_get(r"Coast.+: +(\d*\.?\d+)", lines[9])), u.deg)

        self.tolerance = Angle(
            float(parse_get(r"Tolerance.+: +(\d*\.?\d+)", lines[10])), u.deg
        )

        self.encoder_counts_per_360 = int(
            parse_get(r"Encoder Counts per 360: +(\d+)", lines[11])
        )

        self.encoder_counts = int(parse_get(r"Encoder Counts: +(\d+)", lines[12]))

        self.last_azimuth_goto = float(
            parse_get(r"Last Azimuth GoTo: +(\d*\.?\d+)", lines[13])
        )

        self.azimuth_move_timeout = float(
            parse_get(r"Azimuth Move Timeout.+: +(\d*\.?\d+)", lines[14])
        )

        self.rain_sensor_enabled = bool(
            int(parse_get(r"Rain-Snow enabled: +(\d)", lines[15]))
        )

        self.cloud_sensor_enabled = bool(
            int(parse_get(r"Cloud Sensor enabled: +(\d)", lines[16]))
        )

        self.watchdog_timer = float(
            parse_get(r"Watchdog Reset Time: +(\d*\.?\d+)", lines[17])
        )

        self.dropout_timer = float(parse_get(r"Dropout Timer: +(\d*\.?\d+)", lines[18]))

        self.reversal_delay = float(
            parse_get(r"Reverse Delay: +(\d*\.?\d+)", lines[19])
        )

        self.main_door_encoder_closed = int(
            parse_get(r"Main Door Encoder Closed: +(\d+)", lines[20])
        )

        self.main_door_encoder_opened = int(
            parse_get(r"Main Door Encoder Opened: +(\d+)", lines[21])
        )

        self.dropout_door_encoder_closed = int(
            parse_get(r"Dropout Encoder Closed: +(\d+)", lines[22])
        )

        self.dropout_door_encoder_opened = int(
            parse_get(r"Dropout Encoder Opened: +(\d+)", lines[23])
        )

        self.door_move_timeout = float(
            parse_get(r"Door Move Timeout.+: +(\d*\.?\d+)", lines[24])
        )

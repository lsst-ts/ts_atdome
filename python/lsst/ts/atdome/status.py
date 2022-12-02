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
    """Parsed data of the output from "+", the full status command."""

    def __init__(self, lines):
        if len(lines) != 27:
            raise RuntimeError(f"Got {len(lines)} lines; need 27")

        lineiter = iter(lines)

        self.main_door_pct = float(parse_get(r"MAIN +[A-Z]+ +(\d+)", next(lineiter)))

        self.dropout_door_pct = float(parse_get(r"DROP +[A-Z]+ +(\d+)", next(lineiter)))

        auto_shutdown_match = parse(r"\[(ON|OFF)\] +(\d+)", next(lineiter))
        self.auto_shutdown_enabled = auto_shutdown_match.group(1) == "ON"
        self.sensor_code = int(auto_shutdown_match.group(2))

        az_pos_match = parse(r"(POSN|HOME) +(\d*\.?\d+)", next(lineiter))
        self.az_home_switch = az_pos_match.group(1) == "HOME"
        self.az_pos = float(az_pos_match.group(2))

        self.move_code = int(parse_get(r"(?:RL|RR|--) +(\d+)", next(lineiter)))

        self.homed = parse_get(r"Dome (not )?homed", next(lineiter)) is None

        self.estop_active = bool(
            int(parse_get(r"Emergency Stop Active: +(\d)", next(lineiter)))
        )

        self.scb_link_ok = bool(
            int(parse_get(r"Top Comm Link OK: +(\d)", next(lineiter)))
        )

        self.home_azimuth = float(
            parse_get(r"Home Azimuth: +(\d*\.?\d+)", next(lineiter))
        )

        self.high_speed = float(
            parse_get(r"High Speed.+: +(\d*\.?\d+)", next(lineiter))
        )

        self.coast = float(parse_get(r"Coast.+: +(\d*\.?\d+)", next(lineiter)))

        self.tolerance = float(parse_get(r"Tolerance.+: +(\d*\.?\d+)", next(lineiter)))

        self.encoder_counts_per_360 = int(
            parse_get(r"Encoder Counts per 360: +(\d+)", next(lineiter))
        )

        self.encoder_counts = int(parse_get(r"Encoder Counts: +(\d+)", next(lineiter)))

        self.last_azimuth_goto = float(
            parse_get(r"Last Azimuth GoTo: +(\d*\.?\d+)", next(lineiter))
        )

        self.azimuth_move_timeout = float(
            parse_get(r"Azimuth Move Timeout.+: +(\d*\.?\d+)", next(lineiter))
        )

        self.rain_sensor_enabled = bool(
            int(parse_get(r"Rain-Snow enabled: +(\d)", next(lineiter)))
        )

        self.cloud_sensor_enabled = bool(
            int(parse_get(r"Cloud Sensor enabled: +(\d)", next(lineiter)))
        )

        self.watchdog_timer = float(
            parse_get(r"Watchdog Reset Time: +(\d*\.?\d+)", next(lineiter))
        )

        self.dropout_timer = float(
            parse_get(r"Dropout Timer: +(\d*\.?\d+)", next(lineiter))
        )

        self.reversal_delay = float(
            parse_get(r"Reverse Delay: +(\d*\.?\d+)", next(lineiter))
        )

        self.main_door_encoder_closed = int(
            parse_get(r"Main Door Encoder Closed: +(\d+)", next(lineiter))
        )

        self.main_door_encoder_opened = int(
            parse_get(r"Main Door Encoder Opened: +(\d+)", next(lineiter))
        )

        self.dropout_door_encoder_closed = int(
            parse_get(r"Dropout Encoder Closed: +(\d+)", next(lineiter))
        )

        self.dropout_door_encoder_opened = int(
            parse_get(r"Dropout Encoder Opened: +(\d+)", next(lineiter))
        )

        self.door_move_timeout = float(
            parse_get(r"Door Move Timeout.+: +(\d*\.?\d+)", next(lineiter))
        )

        # Ignore the last line, as it is a second (redundant) way of reporting
        # whether the dome has been homed

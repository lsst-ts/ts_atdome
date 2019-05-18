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

__all__ = ["AzimuthCommandedState", "AzimuthState", "ShutterDoorCommandedState", "ShutterDoorState"]

import enum


class AzimuthCommandedState(enum.IntEnum):
    Unknown = 1
    GoToPosition = 2
    Home = 3
    Stop = 4


class AzimuthState(enum.IntEnum):
    NotInMotion = 1
    MovingCW = 2
    MovingCCW = 3


class ShutterDoorCommandedState(enum.IntEnum):
    Unknown = 1
    Closed = 2
    Opened = 3
    Stop = 4


class ShutterDoorState(enum.IntEnum):
    Closed = 1
    Opened = 2
    PartiallyOpened = 3
    Opening = 4
    Closing = 5

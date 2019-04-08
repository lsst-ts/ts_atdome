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

import SALPY_ATDome


class AzimuthCommandedState(enum.IntEnum):
    Unknown = SALPY_ATDome.ATDome_shared_AzimuthCommandedState_Unknown
    GoToPosition = SALPY_ATDome.ATDome_shared_AzimuthCommandedState_GoToPosition
    Home = SALPY_ATDome.ATDome_shared_AzimuthCommandedState_Home
    Stop = SALPY_ATDome.ATDome_shared_AzimuthCommandedState_Stop


class AzimuthState(enum.IntEnum):
    NotInMotion = SALPY_ATDome.ATDome_shared_AzimuthState_NotInMotion
    MovingCW = SALPY_ATDome.ATDome_shared_AzimuthState_MovingCW
    MovingCCW = SALPY_ATDome.ATDome_shared_AzimuthState_MovingCCW


class ShutterDoorCommandedState(enum.IntEnum):
    Unknown = SALPY_ATDome.ATDome_shared_ShutterDoorCommandedState_Unknown
    Closed = SALPY_ATDome.ATDome_shared_ShutterDoorCommandedState_Closed
    Opened = SALPY_ATDome.ATDome_shared_ShutterDoorCommandedState_Opened
    Stop = SALPY_ATDome.ATDome_shared_ShutterDoorCommandedState_Stop


class ShutterDoorState(enum.IntEnum):
    Closed = SALPY_ATDome.ATDome_shared_ShutterDoorState_Closed
    Opened = SALPY_ATDome.ATDome_shared_ShutterDoorState_Opened
    PartiallyOpened = SALPY_ATDome.ATDome_shared_ShutterDoorState_PartiallyOpened
    Opening = SALPY_ATDome.ATDome_shared_ShutterDoorState_Opening
    Closing = SALPY_ATDome.ATDome_shared_ShutterDoorState_Closing

#!/usr/bin/env python
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
import asyncio
import argparse

from lsst.ts import ATDome


def main():
    parser = argparse.ArgumentParser(f"Run ATDome")
    parser.add_argument("-s", "--simulate", action="store_true",
                        help="Run in simuation mode?")
    args = parser.parse_args()
    return ATDome.ATDomeCsc(initial_simulation_mode=args.simulate)


csc = main()
asyncio.get_event_loop().run_until_complete(csc.done_task)

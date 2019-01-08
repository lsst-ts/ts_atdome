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

__all__ = ["angle_diff", "assert_angles_almost_equal"]

from astropy.coordinates import Angle
import astropy.units as u

MIDDLE_WRAP_ANGLE = Angle(180, u.deg)


def angle_diff(angle1, angle2):
    """Return angle1 - angle2 wrapped into the range [-180, 180] deg.

    Parameters
    ----------
    angle1 : `astropy.coordinates.Angle` or `float`
        Angle 1; if a float then in degrees
    angle2 : `astropy.coordinates.Angle` or `float`
        Angle 2; if a float then in degrees

    Returns
    -------
    diff : `astropy.coordinates.Angle` or `float`
        angle1 - angle2 wrapped into the range [-180, 180] deg.
    """
    return (Angle(angle1, u.deg) - Angle(angle2, u.deg)).wrap_at(MIDDLE_WRAP_ANGLE)


def assert_angles_almost_equal(angle1, angle2, max_diff=1e-5):
    """Raise AssertionError if angle1 and angle2 are too different,
    ignoring wrap.

    Parameters
    ----------
    angle1 : `astropy.coordinates.Angle` or `float`
        Angle 1; if a float then in degrees
    angle2 : `astropy.coordinates.Angle` or `float`
        Angle 2; if a float then in degrees
    max_diff : `astropy.coordinates.Angle` or `float`
        Maximum allowed difference.

    Raises
    ------
    AssertionError
        If `angle_diff` of angle1 and angle2 exceeds max_diff.
    """
    diff = abs(angle_diff(angle1, angle2))
    if diff > Angle(max_diff, u.deg):
        raise AssertionError(f"{angle1} and {angle2} differ by {diff} > {max_diff}")

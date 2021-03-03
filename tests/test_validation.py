# This file is part of ts_ATDomeTrajectory.
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

import unittest

import jsonschema

from lsst.ts import salobj
from lsst.ts import ATDome


class ValidationTestCase(unittest.TestCase):
    """Test validation of the config schema."""

    def setUp(self):
        self.schema = ATDome.CONFIG_SCHEMA
        self.validator = salobj.DefaultingValidator(schema=self.schema)
        self.default = dict(
            host="192.168.223.14", port=17310, connection_timeout=10, read_timeout=10
        )

    def test_default(self):
        result = self.validator.validate(None)
        for field, expected_value in self.default.items():
            self.assertEqual(result[field], expected_value)

    def test_some_specified(self):
        data = dict(host="1.2.3.4", port=2345, connection_timeout=3.4, read_timeout=4.5)
        for field, value in data.items():
            one_field_data = {field: value}
            with self.subTest(one_field_data=one_field_data):
                result = self.validator.validate(one_field_data)
                for field, default_value in self.default.items():
                    if field in one_field_data:
                        self.assertEqual(result[field], one_field_data[field])
                    else:
                        self.assertEqual(result[field], default_value)

    def test_all_specified(self):
        data = dict(host="1.2.3.4", port=2345, connection_timeout=3.4, read_timeout=4.5)
        data_copy = data.copy()
        result = self.validator.validate(data)
        self.assertEqual(data, data_copy)
        for field, value in data.items():
            self.assertEqual(result[field], value)

    def test_invalid_configs(self):
        for name, badval in (
            #  ("host", "invalid hostname"),  # jsonschema 3.0.1 doesn't raise
            ("host", 5),  # wrong type
            ("port", "1234"),  # wrong type
            ("connection_timeout", 0),  # not positive
            ("read_timeout", 0),  # not positive
            ("connection_timeout", "1"),  # wrong type
            ("read_timeout", "1"),  # wrong type
        ):
            bad_data = {name: badval}
            with self.subTest(bad_data=bad_data):
                with self.assertRaises(jsonschema.exceptions.ValidationError):
                    self.validator.validate(bad_data)


if __name__ == "__main__":
    unittest.main()

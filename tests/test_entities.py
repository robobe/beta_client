"""Public entity behavior and MAVLink encoding compatibility."""
import unittest
from dataclasses import FrozenInstanceError, astuple
from beta_client.entities import Attitude, RcChannels, RcChannelsOverride
from pymavlink.dialects.v20 import common


class EntityTests(unittest.TestCase):
    def test_rc_validation_and_ownership(self):
        values = [1000] * 8
        entity = RcChannels(values)
        values[0] = 2000
        self.assertEqual(entity.values, (1000,) * 8)
        for invalid in ([0] * 7, [-1] * 8, [65536] * 8, [1.0] * 8, [True] * 8):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                RcChannels(invalid)
        self.assertEqual(RcChannels((0, 65535) * 4).values, (0, 65535) * 4)

    def test_immutable_payload(self):
        attitude = Attitude(100, 1, 2, 3, 4, 5, 6)
        with self.assertRaises(FrozenInstanceError):
            attitude.roll = 2
        message = common.MAVLink_attitude_message(*astuple(attitude))
        self.assertEqual(message.yawspeed, 6)

    def test_rc_override_defaults_ignore_extension_channels(self):
        payload = RcChannelsOverride(1, 1, *((1500,) * 8))
        message = common.MAVLink_rc_channels_override_message(*astuple(payload))
        self.assertEqual(message.chan8_raw, 1500)
        self.assertEqual(message.chan9_raw, 0)

from pathlib import Path
import sys
import unittest
from unittest.mock import patch

EXAMPLE = Path(__file__).resolve().parents[1] / 'examples' / 'msp_mavlink_flight'
with patch.object(sys, 'path', [str(EXAMPLE), *sys.path]):
    import altitude_control
    import msp_rc


class FlightRcTests(unittest.TestCase):
    @patch.object(msp_rc, 'TcpTransport')
    @patch.object(msp_rc, 'send_rc')
    def test_channel_mapping_drain_and_cleanup(self, send_rc, connect):
        sender = msp_rc.MspRcSender()
        for arm, aux in ((True, 2000), (False, 1000)):
            with self.subTest(arm=arm):
                sender.send(1350, arm)
                send_rc.assert_called_with(connect.return_value,
                                           [1500, 1500, 1350, 1500, aux, 2000, 1000, 1000])
        self.assertEqual(connect.return_value.drain_responses.call_count, 2)
        sender.close()
        connect.return_value.close.assert_called_once()
        connect.assert_called_once_with('127.0.0.1', 5761)

    def test_controller_matches_original_pwm_behavior(self):
        controller = altitude_control.AltitudeControl()
        self.assertEqual(controller.update(10, 0, 1), 1400)
        self.assertEqual(controller.update(10, 0, 1.1), 1554)
        controller.reset()
        controller.update(0, 10, 1, landing=True)
        self.assertEqual(controller.update(0, 10, 1.1, landing=True), 1349)

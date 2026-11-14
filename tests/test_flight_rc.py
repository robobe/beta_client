from beta_client.entities import RcChannels
import ast
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

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
                                           RcChannels((1500, 1500, 1350, 1500, aux, 2000, 1000, 1000)))
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


class FlightTelemetryTests(unittest.TestCase):
    def test_mission_updates_controller_only_for_new_sample(self):
        # Exercise the real mission branch without starting SITL or physics.
        tree = ast.parse((EXAMPLE / 'main.py').read_text())
        branch = next(node for node in ast.walk(tree) if isinstance(node, ast.If)
                      and ast.unparse(node.test) == 'sample.vfr_hud_at > last_sample')
        code = compile(ast.Module(body=[branch], type_ignores=[]), 'mission-sample', 'exec')
        controller = Mock(throttle=1400)
        controller.update.return_value = 1450
        context = dict(sample=SimpleNamespace(vfr_hud_at=1.0), last_sample=0.0,
                       controller=controller, target=10.0, alt=2.0, state='takeoff')
        exec(code, context)
        exec(code, context)
        controller.update.assert_called_once_with(10.0, 2.0, 1.0, landing=False)
        context['sample'] = SimpleNamespace(vfr_hud_at=2.0)
        exec(code, context)
        self.assertEqual(controller.update.call_count, 2)

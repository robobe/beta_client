from dataclasses import FrozenInstanceError
from queue import Empty, Queue
from types import SimpleNamespace
import threading
import time
import unittest
from unittest.mock import patch

from pymavlink import mavutil
from beta_client.mavlink import MavlinkTelemetry


def message(kind, source=(1, 1), **fields):
    return SimpleNamespace(get_type=lambda: kind, get_srcSystem=lambda: source[0],
                           get_srcComponent=lambda: source[1], **fields)


class Connection:
    def __init__(self, result=0, acknowledge=True):
        self.messages = Queue()
        self.messages.put(message('HEARTBEAT', type=2, base_mode=128))
        self.mav = self
        self.result = result
        self.acknowledge = acknowledge
        self.operations = []
        self.commands = []
        self.closed = threading.Event()

    def recv_match(self, **kwargs):
        self.operations.append(threading.get_ident())
        try:
            item = self.messages.get(timeout=kwargs['timeout'])
        except Empty:
            return None
        if isinstance(item, Exception):
            raise item
        return item

    def command_long_send(self, *args):
        self.operations.append(threading.get_ident())
        self.commands.append(args)
        # Samples arriving during setup must survive acknowledgement handling.
        self.messages.put(message('VFR_HUD', alt=4.5))
        if self.acknowledge:
            self.messages.put(message('COMMAND_ACK', command=args[2], result=self.result))

    def close(self):
        self.operations.append(threading.get_ident())
        self.closed.set()


class MavlinkTests(unittest.TestCase):
    def wait_for(self, predicate):
        deadline = time.monotonic() + 1
        while not predicate():
            if time.monotonic() >= deadline:
                self.fail('Worker did not publish expected state')
            time.sleep(0.001)

    def test_snapshot_setup_filtering_and_worker_ownership(self):
        connection = Connection()
        created_on = []
        def create(*args, **kwargs):
            created_on.append(threading.get_ident())
            return connection
        with patch('beta_client.mavlink.mavutil.mavlink_connection', side_effect=create):
            with MavlinkTelemetry('tcp:localhost:5762') as telemetry:
                first = telemetry.snapshot()
                self.assertEqual(first.altitude, 4.5)
                self.assertTrue(first.armed)
                with self.assertRaises(FrozenInstanceError):
                    first.altitude = 10
                connection.messages.put(message('VFR_HUD', source=(2, 1), alt=99))
                connection.messages.put(message('ATTITUDE', roll=1, pitch=2, yaw=3))
                self.wait_for(lambda: telemetry.snapshot().attitude is not None)
                sample = telemetry.snapshot()
                self.assertEqual(sample.altitude, 4.5)
                self.assertEqual(sample.attitude, (1, 2, 3))
                self.assertGreater(sample.altitude_at, 0)
                self.assertGreater(sample.attitude_at, 0)
                self.assertIsNone(first.attitude)
                self.assertEqual([args[4] for args in connection.commands], [74, 30])
                self.assertTrue(all(args[:2] == (1, 1) and args[5] == 20000
                                    for args in connection.commands))
            self.assertTrue(connection.closed.is_set())
            self.assertEqual(set(connection.operations), set(created_on))
            self.assertNotEqual(created_on[0], threading.get_ident())

    def test_setup_failure_closes_connection(self):
        for result, acknowledge in ((2, True), (0, False)):
            with self.subTest(result=result, acknowledge=acknowledge):
                connection = Connection(result, acknowledge)
                with patch('beta_client.mavlink.mavutil.mavlink_connection', return_value=connection), \
                        patch('beta_client.mavlink.ACK_TIMEOUT', 0.01):
                    with self.assertRaises(RuntimeError):
                        MavlinkTelemetry('/dev/test').start()
                self.assertTrue(connection.closed.is_set())

    def test_missing_heartbeat(self):
        connection = Connection()
        connection.messages.get_nowait()
        connection.messages.put(message('HEARTBEAT', type=mavutil.mavlink.MAV_TYPE_GCS, base_mode=0))
        with patch('beta_client.mavlink.mavutil.mavlink_connection', return_value=connection), \
                patch('beta_client.mavlink.HEARTBEAT_TIMEOUT', 0.01):
            with self.assertRaises(RuntimeError) as caught:
                MavlinkTelemetry('/dev/test').start()
        self.assertIsInstance(caught.exception.__cause__, TimeoutError)
        self.assertTrue(connection.closed.is_set())

    def test_worker_error_surfaces_on_snapshot(self):
        connection = Connection()
        error = OSError('disconnected')
        with patch('beta_client.mavlink.mavutil.mavlink_connection', return_value=connection):
            telemetry = MavlinkTelemetry('/dev/test').start()
            connection.messages.put(error)
            self.assertTrue(connection.closed.wait(1))
            with self.assertRaises(RuntimeError) as caught:
                telemetry.snapshot()
            self.assertIs(caught.exception.__cause__, error)
            telemetry.close()

    def test_measured_rate_and_snapshot_has_no_io(self):
        connection = Connection()
        with patch('beta_client.mavlink.mavutil.mavlink_connection', return_value=connection):
            with MavlinkTelemetry('/dev/test') as telemetry:
                # Rates use counts over actual elapsed time, independently of requested rate.
                with patch('beta_client.mavlink.RATE_WINDOW', 0):
                    connection.messages.put(message('VFR_HUD', alt=8))
                    self.wait_for(lambda: telemetry.snapshot().altitude == 8)
                    self.assertGreater(telemetry.snapshot().vfr_hud_hz, 0)
                # Only the worker invokes the connection, including while snapshots are read.
                for _ in range(100):
                    telemetry.snapshot()
                self.assertNotIn(threading.get_ident(), connection.operations)

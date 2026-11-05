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
        # Samples arriving during a rate request must survive ACK handling.
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
                self.assertEqual(connection.commands, [])
                self.assertIsNone(telemetry.snapshot().altitude)
                telemetry.set_message_rate(mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 50)
                telemetry.set_message_rate(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 50)
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

    def test_rate_rejection_and_timeout(self):
        for result, acknowledge in ((2, True), (0, False)):
            with self.subTest(result=result, acknowledge=acknowledge):
                connection = Connection(result, acknowledge)
                with patch('beta_client.mavlink.mavutil.mavlink_connection', return_value=connection):
                    with MavlinkTelemetry('/dev/test') as telemetry:
                        expected = RuntimeError if acknowledge else TimeoutError
                        with self.assertRaises(expected):
                            telemetry.set_message_rate(74, 50, timeout=0.02)
                        if acknowledge:
                            # A terminal rejection leaves the link usable for another request.
                            connection.result = 0
                            telemetry.set_message_rate(74, 25)
                        else:
                            with self.assertRaises(RuntimeError):
                                telemetry.snapshot()
                            with self.assertRaises(RuntimeError):
                                telemetry.set_message_rate(30, 50)
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


class RateRequestTests(unittest.TestCase):
    def launch_request(self, telemetry, message_id=74, timeout=0.5):
        outcome = Queue()
        def request():
            try:
                telemetry.set_message_rate(message_id, 50, timeout=timeout)
                outcome.put(None)
            except Exception as error:
                outcome.put(error)
        thread = threading.Thread(target=request)
        thread.start()
        self.addCleanup(thread.join, 2)
        return outcome

    def await_commands(self, connection, count):
        deadline = time.monotonic() + 1
        while len(connection.commands) < count:
            if time.monotonic() >= deadline:
                self.fail('Worker did not send rate request')
            time.sleep(0.001)

    def ack(self, connection, source=(1, 1), command=None, result=0):
        connection.messages.put(message('COMMAND_ACK', source=source,
            command=command if command is not None else mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            result=result))

    def test_unrelated_and_progress_acks_do_not_complete_request(self):
        connection = Connection(acknowledge=False)
        with patch('beta_client.mavlink.mavutil.mavlink_connection', return_value=connection):
            with MavlinkTelemetry('/dev/test') as telemetry:
                outcome = self.launch_request(telemetry)
                self.await_commands(connection, 1)
                self.ack(connection, source=(2, 1))
                self.ack(connection, source=(1, 2))
                self.ack(connection, command=999)
                self.ack(connection, result=mavutil.mavlink.MAV_RESULT_IN_PROGRESS)
                connection.messages.put(message('ATTITUDE', roll=1, pitch=2, yaw=3))
                deadline = time.monotonic() + 1
                while telemetry.snapshot().attitude is None:
                    self.assertLess(time.monotonic(), deadline)
                    time.sleep(0.001)
                self.assertTrue(outcome.empty())
                self.ack(connection)
                self.assertIsNone(outcome.get(timeout=1))

    def test_progress_ack_does_not_extend_timeout(self):
        connection = Connection(acknowledge=False)
        with patch('beta_client.mavlink.mavutil.mavlink_connection', return_value=connection):
            with MavlinkTelemetry('/dev/test') as telemetry:
                outcome = self.launch_request(telemetry, timeout=0.02)
                self.await_commands(connection, 1)
                self.ack(connection, result=mavutil.mavlink.MAV_RESULT_IN_PROGRESS)
                self.assertIsInstance(outcome.get(timeout=1), TimeoutError)
                self.assertTrue(connection.closed.wait(1))

    def test_concurrent_requests_are_serialized(self):
        connection = Connection(acknowledge=False)
        with patch('beta_client.mavlink.mavutil.mavlink_connection', return_value=connection):
            with MavlinkTelemetry('/dev/test') as telemetry:
                first = self.launch_request(telemetry, 74)
                self.await_commands(connection, 1)
                second = self.launch_request(telemetry, 30)
                # Feed unrelated data and confirm no second command appears before the ACK.
                connection.messages.put(message('ATTITUDE', roll=1, pitch=2, yaw=3))
                self.assertEqual(len(connection.commands), 1)
                self.ack(connection)
                self.assertIsNone(first.get(timeout=1))
                self.await_commands(connection, 2)
                self.assertEqual([cmd[4] for cmd in connection.commands], [74, 30])
                self.ack(connection)
                self.assertIsNone(second.get(timeout=1))

    def test_shutdown_and_worker_failure_release_waiter(self):
        for fail in (True, False):
            with self.subTest(fail=fail):
                connection = Connection(acknowledge=False)
                with patch('beta_client.mavlink.mavutil.mavlink_connection', return_value=connection):
                    with MavlinkTelemetry('/dev/test') as telemetry:
                        outcome = self.launch_request(telemetry)
                        self.await_commands(connection, 1)
                        if fail:
                            connection.messages.put(OSError('disconnected'))
                        else:
                            telemetry.close()
                        self.assertIsInstance(outcome.get(timeout=1), OSError if fail else RuntimeError)
                    self.assertTrue(connection.closed.is_set())

    def test_invalid_arguments_and_lifecycle(self):
        telemetry = MavlinkTelemetry('/dev/test')
        for args in ((-1, 50), (0x1000000, 50), (74, 0), (74, -1),
                     (74, float('nan')), (74, float('inf')), (74, 1e20), (74, 1e-20)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                telemetry.set_message_rate(*args)
        for timeout in (0, -1, float('inf'), float('nan')):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                telemetry.set_message_rate(74, 50, timeout=timeout)
        with self.assertRaises(RuntimeError):
            telemetry.set_message_rate(74, 50)
        connection = Connection()
        with patch('beta_client.mavlink.mavutil.mavlink_connection', return_value=connection):
            with telemetry:
                telemetry.set_message_rate(74, 30)
                self.assertEqual(connection.commands[0][5], 33333)
            with self.assertRaises(RuntimeError):
                telemetry.set_message_rate(74, 50)

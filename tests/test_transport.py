import socket
import unittest
from unittest.mock import patch

import serial

from beta_client.transport import DEFAULT_WRITE_TIMEOUT, SerialTransport, TcpTransport


class TransportTests(unittest.TestCase):
    def test_tcp_drains_without_waiting_and_preserves_timeout(self):
        connection, peer = socket.socketpair()
        with peer, patch('beta_client.transport.socket.create_connection', return_value=connection):
            with TcpTransport('localhost', 5761) as transport:
                transport.drain_responses()
                peer.sendall(b'reply')
                transport.drain_responses()
                connection.setblocking(False)
                with self.assertRaises(BlockingIOError):
                    connection.recv(1)
                connection.settimeout(DEFAULT_WRITE_TIMEOUT)
                transport.drain_responses()
                self.assertEqual(connection.gettimeout(), DEFAULT_WRITE_TIMEOUT)

    def test_tcp_closed_peer_raises_and_restores_timeout(self):
        connection, peer = socket.socketpair()
        peer.close()
        with patch('beta_client.transport.socket.create_connection', return_value=connection):
            with TcpTransport('localhost', 5761) as transport:
                with self.assertRaises(ConnectionError):
                    transport.drain_responses()
                self.assertEqual(connection.gettimeout(), DEFAULT_WRITE_TIMEOUT)

    @patch('beta_client.transport.serial.Serial')
    def test_serial_uses_one_deadline(self, open_serial):
        connection = open_serial.return_value
        connection.write_timeout = DEFAULT_WRITE_TIMEOUT
        connection.write.return_value = 1
        with SerialTransport('/dev/test', 115200) as transport:
            with patch('beta_client.transport.time.monotonic', side_effect=[0, 0.005, 0.021]):
                with self.assertRaises(serial.SerialTimeoutException):
                    transport.send(b'ab')
            connection.write.assert_called_once()
            self.assertEqual(connection.write_timeout, DEFAULT_WRITE_TIMEOUT)
        connection.close.assert_called_once()

    @patch('beta_client.transport.serial.Serial')
    def test_serial_drain_is_bounded_and_nonblocking(self, open_serial):
        with SerialTransport('/dev/test', 115200) as transport:
            open_serial.return_value.in_waiting = 10000
            transport.drain_responses()
        open_serial.return_value.read.assert_called_once_with(4096)
        self.assertEqual(open_serial.call_args.kwargs['timeout'], 0)

    def test_rejects_nonpositive_timeout_before_connecting(self):
        with patch('beta_client.transport.socket.create_connection') as connect:
            with self.assertRaises(ValueError):
                TcpTransport('localhost', 5761, timeout=0)
            connect.assert_not_called()
        with patch('beta_client.transport.serial.Serial') as connect:
            with self.assertRaises(ValueError):
                SerialTransport('/dev/test', 115200, timeout=-1)
            connect.assert_not_called()

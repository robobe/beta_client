import unittest
from unittest.mock import Mock, patch

from beta_client.entities import RcChannels
from beta_client.msp import build_rc_message, send_rc
from beta_client.transport import SerialTransport, TcpTransport


class MspTests(unittest.TestCase):
    def test_frame(self):
        self.assertEqual(build_rc_message(RcChannels((1000,) * 8)), b'$M<\x10\xc8' + b'\xe8\x03' * 8 + b'\xd8')

    def test_invalid_channels(self):
        transport = Mock()
        for channels in ([1000] * 7, [1000] * 7 + [-1], [1000] * 7 + [65536]):
            with self.subTest(channels=channels), self.assertRaises(ValueError):
                send_rc(transport, RcChannels(channels))
        transport.send.assert_not_called()

    @patch('beta_client.transport.socket.create_connection')
    def test_tcp(self, connect):
        with TcpTransport('localhost', 5761) as transport:
            send_rc(transport, RcChannels((1000,) * 8))
        connect.return_value.sendall.assert_called_once_with(build_rc_message(RcChannels((1000,) * 8)))
        connect.return_value.close.assert_called_once()

    @patch('beta_client.transport.serial.Serial')
    def test_serial_partial_writes_and_close(self, serial):
        serial.return_value.write_timeout = 0.02
        serial.return_value.write.side_effect = [1, 2]
        with SerialTransport('/dev/test', 115200) as transport:
            transport.send(b'abc')
        self.assertEqual(bytes(serial.return_value.write.call_args_list[1].args[0]), b'bc')
        serial.return_value.close.assert_called_once()

    @patch('beta_client.transport.serial.Serial')
    def test_serial_no_progress(self, serial):
        serial.return_value.write_timeout = 0.02
        serial.return_value.write.return_value = 0
        with SerialTransport('/dev/test', 115200) as transport, self.assertRaises(OSError):
            transport.send(b'a')
        serial.return_value.close.assert_called_once()

    def test_send_error_propagates(self):
        transport = Mock()
        transport.send.side_effect = OSError('disconnected')
        with self.assertRaises(OSError):
            send_rc(transport, RcChannels((1000,) * 8))


if __name__ == '__main__':
    unittest.main()

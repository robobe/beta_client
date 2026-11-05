"""Send MSP v1 RC frames to local SITL."""
import socket
import struct


class MspRcSender:
    def __init__(self):
        self.socket = socket.create_connection(('127.0.0.1', 5761), timeout=3)

    def send(self, throttle, arm):
        payload = struct.pack('<8H', 1500, 1500, throttle, 1500, 2000 if arm else 1000, 2000, 1000, 1000)
        body = bytes((len(payload), 200)) + payload
        checksum = 0
        for byte in body:
            checksum ^= byte
        self.socket.sendall(b'$M<' + body + bytes((checksum,)))
        # Consume responses so repeated acknowledgements cannot fill the socket.
        self.socket.setblocking(False)
        try:
            while self.socket.recv(4096):
                pass
        except BlockingIOError:
            pass
        finally:
            self.socket.settimeout(3)

    def close(self):
        self.socket.close()

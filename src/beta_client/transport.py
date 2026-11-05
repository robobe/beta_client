"""Blocking TCP and serial writes with optional response draining."""

import socket
import time
from typing import Protocol, Self

import serial

# One 50 Hz control period, callers with a tighter budget should lower this.
DEFAULT_WRITE_TIMEOUT = 0.02
# Connection establishment happens before the control loop starts.
DEFAULT_CONNECT_TIMEOUT = 3.0
# Bound each drain so incoming traffic cannot monopolize the control loop.
DRAIN_BUFFER_SIZE = 4096


class Transport(Protocol):
    def send(self, message: bytes) -> None: ...


class TcpTransport:
    def __init__(self, host: str, port: int, timeout: float = DEFAULT_WRITE_TIMEOUT,
                 *, connect_timeout: float = DEFAULT_CONNECT_TIMEOUT):
        if timeout <= 0:
            raise ValueError('Write timeout must be positive')
        self.connection = socket.create_connection((host, port), timeout=connect_timeout)
        self.connection.settimeout(timeout)

    def send(self, message: bytes) -> None:
        """Send a complete message, raising on write timeout or connection failure."""
        self.connection.sendall(message)

    def drain_responses(self) -> None:
        """Discard up to one buffer of replies without waiting for incoming data."""
        timeout = self.connection.gettimeout()
        self.connection.setblocking(False)
        try:
            if not self.connection.recv(DRAIN_BUFFER_SIZE):
                raise ConnectionError('TCP peer closed the connection')
        except BlockingIOError:
            pass
        finally:
            self.connection.settimeout(timeout)

    def close(self) -> None:
        self.connection.close()

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *args) -> None:
        self.close()


class SerialTransport:
    def __init__(self, device: str, baudrate: int, timeout: float = DEFAULT_WRITE_TIMEOUT):
        if timeout <= 0:
            raise ValueError('Write timeout must be positive')
        self.connection = serial.Serial(device, baudrate, timeout=0, write_timeout=timeout)

    def send(self, message: bytes) -> None:
        """Send a complete message within one budget, including partial writes."""
        timeout = self.connection.write_timeout
        deadline = time.monotonic() + timeout
        remaining = memoryview(message)
        try:
            while remaining:
                budget = deadline - time.monotonic()
                if budget <= 0:
                    raise serial.SerialTimeoutException('Serial write timed out')
                self.connection.write_timeout = budget
                written = self.connection.write(remaining)
                if written is None or written <= 0:
                    raise OSError('Serial write made no progress')
                remaining = remaining[written:]
        finally:
            self.connection.write_timeout = timeout

    def drain_responses(self) -> None:
        """Discard currently buffered replies, without waiting for more data."""
        self.connection.read(min(self.connection.in_waiting, DRAIN_BUFFER_SIZE))

    def close(self) -> None:
        self.connection.close()

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *args) -> None:
        self.close()

"""Receive MAVLink telemetry in one worker and expose consistent snapshots."""

from dataclasses import dataclass, field, replace
import math
import threading
import time
from typing import Self

from pymavlink import mavutil

DEFAULT_BAUD = 57600
# Allow firmware startup and a bounded wait for each rate acknowledgement.
HEARTBEAT_TIMEOUT = 10.0
ACK_TIMEOUT = 5.0
# Wake periodically to observe shutdown even when telemetry is silent.
RECEIVE_WAIT = 0.1
RATE_WINDOW = 1.0
MICROSECONDS_PER_SECOND = 1_000_000
MAX_MESSAGE_ID = 0xFFFFFF
MAX_INTERVAL_US = 0x7FFFFFFF


@dataclass(frozen=True)
class TelemetrySnapshot:
    altitude: float | None = None
    attitude: tuple[float, float, float] | None = None
    armed: bool | None = None
    altitude_at: float = 0.0
    attitude_at: float = 0.0
    heartbeat_at: float = 0.0
    vfr_hud_hz: float = 0.0
    attitude_hz: float = 0.0


@dataclass
class _RateRequest:
    message_id: int
    interval_us: int
    timeout: float
    done: threading.Event = field(default_factory=threading.Event)
    error: Exception | None = None


class MavlinkTelemetry:
    def __init__(self, endpoint: str, baud: int = DEFAULT_BAUD):
        self._endpoint = endpoint
        self._baud = baud
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread: threading.Thread | None = None
        self._error: Exception | None = None
        self._snapshot = TelemetrySnapshot()
        self._request_lock = threading.Lock()
        self._pending: _RateRequest | None = None

    def start(self) -> Self:
        """Start once and wait for a vehicle heartbeat."""
        if self._thread is not None:
            raise RuntimeError('Telemetry has already been started')
        if self._stop.is_set():
            raise RuntimeError('Telemetry is closed')
        self._thread = threading.Thread(target=self._run, name='mavlink-telemetry')
        self._thread.start()
        self._ready.wait()
        try:
            self.snapshot()
        except Exception:
            self.close()
            raise
        return self

    def set_message_rate(self, message_id: int, rate_hz: float, timeout: float = ACK_TIMEOUT) -> None:
        """Wait for a rate ACK, raising on rejection, timeout or worker failure.

        Concurrent requests are serialized. The ACK timeout starts when the worker
        sends the command, excluding time spent waiting behind another caller.
        A timeout fails the connection because a late ACK cannot be correlated
        with a subsequent SET_MESSAGE_INTERVAL command.
        """
        if type(message_id) is not int or not 0 <= message_id <= MAX_MESSAGE_ID:
            raise ValueError('Expected a valid MAVLink message ID')
        if not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError('Rate must be finite and positive')
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Timeout must be finite and positive')
        interval = MICROSECONDS_PER_SECOND / rate_hz
        if not math.isfinite(interval) or not 1 <= interval <= MAX_INTERVAL_US:
            raise ValueError('Rate cannot be represented as a positive microsecond interval')
        request = _RateRequest(message_id, round(interval), timeout)
        with self._request_lock:
            with self._lock:
                if self._error is not None:
                    raise RuntimeError('MAVLink telemetry failed') from self._error
                if not self._ready.is_set() or self._stop.is_set():
                    raise RuntimeError('Telemetry must be started and open')
                self._pending = request
            request.done.wait()
            if request.error is not None:
                raise request.error

    def snapshot(self) -> TelemetrySnapshot:
        """Read without I/O, raising if the worker has failed."""
        with self._lock:
            if self._error is not None:
                raise RuntimeError('MAVLink telemetry failed') from self._error
            if not self._ready.is_set():
                raise RuntimeError('Telemetry is not ready, call start first')
            return self._snapshot

    def close(self) -> None:
        """Stop reception and wait for the worker to release its connection."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join()

    def __enter__(self) -> Self:
        return self.start()

    def __exit__(self, *args) -> None:
        self.close()

    def _run(self) -> None:
        connection = None
        try:
            connection = mavutil.mavlink_connection(
                self._endpoint, baud=self._baud, autoreconnect=False, retries=1)
            source = None
            counts = {'VFR_HUD': 0, 'ATTITUDE': 0}
            rate_started = time.monotonic()

            def receive(timeout=RECEIVE_WAIT):
                nonlocal source, rate_started
                if self._stop.is_set():
                    raise InterruptedError('Telemetry stopped')
                message = connection.recv_match(blocking=True, timeout=timeout)
                now = time.monotonic()
                updates = {}
                if message is not None:
                    identity = (message.get_srcSystem(), message.get_srcComponent())
                    kind = message.get_type()
                    if source is None and kind == 'HEARTBEAT' and message.type != mavutil.mavlink.MAV_TYPE_GCS:
                        source = identity
                    if identity != source:
                        message = None
                    elif kind == 'HEARTBEAT':
                        updates.update(armed=bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                                       heartbeat_at=now)
                    elif kind == 'VFR_HUD':
                        counts[kind] += 1
                        updates.update(altitude=message.alt, altitude_at=now)
                    elif kind == 'ATTITUDE':
                        counts[kind] += 1
                        updates.update(attitude=(message.roll, message.pitch, message.yaw), attitude_at=now)
                elapsed = now - rate_started
                if elapsed >= RATE_WINDOW:
                    updates.update(vfr_hud_hz=counts['VFR_HUD'] / elapsed,
                                   attitude_hz=counts['ATTITUDE'] / elapsed)
                    counts['VFR_HUD'] = counts['ATTITUDE'] = 0
                    rate_started = now
                if updates:
                    with self._lock:
                        self._snapshot = replace(self._snapshot, **updates)
                return message

            deadline = time.monotonic() + HEARTBEAT_TIMEOUT
            while source is None:
                if time.monotonic() >= deadline:
                    raise TimeoutError('No MAVLink vehicle heartbeat')
                receive()
            self._ready.set()
            while not self._stop.is_set():
                with self._lock:
                    request = self._pending
                if request is None:
                    receive()
                    continue
                connection.mav.command_long_send(
                    *source, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    request.message_id, request.interval_us, 0, 0, 0, 0, 0)
                deadline = time.monotonic() + request.timeout
                while True:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError(f'No rate acknowledgement for message {request.message_id}')
                    message = receive(min(RECEIVE_WAIT, remaining))
                    if time.monotonic() >= deadline:
                        raise TimeoutError(f'No rate acknowledgement for message {request.message_id}')
                    if (message is None or message.get_type() != 'COMMAND_ACK'
                            or message.command != mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL):
                        continue
                    if message.result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                        continue
                    if message.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        request.error = RuntimeError(
                            f'Rate request for message {request.message_id} rejected: {message.result}')
                    with self._lock:
                        self._pending = None
                    request.done.set()
                    break
        except Exception as error:
            if not (self._stop.is_set() and isinstance(error, InterruptedError)):
                with self._lock:
                    self._error = error
        finally:
            try:
                if connection is not None:
                    connection.close()
            except Exception as error:
                with self._lock:
                    if self._error is None:
                        self._error = error
            self._ready.set()
            # Release an active caller even if receiving, sending or closing failed.
            with self._lock:
                if self._pending is not None:
                    self._pending.error = self._error or RuntimeError('Telemetry closed during rate request')
                    self._pending.done.set()
                    self._pending = None

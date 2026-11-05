"""Read the armed state and altitude exclusively from MAVLink."""
import time
from pymavlink import mavutil


class MavlinkTelemetry:
    def __init__(self):
        self.connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
        if not self.connection.wait_heartbeat(timeout=10):
            raise TimeoutError('No MAVLink heartbeat')
        self.armed = False
        self.altitude = None
        self.heartbeat_at = self.altitude_at = 0.0
        self.attitude = None
        self.counts = {'VFR_HUD': 0, 'ATTITUDE': 0}
        self.rate_started = time.monotonic()
        for name, message_id in (('VFR_HUD', mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD),
                                 ('ATTITUDE', mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE)):
            self.connection.mav.command_long_send(self.connection.target_system, self.connection.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, message_id, 20000, 0, 0, 0, 0, 0)
            deadline = time.monotonic() + 5
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f'No interval acknowledgement for {name}')
                ack = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=remaining)
                if ack is None:
                    raise TimeoutError(f'No interval acknowledgement for {name}')
                if ack.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
                    if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        raise RuntimeError(f'Interval request for {name} rejected: {ack.result}')
                    print(f'{name}: 50 Hz requested and accepted', flush=True)
                    break

    def poll(self):
        while message := self.connection.recv_match(blocking=False):
            now = time.monotonic()
            if message.get_type() == 'HEARTBEAT':
                self.armed = bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.heartbeat_at = now
            elif message.get_type() == 'ATTITUDE':
                self.counts['ATTITUDE'] += 1
                self.attitude = (message.roll, message.pitch, message.yaw)
            elif message.get_type() == 'VFR_HUD':
                self.counts['VFR_HUD'] += 1
                self.altitude = message.alt
                self.altitude_at = now

    def rates(self):
        now = time.monotonic()
        elapsed = now - self.rate_started
        rates = {name: count / elapsed for name, count in self.counts.items()}
        self.counts = dict.fromkeys(self.counts, 0)
        self.rate_started = now
        return rates

    def close(self):
        self.connection.close()

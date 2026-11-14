"""Read the armed state and altitude exclusively from MAVLink."""
import time
from dataclasses import replace, astuple
from beta_client.entities import TelemetrySnapshot, Heartbeat, Attitude, VfrHud, RcChannels, RcChannelsOverride
from beta_client.mavlink import decode
from pymavlink import mavutil


class MavlinkTelemetry:
    def __init__(self):
        self.connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
        if not self.connection.wait_heartbeat(timeout=10):
            raise TimeoutError('No MAVLink heartbeat')
        self._snapshot = TelemetrySnapshot()
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
                self._snapshot = replace(self._snapshot, heartbeat=decode(Heartbeat, message), heartbeat_at=now)
            elif message.get_type() == 'ATTITUDE':
                self.counts['ATTITUDE'] += 1
                self._snapshot = replace(self._snapshot, attitude=decode(Attitude, message), attitude_at=now)
            elif message.get_type() == 'VFR_HUD':
                self.counts['VFR_HUD'] += 1
                self._snapshot = replace(self._snapshot, vfr_hud=decode(VfrHud, message), vfr_hud_at=now)

    def rates(self):
        now = time.monotonic()
        elapsed = now - self.rate_started
        rates = {name: count / elapsed for name, count in self.counts.items()}
        self.counts = dict.fromkeys(self.counts, 0)
        self.rate_started = now
        return rates

    def snapshot(self) -> TelemetrySnapshot:
        return self._snapshot

    def send(self, throttle, arm):
        channels = RcChannels((1500, 1500, throttle, 1500, 2000 if arm else 1000, 2000, 1000, 1000))
        payload = RcChannelsOverride(self.connection.target_system, self.connection.target_component,
                                     *channels.values)
        self.connection.mav.rc_channels_override_send(*astuple(payload)[:10])

    def close(self):
        self.connection.close()

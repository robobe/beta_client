"""Request ATTITUDE and VFR_HUD at 50 Hz and measure reception for 60 seconds."""

import argparse
import time

from pymavlink import mavutil

DEFAULT_BAUD = 57600
HEARTBEAT_TIMEOUT_S = 10
ACK_TIMEOUT_S = 5
REQUESTED_RATE_HZ = 50
MICROSECONDS_PER_SECOND = 1_000_000
MESSAGE_INTERVAL_US = MICROSECONDS_PER_SECOND // REQUESTED_RATE_HZ
DURATION_S = 60

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("endpoint", help="TCP endpoint or serial device, e.g. /dev/ttyUSB0")
parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Serial baud rate (default: {DEFAULT_BAUD})")
args = parser.parse_args()
connection = mavutil.mavlink_connection(args.endpoint, baud=args.baud)

try:
    heartbeat = connection.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_S)
    if heartbeat is None:
        raise SystemExit(f"No heartbeat received within {HEARTBEAT_TIMEOUT_S} seconds")
    print(f"MAVLink connected: heartbeat from system {heartbeat.get_srcSystem()}, "
          f"component {heartbeat.get_srcComponent()}", flush=True)

    for name, message_id in (("ATTITUDE", mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE),
                             ("VFR_HUD", mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD)):
        connection.mav.command_long_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, MESSAGE_INTERVAL_US, 0, 0, 0, 0, 0,
        )
        deadline = time.monotonic() + ACK_TIMEOUT_S
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise SystemExit(f"No interval acknowledgement for {name}")
            ack = connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=remaining)
            if ack is None:
                raise SystemExit(f"No interval acknowledgement for {name}")
            if ack.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
                if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    raise SystemExit(f"Interval request for {name} rejected: result {ack.result}")
                print(f"Requested {name} at {REQUESTED_RATE_HZ} Hz")
                break

    counts = {"ATTITUDE": 0, "VFR_HUD": 0}
    start = time.monotonic()
    last = start
    for second in range(1, DURATION_S + 1):
        deadline = start + second
        while (remaining := deadline - time.monotonic()) > 0:
            message = connection.recv_match(type=list(counts), blocking=True, timeout=remaining)
            if message is not None:
                counts[message.get_type()] += 1
        now = time.monotonic()
        elapsed = now - last
        print(f"{second:2d}s: ATTITUDE {counts['ATTITUDE'] / elapsed:.1f} Hz, "
              f"VFR_HUD {counts['VFR_HUD'] / elapsed:.1f} Hz", flush=True)
        counts = dict.fromkeys(counts, 0)
        last = now
finally:
    connection.close()

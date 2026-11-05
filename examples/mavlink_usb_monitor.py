"""Print MAVLink messages and per-type receive rates from a serial adapter."""
import argparse
from collections import Counter
import sys
import time

from pymavlink import mavutil

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('device', nargs='?', default='/dev/ttyUSB0', help='Serial device (default: /dev/ttyUSB0)')
parser.add_argument('--baud', type=int, default=57600, help='Serial baud rate (default: 57600)')
args = parser.parse_args()

connection = mavutil.mavlink_connection(args.device, baud=args.baud)
colors = ('\033[36m', '\033[32m', '\033[33m', '\033[35m', '\033[34m')
assigned = {}
counts = Counter()
started = time.monotonic()
try:
    print(f'Listening on {args.device} at {args.baud} baud. Press Ctrl+C to stop.', flush=True)
    while True:
        remaining = max(0, 1 - (time.monotonic() - started))
        message = connection.recv_match(blocking=True, timeout=remaining)
        if message is not None:
            name = message.get_type()
            counts[name] += 1
            if sys.stdout.isatty():
                color = assigned.setdefault(name, colors[len(assigned) % len(colors)])
                print(f'{color}{message}\033[0m', flush=True)
            else:
                print(message, flush=True)
        now = time.monotonic()
        elapsed = now - started
        if elapsed >= 1:
            rates = ', '.join(f'{name}: {count / elapsed:.1f} Hz' for name, count in sorted(counts.items()))
            print(f'Rates: {rates or "no messages"}', flush=True)
            counts.clear()
            started = now
except KeyboardInterrupt:
    pass
finally:
    connection.close()

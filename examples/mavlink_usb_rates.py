"""Measure MAVLink telemetry rates from a USB serial adapter."""
import argparse
from pathlib import Path
import runpy
import sys

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('device', nargs='?', default='/dev/ttyUSB0', help='Serial device (default: /dev/ttyUSB0)')
parser.add_argument('--baud', type=int, default=57600, help='Serial baud rate (default: 57600)')
args = parser.parse_args()

# Reuse the same rate measurement as the TCP example.
sys.argv = [str(Path(__file__).with_name('mavlink_rates.py')), args.device, '--baud', str(args.baud)]
runpy.run_path(sys.argv[0], run_name='__main__')

"""Read one heartbeat from a MAVLink-enabled Betaflight SITL UART."""

import argparse

from pymavlink import mavutil

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("endpoint", help="TCP endpoint, for example tcp:127.0.0.1:5762")
args = parser.parse_args()

connection = mavutil.mavlink_connection(args.endpoint)
try:
    # Bound the wait so a wrong endpoint or inactive telemetry does not hang forever.
    heartbeat = connection.wait_heartbeat(timeout=10)
    if heartbeat is None:
        raise SystemExit("No heartbeat received within 10 seconds; check MAVLink configuration.")
    print(f"Heartbeat from system {heartbeat.get_srcSystem()}, component {heartbeat.get_srcComponent()}")
finally:
    connection.close()

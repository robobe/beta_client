"""Experimental local-only takeoff and landing demonstration."""
import math
from pathlib import Path
import subprocess
import sys
import threading
import time

from altitude_control import AltitudeControl
from beta_client.mavlink import MavlinkTelemetry
from msp_rc import MspRcSender
from pwm import THROTTLE_MIN
from physics import run as run_physics

# Timing is in seconds, altitude in metres, and speed in metres per second.
SITL_STARTUP_DELAY_S = 2
MISSION_TIMEOUT_S = 150
TELEMETRY_STALE_S = 3
HEARTBEAT_GRACE_S = 5
ALTITUDE_GRACE_S = 10
# Allow the simulated sensors to settle before requesting arming.
PREARM_SETTLE_S = 12
ARM_TIMEOUT_S = 15
DISARM_TIMEOUT_S = 10
FLIGHT_PHASE_TIMEOUT_S = 60
HOVER_DURATION_S = 10
CONTROL_PERIOD_S = 0.02
LOG_PERIOD_S = 1
SHUTDOWN_TIMEOUT_S = 5
TARGET_ALTITUDE_M = 10.0
LANDING_ALTITUDE_M = 0.0
HOVER_ALTITUDE_TOLERANCE_M = 0.5
HOVER_SPEED_TOLERANCE_M_S = 0.5
LANDING_ALTITUDE_TOLERANCE_M = 0.2
LANDING_SPEED_TOLERANCE_M_S = 0.4
MAVLINK_ENDPOINT = 'tcp:127.0.0.1:5762'
STATE_COLOR = '\033[1;36m'
COLOR_RESET = '\033[0m'


def print_state(state):
    message = f'state: {state}'
    print(f'{STATE_COLOR}{message}{COLOR_RESET}' if sys.stdout.isatty() else message, flush=True)


ROOT = Path(__file__).resolve().parents[2]
WORK = ROOT / '.sitl' / 'bullet-flight'
WORK.mkdir(parents=True, exist_ok=True)
subprocess.run([str(ROOT / 'bin/betaflight_2026.6.2_SITL'), '--config', str(Path(__file__).with_name('sitl.config'))],
               cwd=WORK, check=True, stdout=subprocess.DEVNULL)
sitl = subprocess.Popen([str(ROOT / 'bin/betaflight_2026.6.2_SITL')], cwd=WORK, stdout=subprocess.DEVNULL)
stop = threading.Event()
physics_error = []


def physics():
    try:
        run_physics(stop)
    except Exception as error:
        physics_error.append(error)
        stop.set()

thread = threading.Thread(target=physics, daemon=True)
thread.start()
rc = telemetry = None
try:
    time.sleep(SITL_STARTUP_DELAY_S)
    rc = MspRcSender()
    telemetry = MavlinkTelemetry(MAVLINK_ENDPOINT)
    telemetry.start()
    state = 'disarmed'
    print_state('disarmed')
    started = entered = time.monotonic()
    controller = AltitudeControl()
    last_sample = 0.0
    speed = 0.0
    last_log = 0.0
    while time.monotonic() - started < MISSION_TIMEOUT_S:
        now = time.monotonic()
        if stop.is_set() or sitl.poll() is not None:
            raise RuntimeError(f'Simulation stopped: {physics_error}')
        # Read once so altitude and its timestamp come from the same snapshot.
        sample = telemetry.snapshot()
        if now - sample.heartbeat_at > TELEMETRY_STALE_S and now - started > HEARTBEAT_GRACE_S:
            raise RuntimeError('MAVLink heartbeat stale')
        if now - sample.altitude_at > TELEMETRY_STALE_S and now - started > ALTITUDE_GRACE_S:
            raise RuntimeError('MAVLink altitude stale')
        alt = sample.altitude
        if alt is not None and not math.isfinite(alt):
            raise RuntimeError('Invalid altitude')
        new = state
        throttle, arm = THROTTLE_MIN, False
        if state == 'disarmed' and now - entered > PREARM_SETTLE_S and alt is not None:
            new = 'arming'
        elif state == 'arming':
            arm = True
            if sample.armed:
                new = 'armed'
            elif now - entered > ARM_TIMEOUT_S:
                raise TimeoutError('Arming not confirmed')
        elif state == 'armed':
            arm = True
            new = 'takeoff'
        elif state in ('takeoff', 'hover', 'land'):
            arm = True
            if not sample.armed:
                raise RuntimeError('Vehicle unexpectedly disarmed')
            target = LANDING_ALTITUDE_M if state == 'land' else TARGET_ALTITUDE_M
            # Integrate only new measurements, not every RC transmission.
            if sample.altitude_at > last_sample:
                throttle = controller.update(target, alt, sample.altitude_at, landing=state == 'land')
                last_sample = sample.altitude_at
            else:
                throttle = controller.throttle
            speed = controller.speed
            if state == 'takeoff' and abs(alt - TARGET_ALTITUDE_M) < HOVER_ALTITUDE_TOLERANCE_M and abs(speed) < HOVER_SPEED_TOLERANCE_M_S:
                new = 'hover'
            # Leaving the altitude band restarts the hover dwell.
            elif state == 'hover' and abs(alt - TARGET_ALTITUDE_M) > HOVER_ALTITUDE_TOLERANCE_M:
                new = 'takeoff'
            elif state == 'hover' and now - entered >= HOVER_DURATION_S:
                new = 'land'
            elif state == 'land' and alt < LANDING_ALTITUDE_TOLERANCE_M and abs(speed) < LANDING_SPEED_TOLERANCE_M_S:
                new = 'disarming'
            elif now - entered > FLIGHT_PHASE_TIMEOUT_S:
                raise TimeoutError(f'{state} timed out')
        elif state == 'disarming':
            if not sample.armed:
                print_state('disarmed')
                break
            if now - entered > DISARM_TIMEOUT_S:
                raise TimeoutError('Disarm not confirmed')
        if new != state:
            state, entered = new, now
            if state in ('takeoff', 'land'):
                controller.reset()
                last_sample = 0.0
            print_state(state)
        if now - last_log >= LOG_PERIOD_S:
            print(f"altitude={alt} speed={speed:.2f} target_speed={controller.target_speed:.2f} throttle={throttle} armed={sample.armed} attitude={sample.attitude} VFR_HUD={sample.vfr_hud_hz:.1f}Hz ATTITUDE={sample.attitude_hz:.1f}Hz", flush=True)
            last_log = now
        rc.send(throttle, arm)
        time.sleep(CONTROL_PERIOD_S)
    else:
        raise TimeoutError('Mission timed out')
finally:
    if rc:
        try:
            rc.send(THROTTLE_MIN, False)
        except OSError:
            pass
        rc.close()
    if telemetry:
        telemetry.close()
    stop.set()
    thread.join(timeout=SHUTDOWN_TIMEOUT_S)
    sitl.terminate()
    sitl.wait(timeout=SHUTDOWN_TIMEOUT_S)

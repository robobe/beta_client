"""Experimental local-only takeoff and landing demonstration."""
from pymavlink import mavutil

import math
from pathlib import Path
import subprocess
import threading
import time

from altitude_control import AltitudeControl
from mavlink_telemetry import MavlinkTelemetry
from physics import run as run_physics

ROOT = Path(__file__).resolve().parents[2]
WORK = ROOT / '.sitl' / 'mavlink-flight'
WORK.mkdir(parents=True, exist_ok=True)
config = subprocess.run([str(ROOT / 'bin/betaflight_2026.6.2_SITL'), '--config', str(Path(__file__).with_name('sitl.config'))],
                        cwd=WORK, check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
errors = [line for line in config.stdout.splitlines() if '###ERROR' in line]
if errors:
    raise RuntimeError('SITL configuration rejected:\n' + '\n'.join(errors))
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
    time.sleep(2)
    telemetry = MavlinkTelemetry()
    rc = telemetry
    state = 'disarmed'
    print('state: disarmed', flush=True)
    started = entered = time.monotonic()
    controller = AltitudeControl()
    last_sample = 0.0
    speed = 0.0
    last_log = 0.0
    while time.monotonic() - started < 150:
        now = time.monotonic()
        if stop.is_set() or sitl.poll() is not None:
            raise RuntimeError(f'Simulation stopped: {physics_error}')
        telemetry.poll()
        sample = telemetry.snapshot()
        if now - sample.heartbeat_at > 3 and now - started > 5:
            raise RuntimeError('MAVLink heartbeat stale')
        if now - sample.vfr_hud_at > 3 and now - started > 10:
            raise RuntimeError('MAVLink altitude stale')
        alt = (sample.vfr_hud.alt if sample.vfr_hud else None)
        if alt is not None and not math.isfinite(alt):
            raise RuntimeError('Invalid altitude')
        new = state
        throttle, arm = 1000, False
        if state == 'disarmed' and now - entered > 12 and alt is not None:
            new = 'arming'
        elif state == 'arming':
            arm = True
            if bool(sample.heartbeat and sample.heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                new = 'armed'
            elif now - entered > 15:
                raise TimeoutError('Arming not confirmed')
        elif state == 'armed':
            arm = True
            new = 'takeoff'
        elif state in ('takeoff', 'hover', 'land'):
            arm = True
            if not bool(sample.heartbeat and sample.heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                raise RuntimeError('Vehicle unexpectedly disarmed')
            target = 0.0 if state == 'land' else 10.0
            if sample.vfr_hud_at > last_sample:
                throttle = controller.update(target, alt, sample.vfr_hud_at, landing=state == 'land')
                last_sample = sample.vfr_hud_at
            else:
                throttle = controller.throttle
            speed = controller.speed
            if state == 'takeoff' and abs(alt - 10) < 0.5 and abs(speed) < 0.5:
                new = 'hover'
            elif state == 'hover' and abs(alt - 10) > 0.5:
                new = 'takeoff'
            elif state == 'hover' and now - entered >= 10:
                new = 'land'
            elif state == 'land' and alt < 0.2 and abs(speed) < 0.4:
                new = 'disarming'
            elif now - entered > 60:
                raise TimeoutError(f'{state} timed out')
        elif state == 'disarming':
            if not bool(sample.heartbeat and sample.heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print('state: disarmed', flush=True)
                break
            if now - entered > 10:
                raise TimeoutError('Disarm not confirmed')
        if new != state:
            state, entered = new, now
            if state in ('takeoff', 'land'):
                controller.reset()
                last_sample = 0.0
            print(f'state: {state}', flush=True)
        if now - last_log >= 1:
            rates = telemetry.rates()
            print(f"altitude={alt} speed={speed:.2f} target_speed={controller.target_speed:.2f} throttle={throttle} armed={bool(sample.heartbeat and sample.heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)} attitude={sample.attitude} VFR_HUD={rates['VFR_HUD']:.1f}Hz ATTITUDE={rates['ATTITUDE']:.1f}Hz", flush=True)
            last_log = now
        rc.send(throttle, arm)
        time.sleep(0.02)
    else:
        raise TimeoutError('Mission timed out')
finally:
    if rc:
        try:
            rc.send(1000, False)
        except OSError:
            pass
        rc.close()
    stop.set()
    thread.join(timeout=5)
    sitl.terminate()
    sitl.wait(timeout=5)

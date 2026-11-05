"""Run Bullet physics in real time against an already running SITL."""
import time
from bullet_drone.config import ControlSettings, DroneSettings, SimulationConfig, SimulationSettings
from bullet_drone.controllers import InternalControlSettings, PidSettings
from bullet_drone.simulation import RcSimulation


PHYSICS_RATE_HZ = 240
MAX_SIMULATION_STEPS = 1_000_000
GRAVITY_M_S2 = (0, 0, -9.81)
DRONE_MASS_KG = 1.0
# Start slightly above the ground to avoid initial mesh penetration.
INITIAL_POSITION_M = (0, 0, 0.015)
ATTITUDE_KP = (4, 4, 4)
MAX_RATES_RAD_S = (2, 2, 2)
RATE_KP = 0.05
RATE_OUTPUT_LIMIT_NM = 0.1


def run(stop):
    dt = 1 / PHYSICS_RATE_HZ
    config = SimulationConfig(SimulationSettings(dt, MAX_SIMULATION_STEPS, GRAVITY_M_S2, gui=False),
                              DroneSettings(DRONE_MASS_KG, INITIAL_POSITION_M), control=ControlSettings('betaflight'))
    pid = PidSettings(kp=RATE_KP, ki=0, kd=0, output_limit=RATE_OUTPUT_LIMIT_NM, integral_limit=0)
    settings = InternalControlSettings(ATTITUDE_KP, MAX_RATES_RAD_S, (pid, pid, pid))
    simulation = RcSimulation(config, settings)
    try:
        simulation.reset()
        next_step = time.monotonic()
        while not stop.is_set():
            simulation.step()
            # Schedule against an absolute clock so step time does not accumulate drift.
            next_step += dt
            stop.wait(max(0, next_step - time.monotonic()))
    finally:
        simulation.close()

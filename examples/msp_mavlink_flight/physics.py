"""Run Bullet physics in real time against an already running SITL."""
import time
from bullet_drone.config import ControlSettings, DroneSettings, SimulationConfig, SimulationSettings
from bullet_drone.controllers import InternalControlSettings, PidSettings
from bullet_drone.simulation import RcSimulation


def run(stop):
    dt = 1 / 240
    config = SimulationConfig(SimulationSettings(dt, 1000000, (0, 0, -9.81), gui=False),
                              DroneSettings(1.0, (0, 0, 0.015)), control=ControlSettings('betaflight'))
    pid = PidSettings(kp=0.05, ki=0, kd=0, output_limit=0.1, integral_limit=0)
    settings = InternalControlSettings((4, 4, 4), (2, 2, 2), (pid, pid, pid))
    simulation = RcSimulation(config, settings)
    try:
        simulation.reset()
        next_step = time.monotonic()
        while not stop.is_set():
            simulation.step()
            next_step += dt
            stop.wait(max(0, next_step - time.monotonic()))
    finally:
        simulation.close()

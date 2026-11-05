"""Simulation-only altitude-to-velocity and velocity-to-throttle cascade."""


from pwm import CONTROL_THROTTLE_MAX, CONTROL_THROTTLE_MIN, HOVER_THROTTLE


def clamp(value, low, high):
    return max(low, min(high, value))


class AltitudeControl:
    def __init__(self):
        self.reset()

    def reset(self):
        self.previous = None
        self.speed = 0.0
        self.integral = 0.0
        self.target_speed = 0.0
        self.throttle = HOVER_THROTTLE

    def update(self, target, altitude, timestamp, landing=False):
        if self.previous is None:
            self.previous = (altitude, timestamp)
            return self.throttle
        old_altitude, old_time = self.previous
        dt = timestamp - old_time
        if dt <= 0:
            return self.throttle
        self.previous = (altitude, timestamp)
        if dt > 1:
            self.speed = 0.0
            self.integral = 0.0
            return self.throttle
        self.speed += 0.4 * ((altitude - old_altitude) / dt - self.speed)
        self.target_speed = clamp(0.6 * (target - altitude), -0.5 if landing else -1.5, 1.5)
        error = self.target_speed - self.speed
        self.integral = clamp(self.integral + error * dt, -8, 8)
        self.throttle = round(clamp(HOVER_THROTTLE + 100 * error + 25 * self.integral,
                                    CONTROL_THROTTLE_MIN, CONTROL_THROTTLE_MAX))
        return self.throttle

"""Simulation-only altitude-to-velocity and velocity-to-throttle cascade."""


from pwm import CONTROL_THROTTLE_MAX, CONTROL_THROTTLE_MIN, HOVER_THROTTLE


# Simulation-specific tuning, speed limits are in metres per second.
SPEED_FILTER_ALPHA = 0.4
ALTITUDE_KP = 0.6
MAX_VERTICAL_SPEED_M_S = 1.5
LANDING_DESCENT_SPEED_M_S = 0.5
VELOCITY_KP = 100
VELOCITY_KI = 25
INTEGRAL_LIMIT = 8
MAX_SAMPLE_GAP_S = 1


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
        # A long gap makes the derivative unreliable, discard accumulated error.
        if dt > MAX_SAMPLE_GAP_S:
            self.speed = 0.0
            self.integral = 0.0
            return self.throttle
        self.speed += SPEED_FILTER_ALPHA * ((altitude - old_altitude) / dt - self.speed)
        self.target_speed = clamp(ALTITUDE_KP * (target - altitude),
                                  -LANDING_DESCENT_SPEED_M_S if landing else -MAX_VERTICAL_SPEED_M_S,
                                  MAX_VERTICAL_SPEED_M_S)
        error = self.target_speed - self.speed
        # Limit integral windup while the throttle output is saturated.
        self.integral = clamp(self.integral + error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)
        self.throttle = round(clamp(HOVER_THROTTLE + VELOCITY_KP * error + VELOCITY_KI * self.integral,
                                    CONTROL_THROTTLE_MIN, CONTROL_THROTTLE_MAX))
        return self.throttle

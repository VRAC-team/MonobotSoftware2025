class PID:
    def __init__(self, kp: float, ki: float, kd: float, integrator_max: float | None = 1000):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integrator_max = integrator_max

        self.last_error = 0.0
        self.integrator = 0.0

    def reset(self):
        self.integrator = 0.0
        self.last_error = 0.0

    def compute(self, error: float) -> float:
        self.integrator += error

        if self.integrator_max is not None:
            if self.integrator > self.integrator_max:
                self.integrator = self.integrator_max
            elif self.integrator < -self.integrator_max:
                self.integrator = -self.integrator_max

        output = self.kp * error
        output += self.ki * self.integrator
        output += self.kd * (error - self.last_error)

        self.last_error = error

        return output

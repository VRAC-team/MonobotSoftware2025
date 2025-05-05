import collections


class ThresholdFilter:
    def __init__(self, threshold: int):
        self.threshold = threshold
        self.filtered_value: int = 0

    def reset(self):
        self.filtered_value = 0

    def update(self, value: int):
        if abs(value - self.filtered_value) > self.threshold:
            self.filtered_value = value

    def get(self):
        return self.filtered_value


class MovingAverageFilter:
    def __init__(self, window_size: int = 2):
        self.window_size = window_size

        self.values = collections.deque()
        self.values.append(0)
        self.sum = 0

    def reset(self):
        self.values.clear()
        self.values.append(0)
        self.sum = 0

    def update(self, value: int):
        self.values.append(value)
        self.sum += value

        if len(self.values) > self.window_size:
            removed = self.values.popleft()
            self.sum -= removed

    def get(self) -> float:
        return self.sum / len(self.values)


class RampFilter:
    def __init__(self, control_loop_period: float, accel_rate: float, decel_rate: float):
        self.accel_rate = accel_rate * control_loop_period
        self.decel_rate = decel_rate * control_loop_period
        self.current_value = 0.0

    def reset(self):
        self.current_value = 0.0

    def update(self, target: float):
        delta = target - self.current_value

        if delta == 0:
            return self.current_value

        direction = 1 if delta > 0 else -1

        if (self.current_value * direction) < 0:
            change = self.decel_rate * direction
        elif abs(delta) > self.accel_rate:
            if (self.current_value < target and direction > 0) or (self.current_value > target and direction < 0):
                change = self.accel_rate * direction
            else:
                change = self.decel_rate * direction
        else:
            self.current_value = target
            return self.current_value

        self.current_value += change

        if (direction > 0 and self.current_value > target) or (direction < 0 and self.current_value < target):
            self.current_value = target

        return self.current_value

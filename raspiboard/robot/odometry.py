import math
import threading
from robot.filters import MovingAverageFilter


class Odometry:
    def __init__(
        self,
        control_loop_period: float,
        wheel_perimeter: float,
        ticks_per_rev: int,
        wheel_spacing: float,
    ):
        """
        :param control_loop_period: in seconds
        :param wheel_perimeter: odometry wheel perimeter, in mm
        :param ticks_per_rev: in ticks
        :param wheel_spacing: spacing between the two odometry wheels, in mm
        """
        self.control_loop_period = control_loop_period
        self.wheel_spacing = wheel_spacing
        self.k_wheel = wheel_perimeter / ticks_per_rev

        self.last_ticks_left = 0
        self.last_ticks_right = 0
        self.x_mm = 0.0
        self.y_mm = 0.0
        self.theta_rad = 0.0
        self.distance_mm = 0.0
        self.filter_vel_dist = MovingAverageFilter(5)
        self.filter_vel_theta = MovingAverageFilter(5)
        self.lock = threading.Lock()

    def reset(self):
        self.last_ticks_left = 0
        self.last_ticks_right = 0
        self.x_mm = 0.0
        self.y_mm = 0.0
        self.theta_rad = 0.0
        self.distance_mm = 0.0
        self.filter_vel_dist.reset()
        self.filter_vel_theta.reset()

    def update(self, ticks_left: int, ticks_right: int):
        delta_left = (ticks_left - self.last_ticks_left) * self.k_wheel
        delta_right = (ticks_right - self.last_ticks_right) * self.k_wheel

        delta_theta = (delta_right - delta_left) / self.wheel_spacing
        delta_distance = (delta_right + delta_left) / 2.0

        delta_distance_x = delta_distance * math.cos(self.theta_rad)
        delta_distance_y = delta_distance * math.sin(self.theta_rad)

        vel_theta = delta_theta / self.control_loop_period
        vel_dist = delta_distance / self.control_loop_period

        with self.lock:
            self.last_ticks_left = ticks_left
            self.last_ticks_right = ticks_right
            self.x_mm += delta_distance_x
            self.y_mm += delta_distance_y
            self.theta_rad += delta_theta
            self.distance_mm += delta_distance
            self.filter_vel_dist.update(vel_dist)
            self.filter_vel_theta.update(vel_theta)

    def get_x(self) -> float:
        with self.lock:
            return self.x_mm

    def get_y(self) -> float:
        with self.lock:
            return self.y_mm

    def get_theta(self) -> float:
        with self.lock:
            return self.theta_rad * 180.0 / math.pi

    def get_velocity_theta(self) -> float:
        with self.lock:
            return self.filter_vel_theta.get() * 180.0 / math.pi

    def get_distance(self) -> float:
        with self.lock:
            return self.distance_mm

    def get_velocity_distance(self) -> float:
        with self.lock:
            return self.filter_vel_dist.get()

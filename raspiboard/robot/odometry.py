import math
import threading

from robot.filters import MovingAverageFilter
from robot.parameters import RobotParameters


class Odometry:
    def __init__(self, params: RobotParameters):
        self.params = params
        self.k_wheel = self.params.ODOMETRY_WHEEL_PERIMETER_MM / self.params.ODOMETRY_TICKS_PER_REV

        self.last_ticks_left = 0
        self.last_ticks_right = 0
        self.x_mm = 0.0
        self.y_mm = 0.0
        self.theta_rad = 0.0
        self.distance_mm = 0.0
        self.filter_vel_dist = MovingAverageFilter(window_size=5)
        self.filter_vel_theta = MovingAverageFilter(window_size=5)

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

    def set(self, x_mm: float = None, y_mm: float = None, theta_rad: float = None):
        with self.lock:
            if x_mm is not None:
                self.x_mm = x_mm
            if y_mm is not None:
                self.y_mm = y_mm
            if theta_rad is not None:
                self.theta_rad = theta_rad
            self.last_ticks_left = 0
            self.last_ticks_right = 0
            self.distance_mm = 0.0
            self.filter_vel_dist.reset()
            self.filter_vel_theta.reset()

    def update(self, ticks_left: int, ticks_right: int) -> None:
        delta_left = (ticks_left - self.last_ticks_left) * self.k_wheel
        delta_right = (ticks_right - self.last_ticks_right) * self.k_wheel

        delta_theta = (delta_right - delta_left) / self.params.ODOMETRY_WHEEL_SPACING_MM
        delta_distance = (delta_right + delta_left) / 2.0

        delta_distance_x = delta_distance * math.cos(self.theta_rad)
        delta_distance_y = delta_distance * math.sin(self.theta_rad)

        vel_theta = delta_theta / self.params.CONTROLLOOP_PERIOD_S
        vel_dist = delta_distance / self.params.CONTROLLOOP_PERIOD_S

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

    def get_theta_rad(self) -> float:
        with self.lock:
            return self.theta_rad

    def get_theta_deg(self) -> float:
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

    def __str__(self):
        return f"Odometry(x={self.x_mm:.1f}mm, y={self.y_mm:.1f}mm, theta={self.get_theta_deg():.1f}deg)"

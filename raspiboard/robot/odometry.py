import math
import threading

from .filters import MovingAverageFilter
from .robot_config import RobotConfig


class Odometry:
    def __init__(self, config: RobotConfig):
        self.config = config

        self.control_loop_period = self.config.get_controlloop_period()

        self.k_dist = self.config.ODOMETRY_WHEEL_PERIMETER / self.config.ODOMETRY_TICKS_PER_REV
        self.k_theta = self.config.ODOMETRY_WHEEL_PERIMETER / self.config.ODOMETRY_TICKS_PER_REV / self.config.ODOMETRY_WHEEL_SPACING

        self.last_ticks_left = 0
        self.last_ticks_right = 0
        self.theta_ticks = 0
        self.theta_rad = 0.0
        self.theta_deg = 0.0
        self.distance_ticks = 0
        self.distance_mm = 0.0
        self.x_mm = 0.0
        self.y_mm = 0.0
        self.avg_vel_dist = MovingAverageFilter(window_size=2)
        self.avg_vel_theta = MovingAverageFilter(window_size=2)

        self.lock = threading.Lock()

    def reset(self):
        self.last_ticks_left = 0
        self.last_ticks_right = 0
        self.theta_ticks = 0
        self.theta_rad = 0.0

        self.distance_ticks = 0
        self.distance_mm = 0.0
        self.x_mm = 0.0
        self.y_mm = 0.0
        self.avg_vel_dist.reset()
        self.avg_vel_theta.reset()

    def set(self, x_mm: float | None = None, y_mm: float | None = None, theta_rad: float | None = None):
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
            self.avg_vel_dist.reset()
            self.avg_vel_theta.reset()

    def update(self, ticks_left: int, ticks_right: int) -> None:
        delta_left_ticks = ticks_left - self.last_ticks_left
        delta_right_ticks = ticks_right - self.last_ticks_right

        delta_theta_ticks = delta_right_ticks - delta_left_ticks
        delta_theta_rad = delta_theta_ticks * self.k_theta
        vel_theta_deg = math.degrees(delta_theta_rad) / self.control_loop_period

        delta_distance_ticks = (delta_right_ticks + delta_left_ticks) / 2
        delta_distance_mm = delta_distance_ticks * self.k_dist
        vel_dist_mm = delta_distance_mm / self.control_loop_period

        with self.lock:
            self.theta_ticks += delta_theta_ticks

            theta_rad = self.theta_ticks * self.k_theta

            self.theta_deg = math.degrees(theta_rad)
            self.distance_ticks += delta_distance_ticks
            self.distance_mm = self.distance_ticks * self.k_dist
            self.x_mm += delta_distance_mm * math.cos(theta_rad)
            self.y_mm += delta_distance_mm * math.sin(theta_rad)
            self.avg_vel_dist.update(vel_dist_mm)
            self.avg_vel_theta.update(vel_theta_deg)
            self.last_ticks_left = ticks_left
            self.last_ticks_right = ticks_right

    def get_x(self) -> float:
        with self.lock:
            return self.x_mm

    def get_y(self) -> float:
        with self.lock:
            return self.y_mm

    def get_theta(self) -> float:
        with self.lock:
            return self.theta_deg

    def get_theta_vel(self) -> float:
        with self.lock:
            return self.avg_vel_theta.get()

    def get_dist(self) -> float:
        with self.lock:
            return self.distance_mm

    def get_dist_vel(self) -> float:
        with self.lock:
            return self.avg_vel_dist.get()

    def __str__(self):
        return f"Odometry(x={self.x_mm:.1f}mm, y={self.y_mm:.1f}mm, theta={self.theta_deg:.1f}deg)"

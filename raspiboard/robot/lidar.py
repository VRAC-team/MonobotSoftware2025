import pyrplidar
import threading
import numpy as np
import time
import math
import os
import logging

# import cv2

from robot.odometry import Odometry

GRID_RESOLUTION_MM = 10
MAP_WIDTH_MM = 3000
MAP_HEIGHT_MM = 2000
MAP_WIDTH_PX = MAP_WIDTH_MM // GRID_RESOLUTION_MM
MAP_HEIGHT_PX = MAP_HEIGHT_MM // GRID_RESOLUTION_MM


class Lidar:
    def __init__(self, odometry: Odometry):
        self.lidar = pyrplidar.PyRPlidar()
        self.t_lidar = threading.Thread(target=self._thread_receive)
        self.odometry = odometry

        self.map_grid = np.zeros((MAP_HEIGHT_PX, MAP_WIDTH_PX), dtype=np.uint8)

        self.emergency_deceleration = 1000  # mm/s^2
        # Here image a retangle in the of the robot used to determine if we should emergency brake
        # The width is the emergency_width (robot width)
        # The rectangle is not starting at the robot front plane, but is offseted in the front by self.emergency_stop_near
        # The rectangle length is self.emergency_stop_min_distance
        self.emergency_width = 350
        self.emergency_stop_near = 70
        self.emergency_stop_min_distance = 250

        self.emergency_stop_front = False
        self.emergency_stop_back = False
        self.lock = threading.Lock()

        self.logger = logging.getLogger(self.__class__.__name__)

    def is_emergency_stop(self, front: bool) -> bool:
        with self.lock:
            if front:
                return self.emergency_stop_front
            else:
                return self.emergency_stop_back

    def _on_new_scan(self, points: list[pyrplidar.PyRPlidarMeasurement]):
        angles = []
        distances = []
        for p in points:
            if p.distance > 0 and p.distance < 1000:
                angles.append(p.angle)
                distances.append(p.distance)

        if not distances:
            return

        # polar_to_cartesian
        angles = np.radians(angles)
        distances = np.array(distances)
        x_local = distances * np.cos(angles)
        y_local = distances * np.sin(angles)

        # robot_rad = self.odometry.get_theta_rad()
        # robot_x = self.odometry.get_x()
        # robot_y = self.odometry.get_y()

        # # Rotate local coordinates to global/world coordinates
        # x_world = x_local * np.cos(robot_rad) - y_local * np.sin(robot_rad) + robot_x
        # y_world = x_local * np.sin(robot_rad) + y_local * np.cos(robot_rad) + robot_y

        # # Convert to map pixel indices
        # x_pix = np.floor(x_world / GRID_RESOLUTION_MM).astype(int)
        # y_pix = np.floor(y_world / GRID_RESOLUTION_MM).astype(int)

        # # Plot on map if within bounds
        # self.map_grid.fill(0)

        # for x, y in zip(x_pix, y_pix):
        #     if 0 <= x < MAP_WIDTH_PX and 0 <= y < MAP_HEIGHT_PX:
        #         self.map_grid[y, x] = 255  # mark enemy pixel

        # emergency stop
        decel_time = math.fabs(self.odometry.get_velocity_distance()) / self.emergency_deceleration
        decel_dist = 0.5 * self.emergency_deceleration * decel_time**2
        decel_dist *= 0.5  # safety margins

        emergency_stop_far = self.emergency_stop_near + self.emergency_stop_min_distance + decel_dist
        emergency_stop_far = max(self.emergency_stop_min_distance, emergency_stop_far)

        in_front_area_counter = 0
        in_back_area_counter = 0

        for x, y in zip(x_local, y_local):
            if (
                self.emergency_stop_near <= x <= emergency_stop_far
                and -self.emergency_width / 2 <= y <= self.emergency_width / 2
            ):
                in_front_area_counter += 1
            if (
                -self.emergency_stop_near >= x >= -emergency_stop_far
                and -self.emergency_width / 2 <= y <= self.emergency_width / 2
            ):
                in_back_area_counter += 1

        with self.lock:
            self.emergency_stop_front = False
            self.emergency_stop_back = False

            if in_front_area_counter > 5:
                # self.logger.warning("in_front_area_counter %d", in_front_area_counter)
                self.emergency_stop_front = True

            if in_back_area_counter > 5:
                # self.logger.warning("in_back_area_counter %d", in_back_area_counter)
                self.emergency_stop_back = True

        # display = cv2.resize(self.map_grid, (MAP_WIDTH_PX * 3, MAP_HEIGHT_PX * 3), interpolation=cv2.INTER_NEAREST)
        # cv2.imshow("Live LIDAR Map", display)
        # key = cv2.waitKey(1)
        # if key == 27:  # ESC key to exit
        #     self.stop()

    def _thread_receive(self):
        scan_generator = self.lidar.start_scan_express(2)  # Boost mode (8k sampling)

        last_fullscan_points = []

        try:
            for p in scan_generator():
                if p.start_flag:
                    self._on_new_scan(last_fullscan_points)
                    last_fullscan_points.clear()
                last_fullscan_points.append(p)
        except TypeError:
            pass

    def is_device_connected(self, device_path="/dev/ttyUSB0"):
        if os.path.exists(device_path):
            print(f"{device_path} is connected.")
            return True
        else:
            print(f"{device_path} is not connected.")
            return False

    def start(self):
        if not self.is_device_connected():
            print("lidar not found")
            return False

        self.lidar.connect(port="/dev/ttyUSB0", baudrate=115200, timeout=2)
        self.lidar.set_motor_pwm(660)
        self.t_lidar.start()
        print("lidar start")
        return True

    def stop(self):
        if self.lidar.lidar_serial is not None:
            self.lidar.stop()
            time.sleep(0.5)
            self.lidar.disconnect()  # this should raise a TypeError in _thread_receive and stop the thread
            print("lidar stop")

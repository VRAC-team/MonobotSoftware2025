import time

from robot import Odometry, RobotParameters, Lidar


def test_lidar():
    params = RobotParameters()
    odo = Odometry(params)
    odo.set(1500, 1000, 0)
    for i in range(5):
        odo.filter_vel_dist.update(200)
    lidar = Lidar(odo)
    print("lidar vel dist:", odo.get_velocity_distance())

    lidar.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        lidar.stop()
        print("Stopped!")


test_lidar()

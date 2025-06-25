import rclpy
from choirbot.collision import ClosestRobotGetter


def main():
    rclpy.init()

    distance = 1.5
    closest_robot_getter = ClosestRobotGetter(sensing_distance=distance)

    rclpy.spin(closest_robot_getter)
    rclpy.shutdown()

import rclpy
from choirbot.controller import UnicyclePoseController


def main():
    rclpy.init()

    controller = UnicyclePoseController('pubsub', 'odom')

    rclpy.spin(controller)
    rclpy.shutdown()

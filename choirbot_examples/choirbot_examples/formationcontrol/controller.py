import rclpy
from choirbot.controller import UnicycleVelocityController


def main():
    rclpy.init()

    controller = UnicycleVelocityController('pubsub', 'odom')

    rclpy.spin(controller)
    rclpy.shutdown()

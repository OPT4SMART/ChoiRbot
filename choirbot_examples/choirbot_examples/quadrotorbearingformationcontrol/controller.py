import rclpy
from choirbot.controller import QuadrotorAccelerationController


def main():
    rclpy.init()

    controller = QuadrotorAccelerationController('pubsub', 'odom')

    rclpy.spin(controller)
    rclpy.shutdown()

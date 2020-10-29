import rclpy
from choirbot.utils import Visualizer


def main():
    rclpy.init()

    visualizer = Visualizer('pubsub', 'odom')

    rclpy.spin(visualizer)
    rclpy.shutdown()

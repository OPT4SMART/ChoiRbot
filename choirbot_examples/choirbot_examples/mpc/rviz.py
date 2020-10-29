import rclpy
from choirbot.utils import Visualizer


def main():
    rclpy.init()

    visualizer = Visualizer(pose_handler='pubsub', pose_topic='odom')

    rclpy.spin(visualizer)
    rclpy.shutdown()

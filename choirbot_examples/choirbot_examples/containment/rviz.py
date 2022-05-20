import rclpy
from choirbot.utils import SimpleVisualizer


def main():
    rclpy.init()

    visualizer = SimpleVisualizer(pose_handler='pubsub', pose_topic='odom')

    rclpy.spin(visualizer)
    rclpy.shutdown()

import rclpy
from rclpy.node import Node
from choirbot.utils import RvizSpawner
import numpy as np

def main():
    rclpy.init(args=None)

    pos_handler = 'pubsub'
    pos_topic = 'odom'

    spawn = RvizSpawner(pos_handler, pos_topic)

    rclpy.spin(spawn)

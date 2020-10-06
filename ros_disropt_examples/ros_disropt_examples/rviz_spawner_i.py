import rclpy
from rclpy.node import Node
from ros_disropt.utils import RvizSpawner
import numpy as np

def main():
    rclpy.init(args=None)

    pos_handler = 'pubsub'
    pos_topic = '/agent_{}/odom'.format(agent_id)

    spawn = RvizSpawner(pos_handler, pos_topic)

    rclpy.spin(spawn)

import rclpy
from rclpy.node import Node
from choirbot.guidance.distributed_control import FormationControlGuidance
import numpy as np

def main():
    rclpy.init(args=None)

    update_frequency = 100

    pos_handler = 'pubsub'
    pos_topic = 'odom'

    guidance = FormationControlGuidance(update_frequency, 0.1, pos_handler, pos_topic)

    rclpy.spin(guidance)

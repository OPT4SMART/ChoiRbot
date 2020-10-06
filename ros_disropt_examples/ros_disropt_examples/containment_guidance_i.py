import rclpy
from rclpy.node import Node
from ros_disropt.guidance.distributed_control import ContainmentGuidance
import numpy as np

def main():
    rclpy.init(args=None)
    
    update_frequency = 100.0

    pos_handler = 'pubsub'
    pos_topic = 'odom'

    guidance = ContainmentGuidance(update_frequency, 0.1, pos_handler, pos_topic)

    rclpy.spin(guidance)

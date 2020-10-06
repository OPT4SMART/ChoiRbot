import rclpy
from rclpy.node import Node
from ros_disropt.guidance.distributed_control import FormationControlGuidance
import numpy as np

def main():
    rclpy.init(args=None)

    update_frequency = 100

    pos_handler = 'pubsub'
    pos_topic = '/agent_{}/odom'.format(agent_id)

    guidance = FormationControlGuidance(update_frequency, 0.1, pos_handler, pos_topic)

    rclpy.spin(guidance)

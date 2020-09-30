import rclpy
from rclpy.node import Node
from ros_disropt.utils import RvizSpawner
import numpy as np

def main():
    rclpy.init(args=None)
    node = Node('node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
    agent_id = node.get_parameter('agent_id').value


    pos_handler = 'pubsub'
    pos_topic = '/agent_{}/odom'.format(agent_id)

    spawn = RvizSpawner(agent_id, pos_handler, pos_topic)

    rclpy.spin(spawn)

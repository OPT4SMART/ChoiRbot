import numpy as np
import rclpy
from geometry_msgs.msg import Point
from typing import List
from .. import RobotData
from ..guidance import Guidance
from time import sleep


class DistributedControl(Guidance):

    def __init__(self, agent_id: int, N: int, in_neigh: List[int], out_neigh: List[int], update_frequency: float, pos_handler: str=None, pos_topic: str=None):
        super().__init__(agent_id, N, in_neigh, out_neigh, pos_handler, pos_topic)
        self.publisher_ = self.create_publisher(Point, 'agent_{}_velocity'.format(agent_id), 1)
        self.update_frequency = update_frequency
        self.timer = self.create_timer(1.0/update_frequency, self.control)

    def control(self):
        pass

    def send_message(self, u):
        msg = Point()

        msg.x = u[0]
        msg.y = u[1]
        msg.z = u[2]

        self.publisher_.publish(msg)

    def evaluate_velocity(self, current_pose, neigh_data):
        print("parent method")
        pass
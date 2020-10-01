import numpy as np
import rclpy
from geometry_msgs.msg import Point
from typing import List
from .distributed_control import DistributedControl
from copy import deepcopy


class ContainmentGuidance(DistributedControl):

    def __init__(self, agent_id: int, N: int, in_neigh: List[int], out_neigh: List[int], update_frequency: float, is_leader: bool, pos_handler: str=None, pos_topic: str=None):
        super().__init__(agent_id, N, in_neigh, out_neigh, update_frequency, pos_handler, pos_topic)
        self.is_leader = is_leader
        self.containment_gain = 0.1

    def control(self):
        pos = np.copy(self.current_pose.position)
        if pos.ndim is not 0:
            pose = deepcopy(self.current_pose)
            data = self.communicator.neighbors_exchange(pose.position, self.in_neighbors, self.out_neighbors, False)
            u = self.evaluate_velocity(pose, data)
            self.send_message(u)

    def evaluate_velocity(self, current_pose, neigh_data):
        u = np.zeros(3)
        if not self.is_leader:
            for ii in neigh_data:
                pos_ii = neigh_data[ii]
                error = self.containment_gain*(pos_ii - current_pose.position)
                u += error
        return u
        
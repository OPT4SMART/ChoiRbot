import numpy as np
import rclpy
from geometry_msgs.msg import Point
from typing import List
from .distributed_control import DistributedControl
from copy import deepcopy


class FormationControlGuidance(DistributedControl):

    def __init__(self, weights:np.ndarray, update_frequency: float, pos_handler: str=None, pos_topic: str=None):
        super().__init__(update_frequency, pos_handler, pos_topic)
        self.x_neigh = {}
        self.weights = weights
        self.formation_control_gain = 0.1

    def control(self):
        self.u = 0.0
        pos = np.copy(self.current_pose.position)
        if pos.ndim is not 0:
            pose = deepcopy(self.current_pose)
            data = self.communicator.neighbors_exchange(pose.position, self.in_neighbors, self.out_neighbors, False)
            u = self.evaluate_velocity(pose, data)
            self.send_message(u)

    def evaluate_velocity(self, current_pose, neigh_data):
        u = np.zeros(3)
        for ii in neigh_data:
            pos_ii = neigh_data[ii]
            error = pos_ii - current_pose.position
            u += self.formation_control_gain*(np.power(np.linalg.norm(error), 2)- np.power(self.weights[ii], 2)) * error
        return u
        
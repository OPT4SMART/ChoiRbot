import numpy as np
import random
from .distributed_control import DistributedControlGuidance


class ContainmentGuidance(DistributedControlGuidance):

    def __init__(self, update_frequency: float, gain: float=0.1, pos_handler: str=None, pos_topic: str=None):
        super().__init__(update_frequency, pos_handler, pos_topic)
        self.containment_gain = gain
        self.is_leader = self.get_parameter('is_leader').value

    def evaluate_velocity(self, neigh_data):
        u = np.zeros(3)
        if not self.is_leader:
            for pos_ii in neigh_data.values():
                u += self.containment_gain*(pos_ii - self.current_pose.position)
        return u

class TimeVaryingContainmentGuidance(ContainmentGuidance):

    def __init__(self, update_frequency: float, gain: float=0.1, edge_prob = 0.8, pos_handler: str=None, pos_topic: str=None):
        super().__init__(update_frequency, gain, pos_handler, pos_topic)
        self.edge_prob = edge_prob
    
    def control(self):
        # skip if position is not available yet
        if self.current_pose.position is None:
            return
        
        # decide in-neighbors
        in_neigh = random.sample(self.in_neighbors, np.random.binomial(len(self.in_neighbors), self.edge_prob))
        
        # exchange current position with neighbors
        data = self.communicator.neighbors_exchange(self.current_pose.position, in_neigh, self.out_neighbors, False)

        # discard messages from non-in-neighbors (to make graph undirected)
        filtered_data = {j:value for j, value in data.items() if j in in_neigh}

        # compute input
        u = self.evaluate_velocity(filtered_data)

        # send input to planner/controller
        self.send_input(u)

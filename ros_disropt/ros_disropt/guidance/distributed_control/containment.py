import numpy as np
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

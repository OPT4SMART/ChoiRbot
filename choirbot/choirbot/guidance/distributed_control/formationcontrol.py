import numpy as np
from numpy.linalg import norm
from .distributed_control import DistributedControlGuidance


class FormationControlGuidance(DistributedControlGuidance):
    """
    Formation Control 

    Implements a formation control law for systems....
    """

    def __init__(self, update_frequency: float, gain: float=0.1, pos_handler: str=None, pos_topic: str=None):
        """
        Init method
        """
        super().__init__(update_frequency, pos_handler, pos_topic)
        self.formation_control_gain = gain
        self.weights = self.get_parameter('weights').value

    def evaluate_velocity(self, neigh_data):
        u = np.zeros(3)
        for ii, pos_ii in neigh_data.items():
            error = pos_ii - self.current_pose.position
            u += self.formation_control_gain*(norm(error)**2- self.weights[ii]**2) * error
        return u

import numpy as np
from .distributed_control import DistributedControlGuidance

class BearingFormationControlGuidance(DistributedControlGuidance):
    """
    Formation Control 

    Implements a bearing-based formation control law for second order systems.
    """
    def __init__(self, update_frequency: float, prop_gain: float=1.0, deriv_gain=5.0, 
                 pose_handler: str=None, pose_topic: str=None, input_topic = 'acceleration'):
        super().__init__(update_frequency, pose_handler, pose_topic, input_topic)

        self.is_leader = self.get_parameter('is_leader').value
        self.agent_dim = self.get_parameter('dd').value
        self.ort_proj_array = self.get_parameter('ort_proj').value

        # reshape ortogonal projection Matrix
        self.ort_proj = np.reshape(self.ort_proj_array, ((self.agent_dim,self.n_agents*self.agent_dim)))

        self.prop_gain = prop_gain
        self.deriv_gain = deriv_gain

    def evaluate_input(self, neigh_data):

        u = np.zeros(3)
        err_pos = np.zeros(3)
        err_vel = np.zeros(3)

        if not self.is_leader:
            i = self.agent_id
            dd = self.agent_dim
            N = self.n_agents
            for j, neigh_pose in neigh_data.items():
                err_pos = self.current_pose.position - neigh_pose.position 
                err_vel = self.current_pose.velocity - neigh_pose.velocity

                P_ij = self.ort_proj[:,j*dd:j*dd+N-1]
                u += - P_ij @ (self.prop_gain * err_pos + self.deriv_gain * err_vel)

        return u

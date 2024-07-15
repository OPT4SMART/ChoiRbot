import numpy as np
from .distributed_control import DistributedControlGuidance

class SimpleDIControl(DistributedControlGuidance):
    """
    Simple prop-deriv controller for Double Integrator dynamics 
    """
    def __init__(self, update_frequency: float, prop_gain: float=1.0, deriv_gain=5.0, 
                 pose_handler: str=None, pose_topic: str=None, input_topic = 'acceleration'):
        super().__init__(update_frequency, pose_handler, pose_topic, input_topic)

        goal = self.get_parameter('goal').value
        self.goal = np.array(goal)

        self.prop_gain = prop_gain
        self.deriv_gain = deriv_gain

    def control(self):

        # skip if position is not available yet
        if self.current_pose.position is None:
            return
        
        error = self.goal - self.current_pose.position 
        error_dot = - self.current_pose.velocity

        u = self.prop_gain * error + self.deriv_gain * error_dot

        self.send_input(u)
